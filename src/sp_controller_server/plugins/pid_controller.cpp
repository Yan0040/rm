#include "pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

double hypot2(double dx, double dy)
{
  return std::hypot(dx, dy);
}

}

namespace pid_controller {

void PidController::computeLqrGains()
{
  const double r = std::max(r_acc_, 1e-6);
  k_pos_ = std::sqrt(q_pos_ / r);
  k_vel_ = std::sqrt(2.0 * k_pos_ + q_vel_ / r);
}

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");
  q_pos_ = require_param<double>(node_, plugin_name_ + ".q_pos");
  q_vel_ = require_param<double>(node_, plugin_name_ + ".q_vel");
  r_acc_ = require_param<double>(node_, plugin_name_ + ".r_acc");
  lookahead_dist_ = require_param<double>(node_, plugin_name_ + ".lookahead_dist");
  max_speed_ = require_param<double>(node_, plugin_name_ + ".max_speed");
  goal_tolerance_ = require_param<double>(node_, plugin_name_ + ".goal_tolerance");
  slowdown_dist_ = require_param<double>(node_, plugin_name_ + ".slowdown_dist");
  dt_ = require_param<double>(node_, plugin_name_ + ".dt");
  computeLqrGains();

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] LQR k_pos=%.3f k_vel=%.3f lookahead=%.2f vmax=%.2f dt=%.3f",
    plugin_name_.c_str(), k_pos_, k_vel_, lookahead_dist_, max_speed_, dt_);
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
  path_index_ = 0;
}

geometry_msgs::msg::TwistStamped PidController::makeZeroCmd() const
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;
  return cmd_vel;
}

std::size_t PidController::findClosestIndex(double x, double y) const
{
  const auto & poses = global_plan_.poses;
  if (poses.empty()) {
    return 0;
  }

  const std::size_t begin = path_index_ < poses.size() ? path_index_ : poses.size() - 1;
  std::size_t best = begin;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < poses.size(); ++i) {
    const double d = hypot2(poses[i].pose.position.x - x, poses[i].pose.position.y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    } else if (i > begin + 8 && d > best_d + 0.4) {
      break;
    }
  }
  return best;
}

std::size_t PidController::lookaheadIndex(
  std::size_t closest, double x, double y) const
{
  const auto & poses = global_plan_.poses;
  std::size_t target = closest;
  for (std::size_t i = closest; i < poses.size(); ++i) {
    const double d = hypot2(poses[i].pose.position.x - x, poses[i].pose.position.y - y);
    target = i;
    if (d >= lookahead_dist_) {
      break;
    }
  }
  return target;
}

double PidController::remainingDistance(std::size_t closest, double x, double y) const
{
  const auto & poses = global_plan_.poses;
  double remaining = hypot2(poses[closest].pose.position.x - x, poses[closest].pose.position.y - y);
  for (std::size_t i = closest + 1; i < poses.size(); ++i) {
    remaining += hypot2(
      poses[i].pose.position.x - poses[i - 1].pose.position.x,
      poses[i].pose.position.y - poses[i - 1].pose.position.y);
  }
  return remaining;
}

void PidController::pathTangent(std::size_t index, double & tx, double & ty) const
{
  const auto & poses = global_plan_.poses;
  const std::size_t j = (index + 1 < poses.size()) ? index + 1 : index;
  if (j == index) {
    if (index == 0) {
      tx = 1.0;
      ty = 0.0;
      return;
    }
    const auto & a = poses[index - 1].pose.position;
    const auto & b = poses[index].pose.position;
    tx = b.x - a.x;
    ty = b.y - a.y;
  } else {
    const auto & a = poses[index].pose.position;
    const auto & b = poses[j].pose.position;
    tx = b.x - a.x;
    ty = b.y - a.y;
  }
  const double n = hypot2(tx, ty);
  if (n < 1e-6) {
    tx = 1.0;
    ty = 0.0;
    return;
  }
  tx /= n;
  ty /= n;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);

  if (global_plan_.poses.empty()) {
    return makeZeroCmd();
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;

  path_index_ = findClosestIndex(x, y);
  const auto & goal = global_plan_.poses.back().pose.position;
  const double dist_goal = hypot2(goal.x - x, goal.y - y);
  const double remaining = remainingDistance(path_index_, x, y);

  if (dist_goal <= goal_tolerance_ || remaining <= goal_tolerance_) {
    return makeZeroCmd();
  }

  const auto & closest_pose = global_plan_.poses[path_index_].pose.position;

  double tdx = 0.0;
  double tdy = 0.0;
  pathTangent(path_index_, tdx, tdy);
  const double nx = -tdy;
  const double ny = tdx;

  const double e_lat = (closest_pose.x - x) * nx + (closest_pose.y - y) * ny;
  const double v_lat = velocity.linear.x * nx + velocity.linear.y * ny;
  const double k_lat = k_pos_ / std::max(k_vel_, 1e-6);
  const double k_damp = std::min(0.55, 0.18 * k_vel_);
  double v_lat_cmd = k_lat * e_lat - k_damp * v_lat;
  v_lat_cmd = std::max(-0.70, std::min(0.70, v_lat_cmd));

  // 直道吃满切向前馈；只有急弯和终点附近降速
  double speed_limit = max_speed_;
  if (remaining < slowdown_dist_) {
    speed_limit = std::max(0.25, max_speed_ * remaining / slowdown_dist_);
  }
  speed_limit /= (1.0 + 1.8 * std::fabs(e_lat));

  const auto & poses = global_plan_.poses;
  std::size_t ahead = path_index_;
  double acc = 0.0;
  while (ahead + 1 < poses.size() && acc < 0.45) {
    acc += hypot2(
      poses[ahead + 1].pose.position.x - poses[ahead].pose.position.x,
      poses[ahead + 1].pose.position.y - poses[ahead].pose.position.y);
    ++ahead;
  }
  double tdx2 = tdx;
  double tdy2 = tdy;
  pathTangent(ahead, tdx2, tdy2);
  const double align = tdx * tdx2 + tdy * tdy2;
  if (align < 0.92) {
    speed_limit *= std::max(0.50, 0.55 + 0.45 * align);
  }

  const double lat_cap = std::min(std::fabs(v_lat_cmd), speed_limit);
  v_lat_cmd = std::copysign(lat_cap, v_lat_cmd);
  const double tang_cap = std::sqrt(
    std::max(0.0, speed_limit * speed_limit - v_lat_cmd * v_lat_cmd));

  const double vx_map = tang_cap * tdx + v_lat_cmd * nx;
  const double vy_map = tang_cap * tdy + v_lat_cmd * ny;

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  geometry_msgs::msg::TwistStamped cmd_vel = makeZeroCmd();
  cmd_vel.twist.linear.x = c * vx_map + s * vy_map;
  cmd_vel.twist.linear.y = -s * vx_map + c * vy_map;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
