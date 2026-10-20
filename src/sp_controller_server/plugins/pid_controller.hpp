#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  geometry_msgs::msg::TwistStamped makeZeroCmd() const;
  void computeLqrGains();
  std::size_t findClosestIndex(double x, double y) const;
  std::size_t lookaheadIndex(std::size_t closest, double x, double y) const;
  double remainingDistance(std::size_t closest, double x, double y) const;
  void pathTangent(std::size_t index, double & tx, double & ty) const;

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path global_plan_;
  std::string base_frame_id_;

  double q_pos_{8.0};
  double q_vel_{1.2};
  double r_acc_{0.22};
  double k_pos_{0.0};
  double k_vel_{0.0};
  double dt_{0.02};
  double lookahead_dist_{0.20};
  double max_speed_{1.70};
  double goal_tolerance_{0.10};
  double slowdown_dist_{0.80};

  std::size_t path_index_{0};
  mutable std::mutex plan_mutex_;
};

}

#endif
