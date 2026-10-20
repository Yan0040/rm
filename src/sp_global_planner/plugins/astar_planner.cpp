#include "astar_planner.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[param missing] node '" << node.get_name() << "' missing '" << name
      << "', fill it in yaml ros__parameters.";
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

}

namespace sp_global_planner {

void AStarPlanner::configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name)
{
  logger_ = node->get_logger();

  lethal_cost_  = require_param<int>(node, plugin_name + ".lethal_cost");
  cost_weight_  = require_param<double>(node, plugin_name + ".cost_weight");
  densify_ds_   = require_param<double>(node, plugin_name + ".densify_ds");
  smooth_iters_ = require_param<int>(node, plugin_name + ".smooth_iters");
  const std::string clear_name = plugin_name + ".clear_cost";
  try {
    if (node->has_parameter(clear_name)) {
      clear_cost_ = node->get_parameter(clear_name).get_value<int>();
    } else {
      clear_cost_ = node->declare_parameter<int>(clear_name, 25);
    }
  } catch (const std::exception &) {
    clear_cost_ = 25;
  }

  RCLCPP_INFO(logger_,
              "AStarPlanner configured: lethal_cost=%d clear_cost=%d cost_weight=%.3f densify_ds=%.3f smooth_iters=%d",
              lethal_cost_, clear_cost_, cost_weight_, densify_ds_, smooth_iters_);
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  map_ = costmap;
}

bool AStarPlanner::isBlocked(int8_t c) const
{
  if (c < 0) return true;
  return c >= lethal_cost_;
}

bool AStarPlanner::isTooClose(int8_t c) const
{
  if (c < 0) return true;
  return c >= clear_cost_;
}

double AStarPlanner::cellCostFactor(int8_t c) const
{
  const double cc = static_cast<double>(std::max(0, static_cast<int>(c)));
  return 1.0 + cost_weight_ * (cc / 100.0);
}

bool AStarPlanner::snapToFree(const nav_msgs::msg::OccupancyGrid & map, GridIndex & cell) const
{
  if (inBounds(map, cell.x, cell.y) && !isBlocked(map.data[toIndex(map, cell.x, cell.y)])) {
    return true;
  }
  const int W = static_cast<int>(map.info.width);
  const int H = static_cast<int>(map.info.height);
  for (int r = 1; r <= 12; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int x = cell.x + dx;
        const int y = cell.y + dy;
        if (x < 0 || y < 0 || x >= W || y >= H) {
          continue;
        }
        if (!isBlocked(map.data[toIndex(map, x, y)])) {
          cell.x = x;
          cell.y = y;
          return true;
        }
      }
    }
  }
  return false;
}

bool AStarPlanner::cellFreeWorld(double wx, double wy) const
{
  if (!map_) {
    return false;
  }
  GridIndex gi;
  if (!worldToGrid(*map_, wx, wy, gi)) {
    return false;
  }
  const auto & map = *map_;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int x = gi.x + dx;
      const int y = gi.y + dy;
      if (!inBounds(map, x, y)) {
        return false;
      }
      if (isTooClose(map.data[toIndex(map, x, y)])) {
        return false;
      }
    }
  }
  return true;
}

bool AStarPlanner::lineFree(double x0, double y0, double x1, double y1) const
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-6) {
    return cellFreeWorld(x0, y0);
  }
  const double step = map_ ? std::max(0.5 * static_cast<double>(map_->info.resolution), 0.02) : 0.02;
  const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    if (!cellFreeWorld(x0 + t * dx, y0 + t * dy)) {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path AStarPlanner::shortcutPath(const nav_msgs::msg::Path & raw) const
{
  nav_msgs::msg::Path out;
  out.header = raw.header;
  const auto & poses = raw.poses;
  if (poses.size() <= 2) {
    out.poses = poses;
    return out;
  }

  std::size_t i = 0;
  out.poses.push_back(poses.front());
  while (i + 1 < poses.size()) {
    std::size_t best = i + 1;
    for (std::size_t j = poses.size() - 1; j > i + 1; --j) {
      if (lineFree(
            poses[i].pose.position.x, poses[i].pose.position.y,
            poses[j].pose.position.x, poses[j].pose.position.y)) {
        best = j;
        break;
      }
    }
    out.poses.push_back(poses[best]);
    i = best;
  }
  return out;
}

nav_msgs::msg::Path AStarPlanner::densifyPath(const nav_msgs::msg::Path & raw, double ds) const
{
  nav_msgs::msg::Path out;
  out.header = raw.header;
  if (raw.poses.empty()) {
    return out;
  }
  out.poses.push_back(raw.poses.front());
  for (std::size_t i = 1; i < raw.poses.size(); ++i) {
    const auto & a = raw.poses[i - 1].pose.position;
    const auto & b = raw.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dist = std::hypot(dx, dy);
    const int n = dist > ds ? static_cast<int>(std::floor(dist / ds)) : 0;
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n + 1);
      geometry_msgs::msg::PoseStamped ps = raw.poses[i];
      ps.pose.position.x = a.x + t * dx;
      ps.pose.position.y = a.y + t * dy;
      ps.pose.position.z = 0.0;
      out.poses.push_back(ps);
    }
    out.poses.push_back(raw.poses[i]);
  }
  return out;
}

nav_msgs::msg::Path AStarPlanner::laplacianSmooth(const nav_msgs::msg::Path & raw, int iters) const
{
  nav_msgs::msg::Path cur = raw;
  if (cur.poses.size() < 3 || iters <= 0) {
    return cur;
  }
  for (int n = 0; n < iters; ++n) {
    nav_msgs::msg::Path nxt = cur;
    for (std::size_t i = 1; i + 1 < cur.poses.size(); ++i) {
      const auto & prev = cur.poses[i - 1].pose.position;
      const auto & mid = cur.poses[i].pose.position;
      const auto & next = cur.poses[i + 1].pose.position;
      const double nx = 0.75 * mid.x + 0.125 * (prev.x + next.x);
      const double ny = 0.75 * mid.y + 0.125 * (prev.y + next.y);
      if (lineFree(prev.x, prev.y, nx, ny) && lineFree(nx, ny, next.x, next.y)) {
        nxt.poses[i].pose.position.x = nx;
        nxt.poses[i].pose.position.y = ny;
      }
    }
    cur = std::move(nxt);
  }
  return cur;
}

nav_msgs::msg::Path AStarPlanner::smoothPath(const nav_msgs::msg::Path & raw) const
{
  if (raw.poses.size() < 3) {
    return raw;
  }
  auto shortcut = shortcutPath(raw);
  if (shortcut.poses.size() < 2) {
    return raw;
  }
  auto dense = densifyPath(shortcut, densify_ds_);
  auto smooth = laplacianSmooth(dense, smooth_iters_);
  auto out = densifyPath(smooth, densify_ds_);
  if (out.poses.size() < 2) {
    return raw;
  }
  out.poses.front() = raw.poses.front();
  out.poses.back() = raw.poses.back();
  return out;
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path path;
  if (!map_) {
    RCLCPP_ERROR(logger_, "No map received yet.");
    return path;
  }
  const auto& map = *map_;
  path.header = map.header;

  if (start.header.frame_id != map.header.frame_id || goal.header.frame_id != map.header.frame_id) {
    RCLCPP_WARN(logger_, "Frame mismatch: start=%s goal=%s map=%s",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map.header.frame_id.c_str());
  }

  GridIndex s, g;
  if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, s)) {
    RCLCPP_ERROR(logger_, "Start out of map bounds.");
    return path;
  }
  if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, g)) {
    RCLCPP_ERROR(logger_, "Goal out of map bounds.");
    return path;
  }

  const int W = static_cast<int>(map.info.width);
  const int H = static_cast<int>(map.info.height);
  const int N = W * H;

  auto idx = [&](int x, int y){ return y * W + x; };

  if (!snapToFree(map, s)) {
    RCLCPP_ERROR(logger_, "Start is in blocked cell (cost=%d).", (int)map.data[idx(s.x, s.y)]);
    return path;
  }
  if (!snapToFree(map, g)) {
    RCLCPP_ERROR(logger_, "Goal is in blocked cell (cost=%d).", (int)map.data[idx(g.x, g.y)]);
    return path;
  }

  struct AStarNode {
    int i;
    double f;
    double g;
    AStarNode(int i_in, double f_in, double g_in)
    : i(i_in), f(f_in), g(g_in) {}
  };
  struct Cmp {
    bool operator()(const AStarNode& a, const AStarNode& b) const { return a.f > b.f; }
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, Cmp> open;
  std::vector<double> gscore(static_cast<std::size_t>(N), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(N), -1);
  std::vector<std::uint8_t> closed(static_cast<std::size_t>(N), 0);

  auto h = [&](int x, int y) {
    const double dx = static_cast<double>(x - g.x);
    const double dy = static_cast<double>(y - g.y);
    return std::sqrt(dx * dx + dy * dy);
  };

  const int s_i = idx(s.x, s.y);
  const int g_i = idx(g.x, g.y);

  gscore[static_cast<std::size_t>(s_i)] = 0.0;
  open.emplace(s_i, h(s.x, s.y), 0.0);

  const int dxs[8] = {1,-1,0,0, 1,1,-1,-1};
  const int dys[8] = {0,0,1,-1, 1,-1,1,-1};

  bool found = false;

  while (!open.empty()) {
    const AStarNode cur = open.top();
    open.pop();

    if (closed[static_cast<std::size_t>(cur.i)]) continue;
    closed[static_cast<std::size_t>(cur.i)] = 1;

    if (cur.i == g_i) {
      found = true;
      break;
    }

    const int cy = cur.i / W;
    const int cx = cur.i - cy * W;

    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dxs[k];
      const int ny = cy + dys[k];
      if (!inBounds(map, nx, ny)) continue;

      const int ni = idx(nx, ny);
      if (closed[static_cast<std::size_t>(ni)]) continue;

      const int8_t c = map.data[ni];
      if (isBlocked(c)) continue;
      if (k >= 4) {
        if (isBlocked(map.data[idx(nx, cy)]) || isBlocked(map.data[idx(cx, ny)])) {
          continue;
        }
      }

      const double step = (k < 4) ? 1.0 : std::sqrt(2.0);
      const double factor = cellCostFactor(c);
      const double tentative = gscore[static_cast<std::size_t>(cur.i)] + step * factor;

      if (tentative < gscore[static_cast<std::size_t>(ni)]) {
        gscore[static_cast<std::size_t>(ni)] = tentative;
        parent[static_cast<std::size_t>(ni)] = cur.i;
        open.emplace(ni, tentative + h(nx, ny), tentative);
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return path;
  }

  std::vector<int> cells;
  int cur = g_i;
  while (cur != -1) {
    cells.push_back(cur);
    if (cur == s_i) break;
    cur = parent[static_cast<std::size_t>(cur)];
  }
  if (cells.back() != s_i) {
    RCLCPP_WARN(logger_, "Path reconstruction failed.");
    return path;
  }
  std::reverse(cells.begin(), cells.end());

  path.poses.reserve(cells.size());
  for (int ci : cells) {
    const int y = ci / W;
    const int x = ci - y * W;

    double wx = 0.0;
    double wy = 0.0;
    gridToWorld(map, x, y, wx, wy);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  const std::size_t raw_n = path.poses.size();
  path = smoothPath(path);
  RCLCPP_INFO(logger_, "A* path raw=%zu smoothed=%zu", raw_n, path.poses.size());
  return path;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sp_global_planner::AStarPlanner, sp_global_planner::GlobalPlannerPlugin)
