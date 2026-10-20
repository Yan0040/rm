#pragma once
#include "sp_global_planner/global_planner_plugin.hpp"
#include "sp_global_planner/grid_utils.hpp"
#include <optional>
#include <vector>

namespace sp_global_planner {

class AStarPlanner : public GlobalPlannerPlugin
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name) override;
  void setMap(const nav_msgs::msg::OccupancyGrid& costmap) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) override;

private:
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};

  int lethal_cost_{100};
  int clear_cost_{25};
  double cost_weight_{2.0};
  double densify_ds_{0.06};
  int smooth_iters_{3};

  bool isBlocked(int8_t c) const;
  bool isTooClose(int8_t c) const;
  double cellCostFactor(int8_t c) const;
  bool snapToFree(const nav_msgs::msg::OccupancyGrid & map, GridIndex & cell) const;
  bool cellFreeWorld(double wx, double wy) const;
  bool lineFree(double x0, double y0, double x1, double y1) const;
  nav_msgs::msg::Path shortcutPath(const nav_msgs::msg::Path & raw) const;
  nav_msgs::msg::Path densifyPath(const nav_msgs::msg::Path & raw, double ds) const;
  nav_msgs::msg::Path laplacianSmooth(const nav_msgs::msg::Path & raw, int iters) const;
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw) const;
};

}
