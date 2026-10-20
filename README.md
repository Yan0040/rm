# 迷宫一次点击导航

在给定迷宫上完成一次点击导航：固定起点 `(0.90, 0.90)`，对终点 `(14.10, 14.10)` 发布一次目标，A\* 规划全局路径，控制器跟踪到终点。

仿真器、行为树、pluginlib 框架未改。必做两项：

1. 对齐话题 / 坐标系 / 插件参数，使工程能在迷宫上跑通。
2. 实现路径跟踪控制器。

加分两项：

1. A\* 轨迹平滑（短路 + 插密 + 带碰撞检查的拉普拉斯）。
2. 切向前馈 + 横向 LQR（pluginlib 类名仍是 `PidController`）。

## 环境

Ubuntu 22.04 + ROS 2 Humble。

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-pluginlib \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-srvs \
  ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev \
  libopencv-dev \
  libyaml-cpp-dev

pip3 install --user pygame numpy Pillow
```

## 怎么跑

```bash
cd ~/Desktop/sp_vision_tutorial_27
colcon build --symlink-install
source install/setup.bash
bash bash/sim_nav_all_start.sh
```

仿真窗口 `Sentry Move: ALLOW` 保持绿色。RViz Fixed Frame 选 `map`。不要拖起点、不要手点迷宫、不要手动发 `cmd_vel`。发一次终点：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 14.1, y: 14.1, z: 0.0}, orientation: {w: 1.0}}}"
```

绿线来自下层行为树调用 `make_plan`，不是 `pub` 本身。成功时 `sp_nav` 终端会出现 `New goal_pose`、`Plan computed`、`A* path`。车沿走廊走到右上角后停。

## 改了什么

### 1. 参数 `src/sp_nav_bringup/config/nav_params.yaml`

话题 / 坐标系 / 插件名与仿真、RViz、pluginlib 对齐，没有改仿真 yaml。

| 节点 | 关键项 | 说明 |
|---|---|---|
| planner_server | `/global_costmap`、`/local_costmap/costmap`、`/global_path` | ESDF、仿真局部图、RViz Path |
| planner_server | `plugin_name: AStar`，`plugin_type: sp_global_planner/AStarPlanner` | yaml 嵌套块名 + plugin_description.xml |
| AStar | `lethal_cost: 100`，`clear_cost: 25`，`cost_weight: 12.0` | 搜索只挡墙；短路/平滑禁止贴膨胀 |
| esdf_map_publisher | 半径 / 边距 / d_safe / w | 与 `sim_robot.yaml` 对齐 |
| controller_server | `/global_path`、`/Odometry`、`lidar_odom`、`/sentry/cmd_vel` | 规划器输出、仿真里程计、仿真速度话题 |
| PidController | `q_pos / q_vel / r_acc / dt` | 双积分器 LQR 的 Q、R 和离散步长 |

### 2. 轨迹平滑 `src/sp_global_planner/plugins/astar_planner.cpp`

A\* 仍 8 连通搜格子。对角移动时若左右或上下邻格是墙，斜边丢掉，避免穿墙角。搜路只把 `cost >= lethal_cost` 当障碍。`createPlan` 返回前对折线做后处理：

1. **短路**：从当前点尽量连最远且不穿障的点，去掉格子锯齿。
2. **插密**：按 `densify_ds`（0.06 m）加密。
3. **拉普拉斯平滑**：中间点略拉向前后邻点；新位置与前后点的连线必须仍在自由格，否则该点不移动。

平滑后再插密一次，并强制起点、终点与原 A\* 一致。短路和平滑用 `clear_cost`（25）加 3×3 邻域，比搜路更严，绿线走走廊中间、拐角不贴墙。

### 3. LQR 控制器 `src/sp_controller_server/plugins/pid_controller.cpp`

仿真是全向底盘，只吃车体系 `linear.x/y`。路径跟踪拆成两个轴：

- **切向**：沿路径切向发速度。直道接近 `max_speed`（1.70）；横向偏了、前方约 0.45 m 内拐弯、或剩余弧长进入 `slowdown_dist` 时降速。
- **横向**：误差是车到路径最近点的法向投影。对横向位置、横向速度用双积分器无穷时域 LQR；限幅时先留横向纠偏，再用余量填切向。

取 \(Q=\mathrm{diag}(q_\mathrm{pos},q_\mathrm{vel}),\ R=r_\mathrm{acc}\)，闭式增益：

\[
k_p=\sqrt{q_\mathrm{pos}/R},\qquad k_v=\sqrt{2k_p+q_\mathrm{vel}/R}
\]

横向速度指令：

\[
v_\perp = (k_p/k_v)\,e_\perp - k_\mathrm{damp}\,v_\perp
\]

其中 \(k_\mathrm{damp}=\min(0.55,\,0.18\,k_v)\)。合成后再限幅，用当前 yaw 转到车体。进入 `goal_tolerance` 输出 0。没有积分项，避免弯道里攒误差贴墙。

## 调参

- 绿路径仍贴墙：加大 `AStar.cost_weight`，或略降 `clear_cost`。
- 起点报 blocked / 没有绿线：`clear_cost` 太大，往 40 回调。
- 横向晃：加大 `r_acc`，或略降 `max_speed`。
- 走得慢：略增 `max_speed`（仿真上限 2.0）。直道应接近 1.7，只有急弯降速。
- 冲终点：加大 `slowdown_dist`。

## 可视化与 Git

`src/sp_nav_bringup/rviz/rviz.rviz` 已订阅 `/global_costmap`、`/global_path`、`/plan_markers`。`.gitignore` 忽略 `build/`、`install/`、`log/`。
