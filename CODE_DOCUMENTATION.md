# EGO Planner Swarm — 代码结构文档

> **EGO Planner Swarm** 是一个基于 ROS 2 (Humble) 的多无人机（UAV）集群运动规划系统，使用 B 样条轨迹优化实现高效、安全的自主飞行规划。

---

## 目录

1. [项目概述](#1-项目概述)
2. [目录结构](#2-目录结构)
3. [系统架构](#3-系统架构)
4. [规划子系统（Planner）](#4-规划子系统planner)
   - 4.1 [plan_manage — 规划管理与状态机](#41-plan_manage--规划管理与状态机)
   - 4.2 [plan_env — 环境与地图表示](#42-plan_env--环境与地图表示)
   - 4.3 [path_searching — 全局路径搜索](#43-path_searching--全局路径搜索)
   - 4.4 [bspline_opt — B 样条轨迹优化](#44-bspline_opt--b-样条轨迹优化)
   - 4.5 [traj_utils — 轨迹工具与消息](#45-traj_utils--轨迹工具与消息)
   - 4.6 [drone_detect — 无人机间检测](#46-drone_detect--无人机间检测)
   - 4.7 [rosmsg_tcp_bridge — TCP 消息桥接](#47-rosmsg_tcp_bridge--tcp-消息桥接)
5. [仿真子系统（Simulator）](#5-仿真子系统simulator)
   - 5.1 [so3_quadrotor_simulator — 四旋翼动力学仿真](#51-so3_quadrotor_simulator--四旋翼动力学仿真)
   - 5.2 [so3_control — SO(3) 姿态控制](#52-so3_control--so3-姿态控制)
   - 5.3 [local_sensing — 局部深度感知仿真](#53-local_sensing--局部深度感知仿真)
   - 5.4 [map_generator — 随机障碍物地图生成](#54-map_generator--随机障碍物地图生成)
   - 5.5 [mockamap — 程序化地图生成](#55-mockamap--程序化地图生成)
   - 5.6 [fake_drone — 轻量无人机仿真](#56-fake_drone--轻量无人机仿真)
6. [工具包（Utils）](#6-工具包utils)
7. [ROS 2 话题接口](#7-ros-2-话题接口)
8. [自定义消息定义](#8-自定义消息定义)
9. [启动文件与配置参数](#9-启动文件与配置参数)
10. [关键算法说明](#10-关键算法说明)
11. [执行流程](#11-执行流程)
12. [关键数据结构](#12-关键数据结构)
13. [依赖库](#13-依赖库)

---

## 1. 项目概述

EGO Planner Swarm 实现了以下核心功能：

| 功能 | 描述 |
|------|------|
| **实时运动规划** | 基于 B 样条的轨迹优化，10 Hz 重规划频率 |
| **集群协同** | 多无人机共享轨迹，通过梯度优化实现避碰 |
| **局部感知** | 模拟深度相机，构建局部三维占据栅格地图 |
| **动态障碍物** | 预测运动障碍物，规避动态环境 |
| **全动力学仿真** | 基于 SO(3) 的完整四旋翼动力学模型 |
| **双模式地图** | 随机森林地图（默认）或程序化噪声地图 |

**技术栈：**
- **框架**：ROS 2 Humble
- **语言**：C++17（主体），Python（启动脚本）
- **构建系统**：ament_cmake
- **中间件**：CycloneDDS（须替换默认 FastDDS，见 `Readme.md`）

---

## 2. 目录结构

```
ego-planner-swarm/
├── LICENSE
├── Readme.md                          # 使用说明（中英双语）
├── CODE_DOCUMENTATION.md             # 本文档
└── src/
    ├── planner/                       # 运动规划核心模块
    │   ├── plan_manage/               # 规划管理器与有限状态机（FSM）
    │   ├── plan_env/                  # 环境表示（栅格地图、动态障碍物）
    │   ├── path_searching/            # 全局路径搜索（动态 A*）
    │   ├── bspline_opt/               # B 样条轨迹优化
    │   ├── traj_utils/                # 轨迹工具、可视化、自定义消息
    │   ├── drone_detect/              # 深度图无人机检测
    │   └── rosmsg_tcp_bridge/         # ROS 消息 TCP 桥接
    └── uav_simulator/                 # UAV 仿真环境
        ├── so3_quadrotor_simulator/   # SO(3) 四旋翼动力学仿真
        ├── so3_control/               # SO(3) 姿态控制器
        ├── local_sensing/             # 局部深度传感器仿真（PCL）
        ├── map_generator/             # 随机森林障碍物地图
        ├── mockamap/                  # Perlin 噪声程序化地图
        ├── fake_drone/                # 轻量级无人机测试节点
        └── Utils/                     # 通用工具包
            ├── quadrotor_msgs/        # 四旋翼自定义 ROS 消息
            ├── multi_map_server/      # 多分辨率地图服务
            ├── odom_visualization/    # 里程计可视化
            ├── pose_utils/            # 位姿变换工具
            ├── uav_utils/             # 通用 UAV 工具（几何、转换）
            ├── waypoint_generator/    # 航点序列生成
            └── cmake_utils/           # CMake 构建工具
```

---

## 3. 系统架构

### 整体架构图

```
┌──────────────────────────────────────────────────────────────────────┐
│                     EGO PLANNER NODE（规划节点）                      │
├──────────────────────────────────────────────────────────────────────┤
│                                                                       │
│   EGOReplanFSM（有限状态机）                                          │
│   ┌─────────┐   ┌──────────────┐   ┌─────────────┐                  │
│   │  INIT   │──▶│ WAIT_TARGET  │──▶│GEN_NEW_TRAJ │                  │
│   └─────────┘   └──────────────┘   └──────┬──────┘                  │
│                                           │                          │
│   ┌───────────────┐   ┌──────────────┐   │                          │
│   │EMERGENCY_STOP │◀──│  EXEC_TRAJ   │◀──┘                          │
│   └───────────────┘   └──────┬───────┘                              │
│                              │ 碰撞/时间触发                          │
│                    ┌─────────▼──────┐                               │
│                    │  REPLAN_TRAJ   │                               │
│                    └────────────────┘                               │
│                                                                      │
│   EGOPlannerManager（规划逻辑）                                       │
│   ├── GridMap（三维占据栅格地图）                                     │
│   ├── BsplineOptimizer（轨迹优化 + L-BFGS）                          │
│   │   └── AStar（A* 全局路径，用于初始化）                           │
│   ├── ObjPredictor（动态障碍物预测）                                  │
│   └── PlanningVisualization（RViz 可视化）                           │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
         ↕ ROS 2 话题（发布 / 订阅）
┌────────────────────────┐     ┌──────────────────────────────┐
│    仿真子系统           │     │         感知子系统            │
├────────────────────────┤     ├──────────────────────────────┤
│ so3_quadrotor_simulator│     │  local_sensing (PCL 渲染)    │
│ so3_control            │     │  drone_detect（无人机检测）   │
│ poscmd_2_odom          │     │  odom_visualization          │
├────────────────────────┤     ├──────────────────────────────┤
│ 输入：位置指令          │     │ 输出：深度图、点云、相机位姿  │
│ 输出：里程计状态        │     │ 输入：全局点云、里程计        │
└────────────────────────┘     └──────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                  多机协同（集群）                              │
├──────────────────────────────────────────────────────────────┤
│ Drone_0 发布 → /drone_0_planning/swarm_trajs                │
│ Drone_1 订阅 ← /drone_0_planning/swarm_trajs                │
│ Drone_1 发布 → /drone_1_planning/swarm_trajs                │
│ ...所有无人机通过共享轨迹实现碰撞规避                         │
└──────────────────────────────────────────────────────────────┘
```

### 各模块职责概览

| 模块 | 职责 | 关键输入 | 关键输出 |
|------|------|----------|----------|
| `plan_manage` | 规划协调、状态机 | 里程计、目标点 | B 样条轨迹 |
| `plan_env` | 地图管理、障碍物表示 | 深度图、点云 | 占据栅格、SDF |
| `path_searching` | 全局路径规划 | 起点、终点、栅格 | 全局路径点 |
| `bspline_opt` | 轨迹平滑与安全优化 | 控制点、代价函数 | 优化后控制点 |
| `traj_utils` | 轨迹数据结构与可视化 | B 样条参数 | 位置/速度/加速度 |
| `drone_detect` | 无人机间位姿估计 | 深度图 | 位姿误差 |
| `so3_quadrotor_simulator` | 四旋翼动力学 | SO(3) 指令 | 里程计 |
| `so3_control` | 位置到姿态指令转换 | 位置指令、里程计 | SO(3) 指令 |
| `local_sensing` | 深度传感器仿真 | 全局点云、相机位姿 | 深度图、局部点云 |
| `map_generator` | 随机障碍物生成 | 参数配置 | 全局点云 |

---

## 4. 规划子系统（Planner）

### 4.1 `plan_manage` — 规划管理与状态机

**路径**：`src/planner/plan_manage/`

该包是整个系统的核心，包含规划状态机（FSM）和规划管理器。

#### 主要文件

| 文件 | 大小 | 功能描述 |
|------|------|----------|
| `src/ego_planner_node.cpp` | ~1 KB | ROS 2 主节点入口，创建 `EGOReplanFSM` 实例 |
| `src/ego_replan_fsm.cpp` | ~31 KB | 规划 FSM 实现，处理状态转换与轨迹执行 |
| `src/planner_manager.cpp` | ~22 KB | 规划管理器，调用路径搜索与轨迹优化 |
| `src/traj_server.cpp` | ~8 KB | 轨迹服务器，从 B 样条中提取并发布位置指令 |
| `include/ego_planner/ego_replan_fsm.h` | — | FSM 类定义与状态枚举 |
| `include/ego_planner/planner_manager.h` | — | 规划管理器接口 |

#### 有限状态机（EGOReplanFSM）

```
           has_odom            has_target           plan OK
  INIT ──────────▶ WAIT_TARGET ──────────▶ GEN_NEW_TRAJ ──────▶ EXEC_TRAJ
                                                │ plan fail              │
                                       EMERGENCY_STOP         时间/碰撞触发
                                                          REPLAN_TRAJ ◀──┘
```

**FSM 状态说明：**

| 状态 | 触发条件 | 主要动作 |
|------|----------|----------|
| `INIT` | 节点启动 | 等待第一帧里程计数据 |
| `WAIT_TARGET` | 收到里程计 | 等待目标点（手动 / 预设航点）|
| `GEN_NEW_TRAJ` | 收到目标 | 调用全局规划 + B 样条优化，生成初始轨迹 |
| `EXEC_TRAJ` | 规划成功 | 执行轨迹，定期检查是否需要重规划 |
| `REPLAN_TRAJ` | 时间阈值 / 碰撞检测 | 从当前状态重新规划局部轨迹 |
| `EMERGENCY_STOP` | 规划连续失败 | 原地悬停，停止前进 |
| `SEQUENTIAL_START` | 集群模式 | 等待前序无人机完成，实现时序启动 |

**目标类型（TARGET_TYPE）：**
- `MANUAL_TARGET`：通过 RViz 手动指定目标
- `PRESET_TARGET`：使用预设航点序列（`advanced_param.launch.py` 中定义）
- `REFERENCE_PATH`：跟踪参考路径

#### 规划管理器（EGOPlannerManager）

```cpp
class EGOPlannerManager {
public:
    // 主规划接口
    bool reboundReplan(start_pt, start_vel, start_acc,
                       end_pt, end_vel, flag_polyInit, flag_randomPolyTraj);
    bool EmergencyStop(stop_pos);
    bool planGlobalTraj(start_pos, ..., end_pos, ...);
    bool planGlobalTrajWaypoints(start_pos, ..., waypoints, ...);
    bool checkCollision(int drone_id);

    // 主要数据成员
    PlanParameters pp_;            // 规划超参数
    LocalTrajData local_data_;     // 当前局部轨迹
    GlobalTrajData global_data_;   // 全局轨迹
    GridMap::Ptr grid_map_;        // 三维占据栅格
    SwarmTrajData swarm_trajs_buf_;// 集群轨迹缓存
};
```

**规划参数（PlanParameters）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_vel` | 2.0 m/s | 最大飞行速度 |
| `max_acc` | 3.0 m/s² | 最大加速度 |
| `planning_horizon` | 7.5 s | 规划时间跨度 |
| `control_points_distance` | 0.5 m | B 样条控制点间距 |
| `drone_id` | 0 | 无人机编号（集群中唯一） |

---

### 4.2 `plan_env` — 环境与地图表示

**路径**：`src/planner/plan_env/`

负责维护三维占据栅格地图，处理深度图投影和动态障碍物预测。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/plan_env/grid_map.h` | 三维栅格地图类定义（含占据概率、膨胀层） |
| `src/grid_map.cpp` | 深度图到点云的投影、光线投射更新占据概率 |
| `include/plan_env/raycast.h` | 快速光线投射算法（DDA 算法） |
| `src/raycast.cpp` | 碰撞检测的光线追踪实现 |
| `include/plan_env/obj_predictor.h` | 动态障碍物运动预测 |
| `src/obj_predictor.cpp` | 基于历史观测的多项式轨迹预测 |
| `src/obj_generator.cpp` | 生成动态障碍物测试轨迹 |
| `include/plan_env/linear_obj_model.hpp` | 线性运动模型（用于预测） |

#### GridMap 核心结构

```cpp
struct MappingParameters {
    Eigen::Vector3d map_origin_, map_size_;     // 地图原点与尺寸
    Eigen::Vector3i map_voxel_num_;             // 体素数量
    double resolution_;                          // 体素分辨率（0.1 m）
    double obstacles_inflation_;                 // 障碍物膨胀半径
    double cx_, cy_, fx_, fy_;                  // 相机内参
    double p_hit_, p_miss_, p_min_, p_max_;     // 占据概率参数
    double min_ray_length_, max_ray_length_;     // 光线长度限制
};
```

**更新流程：**
1. 订阅深度图与相机位姿
2. 使用相机内参将像素投影到三维空间
3. 对每条光线执行概率更新（Log-Odds 模型）
4. 障碍物膨胀以确保安全间距

---

### 4.3 `path_searching` — 全局路径搜索

**路径**：`src/planner/path_searching/`

实现动态 A* 算法，用于生成全局参考路径。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/path_searching/dyn_a_star.h` | 动态 A* 类定义，支持多种启发函数 |
| `src/dyn_a_star.cpp` | A* 搜索实现，返回路径点序列 |

#### 算法说明

**启发函数（Heuristic）：**
- 欧氏距离（Euclidean）
- 曼哈顿距离（Manhattan）
- 对角距离（Diagonal）

**核心数据结构：**

```cpp
struct GridNode {
    Eigen::Vector3i index;     // 三维栅格索引
    double gScore, fScore;     // A* g 值和 f 值
    GridNodePtr cameFrom;      // 路径回溯指针
    enum_state state;          // OPENSET / CLOSEDSET / UNDEFINED
};
```

**用途**：为 B 样条优化提供初始控制点，作为轨迹起点多项式初始化的基础。

---

### 4.4 `bspline_opt` — B 样条轨迹优化

**路径**：`src/planner/bspline_opt/`

系统的核心优化模块，将全局路径优化为平滑、安全、动力学可行的 B 样条轨迹。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/bspline_opt/bspline_optimizer.h` | 优化器类定义，含 `ControlPoints` 结构 |
| `src/bspline_optimizer.cpp` | 多代价项梯度优化实现 |
| `include/bspline_opt/uniform_bspline.h` | 均匀 B 样条曲线定义（De Boor 算法）|
| `src/uniform_bspline.cpp` | B 样条求值与微分 |
| `include/bspline_opt/gradient_descent_optimizer.h` | 梯度下降优化器封装 |
| `include/bspline_opt/lbfgs.hpp` | L-BFGS 准牛顿优化算法（头文件库）|

#### ControlPoints 数据结构

```cpp
class ControlPoints {
public:
    double clearance;                                    // 最小安全间距
    int size;                                            // 控制点数量
    Eigen::MatrixXd points;                              // 3×N 控制点矩阵
    vector<vector<Eigen::Vector3d>> base_point;          // 碰撞点（用于梯度方向）
    vector<vector<Eigen::Vector3d>> direction;           // 安全方向（单位向量）
    vector<bool> flag_temp;                              // 临时标志位
};
```

#### 优化代价函数

$$J = \alpha \cdot J_{\text{smooth}} + \beta \cdot J_{\text{collision}} + \gamma \cdot J_{\text{feasibility}} + \delta \cdot J_{\text{swarm}}$$

> 注：代码中权重参数以 `lambda1`～`lambda4` 命名，对应公式中的 $\alpha$、$\beta$、$\gamma$、$\delta$。

| 代价项 | 代码权重参数 | 描述 |
|--------|-------------|------|
| $J_{\text{smooth}}$（$\alpha$） | `lambda1` | 控制点二阶差分，最小化加速度变化 |
| $J_{\text{collision}}$（$\beta$） | `lambda2` | 障碍物斥力势能（基于 SDF 梯度）|
| $J_{\text{feasibility}}$（$\gamma$） | `lambda3` | 速度/加速度约束惩罚（超限时激活）|
| $J_{\text{swarm}}$（$\delta$） | `lambda4` | 集群无人机间碰撞规避 |

#### UniformBspline 核心方法

```cpp
class UniformBspline {
    // 构建：控制点 + 时间步长 + 样条阶数
    UniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

    // 求值
    Eigen::MatrixXd evaluate(double t);          // 在时刻 t 求位置
    UniformBspline getDerivative();              // 返回速度样条（一阶导数）

    // 可行性检查
    bool checkFeasibility(double& ratio, bool show_info);  // 检查速度/加速度约束

    // 时间重参数化
    static void lengthenTime(double ratio);      // 延长时间以满足约束
};
```

---

### 4.5 `traj_utils` — 轨迹工具与消息

**路径**：`src/planner/traj_utils/`

提供轨迹数据容器、多项式轨迹表示和 RViz 可视化接口。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/traj_utils/plan_container.hpp` | 轨迹数据容器（全局、局部、集群）|
| `include/traj_utils/polynomial_traj.h` | 分段多项式轨迹（用于全局规划）|
| `src/polynomial_traj.cpp` | 多项式轨迹求值与微分 |
| `include/traj_utils/planning_visualization.h` | RViz 可视化接口 |
| `src/planning_visualization.cpp` | 发布 Marker 到 RViz |
| `msg/Bspline.msg` | 自定义 B 样条消息 |
| `msg/MultiBsplines.msg` | 集群多轨迹消息 |
| `msg/DataDisp.msg` | 调试数据显示消息 |

#### 轨迹数据容器

```
GlobalTrajData（全局轨迹）
├── global_traj_: PolynomialTraj   // 全局多项式轨迹
├── local_traj_: vector<UniformBspline>  // 局部 B 样条（位置/速度/加速度）
├── global_duration_               // 总时长
└── local_start_time_, local_end_time_  // 当前局部段时间范围

LocalTrajData（当前执行轨迹）
├── position_traj_: UniformBspline  // 位置 B 样条
├── velocity_traj_: UniformBspline  // 速度 B 样条
├── acceleration_traj_: UniformBspline
├── start_time_: rclcpp::Time       // 轨迹开始时刻
└── duration_                       // 轨迹时长

SwarmTrajData（集群轨迹缓存）
└── trajs_: map<int, UniformBspline>  // 按无人机 ID 存储的邻居轨迹
```

---

### 4.6 `drone_detect` — 无人机间检测

**路径**：`src/planner/drone_detect/`

从深度图中检测其他无人机，估计相机位姿误差，用于提高多机感知精度。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/drone_detect/drone_detector.h` | 检测器类定义 |
| `src/drone_detector.cpp` | 将无人机模型投影到深度图，估计位姿误差 |
| `src/drone_detect_node.cpp` | ROS 2 节点入口 |
| `config/default.yaml` | 检测参数（像素比、误差球、无人机尺寸）|

#### 话题接口

| 类型 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `depth` | `sensor_msgs/Image` | 深度图输入 |
| 订阅 | `camera_pose` | `geometry_msgs/PoseStamped` | 相机位姿 |
| 订阅 | `drone_*/odom` | `nav_msgs/Odometry` | 其他无人机里程计 |
| 发布 | `new_depth` | `sensor_msgs/Image` | 修正后深度图 |
| 发布 | `camera_pose_error` | `geometry_msgs/PoseStamped` | 位姿误差（世界坐标系）|

---

### 4.7 `rosmsg_tcp_bridge` — TCP 消息桥接

**路径**：`src/planner/rosmsg_tcp_bridge/`

将 ROS 2 消息通过 TCP 转发，用于跨机通信或连接外部仿真器。

| 文件 | 说明 |
|------|------|
| `src/bridge_node.cpp` | TCP 服务端/客户端节点 |
| `launch/bridge.launch.py` | 桥接节点启动文件 |

---

## 5. 仿真子系统（Simulator）

### 5.1 `so3_quadrotor_simulator` — 四旋翼动力学仿真

**路径**：`src/uav_simulator/so3_quadrotor_simulator/`

实现完整的四旋翼 6 自由度动力学模型，采用 SO(3) 表示（避免万向锁）。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/so3_quadrotor_simulator/Quadrotor.h` | 四旋翼动力学模型（惯量、推力、力矩）|
| `include/ode/boost/numeric/odeint.hpp` | ODE 求解器（Boost.Odeint）|

#### 动力学模型

动力学方程使用 Boost.Odeint 进行数值积分：

```
状态向量：[位置(3), 速度(3), 姿态四元数(4), 角速度(3)]
输入：    [归一化推力, 力矩(3)]
输出：    里程计（位置 + 姿态 + 速度）
```

**话题接口：**

| 类型 | 话题 | 消息类型 |
|------|------|----------|
| 订阅 | `cmd/so3` | `quadrotor_msgs/SO3Command` |
| 发布 | `odom` | `nav_msgs/Odometry` |

---

### 5.2 `so3_control` — SO(3) 姿态控制

**路径**：`src/uav_simulator/so3_control/`

将位置指令转换为 SO(3) 姿态控制指令，实现级联位置-速度-姿态控制。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `include/so3_control/SO3Control.hpp` | SO(3) 控制器核心算法 |
| `src/so3_control_nodelet.cpp` | ROS 2 ComposableNode 节点 |
| `config/gains_hummingbird.yaml` | 位置/速度 PD 增益 |
| `config/corrections_hummingbird.yaml` | 角度修正参数 |

#### 控制律

$$\mathbf{F} = m \cdot (k_x \cdot e_x + k_v \cdot e_v + g \cdot \hat{z} + a_{\text{ref}})$$

$$\mathbf{M} = k_R \cdot e_R + k_\Omega \cdot e_\Omega$$

**话题接口：**

| 类型 | 话题 | 消息类型 |
|------|------|----------|
| 订阅 | `odom` | `nav_msgs/Odometry` |
| 订阅 | `cmd/pos` | `quadrotor_msgs/PositionCommand` |
| 发布 | `cmd/so3` | `quadrotor_msgs/SO3Command` |

---

### 5.3 `local_sensing` — 局部深度感知仿真

**路径**：`src/uav_simulator/local_sensing/`

使用 PCL 对全局点云进行相机视锥裁剪，模拟深度相机的局部感知输出。

#### 主要文件

| 文件 | 功能描述 |
|------|----------|
| `src/pcl_render_node.cpp` | 主渲染节点，对全局点云执行视锥剔除与深度渲染 |
| `config/camera.yaml` | 相机内参（fx, fy, cx, cy, 分辨率）|

#### 渲染流程

```
全局点云 ──▶ 变换到相机坐标系 ──▶ 视锥裁剪 ──▶ 投影到像素平面
      ──▶ 生成深度图 ──▶ 发布深度图 + 局部点云 + 相机位姿
```

**话题接口：**

| 类型 | 话题 | 消息类型 |
|------|------|----------|
| 订阅 | `global_map` | `sensor_msgs/PointCloud2` |
| 订阅 | `odom` | `nav_msgs/Odometry` |
| 发布 | `depth_image` | `sensor_msgs/Image` |
| 发布 | `local_cloud` | `sensor_msgs/PointCloud2` |
| 发布 | `camera_pose` | `geometry_msgs/PoseStamped` |

**配置参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sensing_horizon` | 5.0 m | 传感器最大探测距离 |
| `sensing_rate` | 30 Hz | 深度图发布频率 |
| `fx, fy` | 387.2 | 焦距（像素） |
| `cx, cy` | 321.0, 243.4 | 主点坐标 |

---

### 5.4 `map_generator` — 随机障碍物地图生成

**路径**：`src/uav_simulator/map_generator/`

生成包含圆柱体和随机多边形障碍物的三维随机森林地图，并以全局点云发布。

#### 配置参数

| 参数 | 说明 |
|------|------|
| `map/x_size`, `map/y_size`, `map/z_size` | 地图尺寸（m） |
| `map/resolution` | 点云分辨率（0.1 m） |
| `map/obs_num` | 随机障碍物数量 |
| `ObstacleShape/lower_rad`, `upper_rad` | 障碍物半径范围 |
| `ObstacleShape/lower_hei`, `upper_hei` | 障碍物高度范围 |
| `map/circle_num` | 圆柱障碍物数量 |
| `min_distance` | 无人机起始位置与障碍物最小间距 |

**发布话题**：`/map_generator/global_cloud`（`sensor_msgs/PointCloud2`）

---

### 5.5 `mockamap` — 程序化地图生成

**路径**：`src/uav_simulator/mockamap/`

使用 Perlin 噪声和分形算法生成程序化三维地图，作为随机森林地图的替代方案。

#### 地图类型

| 类型 | 启动文件 | 描述 |
|------|----------|------|
| `Maze2D` | `maze2d.launch.py` | 二维迷宫结构 |
| `Maze3D` | `maze3d.launch.py` | 三维迷宫结构 |
| `Perlin3D` | `perlin3d.launch.py` | Perlin 噪声三维地形 |
| `Post2D` | `post2d.launch.py` | 柱状二维障碍物 |

**使用方法**：`ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True`

---

### 5.6 `fake_drone` — 轻量无人机仿真

**路径**：`src/uav_simulator/fake_drone/`

不考虑完整动力学的轻量级无人机仿真节点，直接将位置指令转换为里程计，用于快速测试规划算法。

---

## 6. 工具包（Utils）

### `quadrotor_msgs` — 自定义四旋翼消息

**路径**：`src/uav_simulator/Utils/quadrotor_msgs/`

定义系统使用的所有自定义 ROS 2 消息类型。详见 [第 8 节](#8-自定义消息定义)。

### `multi_map_server` — 多分辨率地图服务

管理二维占据栅格和三维稀疏地图，用于多无人机的地图共享。

**自定义消息：**
- `MultiOccupancyGrid.msg`：多分辨率占据栅格列表
- `SparseMap3D.msg`：三维稀疏地图（体素坐标 + 占据值）
- `MultiSparseMap3D.msg`：多机器人稀疏地图集合
- `VerticalOccupancyGridList.msg`：垂直方向分层栅格

### `odom_visualization` — 里程计可视化

在 RViz 中以机器人模型、轨迹历史、速度向量、协方差椭球等形式显示里程计信息。

### `pose_utils` — 位姿变换工具

提供位姿四元数、欧拉角、变换矩阵之间的转换工具函数。

### `uav_utils` — 通用 UAV 工具

| 文件 | 功能 |
|------|------|
| `geometry_utils.h` | 三维几何运算（旋转、投影） |
| `converters.h` | ROS 消息与 Eigen 类型互转 |
| `utils.h` | 日志、断言、时间工具 |

### `waypoint_generator` — 航点生成

订阅用户交互事件，自动生成多点飞行任务的航点序列。

### `cmake_utils` — CMake 工具

包含项目通用的 CMake 查找模块和编译配置。

---

## 7. ROS 2 话题接口

### 规划节点话题（以 `drone_0` 为例，集群中以 `drone_N` 替换）

#### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/drone_0_visual_slam/odom` | `nav_msgs/Odometry` | 无人机里程计（位置、速度） |
| `/drone_0_planning/local_cloud` | `sensor_msgs/PointCloud2` | 局部点云（来自 local_sensing） |
| `/drone_0_planning/depth_image` | `sensor_msgs/Image` | 深度图 |
| `/drone_0_planning/camera_pose` | `geometry_msgs/PoseStamped` | 相机位姿 |
| `/goal` | `geometry_msgs/PoseStamped` | 手动目标点（来自 RViz） |
| `/drone_1_planning/swarm_trajs` | `traj_utils/MultiBsplines` | 邻居无人机轨迹（集群模式）|

#### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/drone_0_planning/bspline` | `traj_utils/Bspline` | 规划的 B 样条轨迹 |
| `/drone_0_planning/swarm_trajs` | `traj_utils/MultiBsplines` | 向其他无人机广播本机轨迹 |
| `/drone_0_planning/data_display` | `traj_utils/DataDisp` | 调试数据显示 |
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 位置指令（到控制器）|

### 仿真节点话题

| 话题名 | 消息类型 | 流向 | 说明 |
|--------|----------|------|------|
| `/drone_0_odom` | `nav_msgs/Odometry` | 仿真器→规划/控制 | 仿真里程计 |
| `/drone_0_cmd/pos` | `quadrotor_msgs/PositionCommand` | 控制器←轨迹服务器 | 位置指令 |
| `/drone_0_cmd/so3` | `quadrotor_msgs/SO3Command` | 控制器→仿真器 | 姿态指令 |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | 地图→local_sensing | 全局点云 |

---

## 8. 自定义消息定义

### traj_utils 消息

#### `Bspline.msg`
```
int32 drone_id          # 发布此轨迹的无人机 ID
int32 order             # B 样条阶数（通常为 3）
int64 traj_id           # 轨迹唯一 ID
builtin_interfaces/Time start_time  # 轨迹开始时刻

float64[] knots         # 节点向量
geometry_msgs/Point[] pos_pts  # 控制点（三维位置）

float64[] yaw_pts       # 偏航角控制点
float64 yaw_dt          # 偏航样条时间步长
```

#### `MultiBsplines.msg`
```
int32 drone_id_from     # 发送方无人机 ID
Bspline[] traj          # 包含多条 B 样条轨迹的数组
```

#### `DataDisp.msg`
```
std_msgs/Header header
float64 a               # 调试数据 a-e（可配置含义）
float64 b
float64 c
float64 d
float64 e
```

### quadrotor_msgs 消息

#### `SO3Command.msg`
```
std_msgs/Header header
geometry_msgs/Vector3 force        # 期望推力方向
geometry_msgs/Quaternion orientation  # 期望姿态四元数
float64[3] kr                      # 旋转增益
float64[3] kom                     # 角速度增益
quadrotor_msgs/AuxCommand aux      # 辅助指令
```

#### `PositionCommand.msg`
```
std_msgs/Header header
geometry_msgs/Point position       # 期望位置
geometry_msgs/Vector3 velocity     # 期望速度
geometry_msgs/Vector3 acceleration # 期望加速度
float64 yaw                        # 期望偏航角
float64 yaw_dot                    # 期望偏航角速率
float64[3] kx                      # 位置增益
float64[3] kv                      # 速度增益
uint32 trajectory_id               # 对应轨迹 ID
uint8 trajectory_flag              # 轨迹状态标志
```

---

## 9. 启动文件与配置参数

### 启动文件汇总

| 启动文件 | 路径 | 用途 |
|----------|------|------|
| `rviz.launch.py` | `plan_manage/launch/` | 启动 RViz 可视化 |
| `single_run_in_sim.launch.py` | `plan_manage/launch/` | 单机仿真（地图 + 规划 + 仿真）|
| `swarm.launch.py` | `plan_manage/launch/` | 10 机集群仿真 |
| `swarm_large.launch.py` | `plan_manage/launch/` | 大规模集群仿真 |
| `run_in_sim.launch.py` | `plan_manage/launch/` | 单无人机规划 + 仿真子启动 |
| `simulator.launch.py` | `plan_manage/launch/` | 仿真组件（控制器、传感器）|
| `advanced_param.launch.py` | `plan_manage/launch/` | 规划参数配置节点 |
| `drone_detect.launch.py` | `drone_detect/launch/` | 无人机检测节点 |
| `bridge.launch.py` | `rosmsg_tcp_bridge/launch/` | TCP 消息桥接 |

### 常用启动命令

```bash
# 1. 启动 RViz 可视化
ros2 launch ego_planner rviz.launch.py

# 2. 单机仿真（默认参数）
ros2 launch ego_planner single_run_in_sim.launch.py

# 3. 单机仿真（指定参数）
ros2 launch ego_planner single_run_in_sim.launch.py \
    use_mockamap:=True use_dynamic:=False

# 4. 10 机集群仿真
ros2 launch ego_planner swarm.launch.py

# 5. 大规模集群仿真
ros2 launch ego_planner swarm_large.launch.py
```

### 关键配置参数

#### 地图参数（`single_run_in_sim.launch.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map/x_size` | 26.0 m | 地图 X 方向尺寸 |
| `map/y_size` | 20.0 m | 地图 Y 方向尺寸 |
| `map/z_size` | 3.0 m | 地图 Z 方向尺寸 |
| `map/resolution` | 0.1 m | 体素分辨率 |
| `map/obs_num` | 250 | 随机障碍物数量 |
| `map/circle_num` | 250 | 圆柱障碍物数量 |
| `min_distance` | 1.2 m | 起点与障碍物最小间距 |

#### 规划参数（`advanced_param.launch.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_vel` | 2.0 m/s | 最大速度 |
| `max_acc` | 3.0 m/s² | 最大加速度 |
| `planning_horizon` | 7.5 s | 规划时间范围 |
| `flight_type` | 2 | 飞行模式（1=手动, 2=预设航点）|
| `drone_id` | 0 | 无人机编号 |

#### 相机内参（`advanced_param.launch.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fx` | 387.23 | X 方向焦距（像素）|
| `fy` | 387.23 | Y 方向焦距（像素）|
| `cx` | 321.05 | 主点 X 坐标（像素）|
| `cy` | 243.45 | 主点 Y 坐标（像素）|

#### 集群配置（`swarm.launch.py`）

| 参数 | 值 | 说明 |
|------|-----|------|
| 无人机数量 | 10 | 固定 10 架 |
| 地图尺寸 | 42×30×5 m | 集群地图（比单机更大）|
| `map/obs_num` | 200 | 障碍物数量 |
| 起点范围 | x=-20, y∈[-9,9] | 均匀分布在左侧 |
| 终点范围 | x=20, y∈[-9,9] | 均匀分布在右侧 |

---

## 10. 关键算法说明

### 10.1 Rebound Replan（弹性带重规划）

`reboundReplan()` 是核心规划函数，流程如下：

```
1. 多项式轨迹初始化
   ├── flag_polyInit=true:  使用 A* 路径初始化
   └── flag_polyInit=false: 使用随机多项式扰动

2. 从多项式轨迹采样得到 B 样条控制点
   └── 控制点间距 = control_points_distance 参数

3. B 样条优化（BsplineOptimizer::BsplineOptimizeTraj）
   ├── 计算障碍物 SDF 梯度
   ├── 计算集群无人机碰撞梯度
   ├── 检查动力学可行性约束
   └── 使用 L-BFGS 迭代优化控制点

4. 可行性检查与时间重参数化
   ├── checkFeasibility()：检查速度/加速度是否超限
   └── lengthenTime()：若超限则拉长时间轴

5. 更新轨迹信息并发布
```

### 10.2 B 样条优化代价梯度

**碰撞代价梯度**（针对控制点 $\mathbf{p}_i$）：

$$\nabla J_{\text{collision}} = -\text{dist}(\mathbf{p}_i) \cdot \nabla \text{SDF}(\mathbf{p}_i), \quad \text{when dist}(\mathbf{p}_i) < \text{clearance}$$

**平滑代价梯度**（弹性加速度）：

$$\nabla J_{\text{smooth}} = 2(\mathbf{p}_{i-1} - 2\mathbf{p}_i + \mathbf{p}_{i+1})$$

**集群避碰代价**：
$$\nabla J_{\text{swarm}} = -(\mathbf{p}_i - \mathbf{q}_j) \cdot f(\|\mathbf{p}_i - \mathbf{q}_j\|)$$

其中 $\mathbf{q}_j$ 是邻居无人机轨迹上与 $\mathbf{p}_i$ 时间对应的控制点。

### 10.3 概率占据地图更新（Log-Odds 模型）

$$L(x) = \log \frac{P(\text{occupied} | z)}{P(\text{free} | z)}$$

$$L_t(x) = L_{t-1}(x) + \begin{cases} \log \frac{p_{\text{hit}}}{1 - p_{\text{hit}}} & \text{命中} \\ \log \frac{p_{\text{miss}}}{1 - p_{\text{miss}}} & \text{未命中} \end{cases}$$

- 体素占据概率从 Log-Odds 值恢复：$P = \frac{e^L}{1 + e^L}$
- 当 $P > p_{\text{occ}}$ 时，体素标记为障碍物

### 10.4 SO(3) 姿态控制

级联 PD 控制器，基于旋转矩阵误差（避免欧拉角奇异性）：

$$e_R = \frac{1}{2}(R_{\text{des}}^T R - R^T R_{\text{des}})^{\vee}$$

$$\tau = -k_R e_R - k_\Omega e_\Omega + \Omega \times J\Omega$$

---

## 11. 执行流程

### 单机仿真完整流程

```
T=0s  ros2 launch ego_planner single_run_in_sim.launch.py
      │
      ├─▶ random_forest 发布全局点云到 /map_generator/global_cloud
      ├─▶ local_sensing 订阅点云，渲染深度图（30 Hz）
      ├─▶ so3_quadrotor_simulator + so3_control 启动动力学仿真
      └─▶ ego_planner_node（EGOReplanFSM）启动

T=0+  FSM: INIT
      └─▶ 等待第一帧里程计

T=0.1s FSM: WAIT_TARGET
       └─▶ 在 RViz 中发送目标点，或使用预设航点（flight_type=2）

T=0.2s FSM: GEN_NEW_TRAJ
       ├─▶ planGlobalTraj()：A* 搜索全局路径
       ├─▶ reboundReplan()：B 样条优化
       └─▶ 发布轨迹到 /drone_0_planning/bspline

T=0.3s FSM: EXEC_TRAJ
       └─▶ traj_server 以 50 Hz 从 B 样条提取位置指令

持续   规划循环（10 Hz）
       ├─▶ 检查是否需要重规划（时间阈值 / 碰撞检测 / 接近终点）
       └─▶ 触发 REPLAN_TRAJ → 重新优化局部轨迹

      控制循环（高频）
       ├─▶ so3_control 收到位置指令 → 计算 SO(3) 指令
       └─▶ 仿真器执行指令 → 更新里程计
```

### 集群仿真流程（10 机）

```
swarm.launch.py 启动 10 个 run_in_sim.launch.py 实例

Drone 0:
  ├─▶ 规划自身轨迹
  └─▶ 发布到 /drone_0_planning/swarm_trajs

Drone 1-9:
  ├─▶ 订阅 /drone_0_planning/swarm_trajs（以及其他无人机）
  ├─▶ 将邻居轨迹加入 BsplineOptimizer 的集群代价
  ├─▶ 优化时自动规避其他无人机
  └─▶ 发布自身轨迹

集群协同原理：
  每架无人机将邻居的 B 样条轨迹存入 SwarmTrajData，
  优化时计算到邻居轨迹的最近控制点距离，
  当距离 < swarm_clearance 时产生排斥梯度。
```

---

## 12. 关键数据结构

### 轨迹数据流图

```
A* 路径点
    │
    ▼
PolynomialTraj（多项式轨迹）
    │ 采样控制点
    ▼
ControlPoints（3×N Eigen 矩阵）
    │ L-BFGS 优化
    ▼
UniformBspline（均匀 B 样条）
├── position_traj_    （位置曲线）
├── velocity_traj_    （速度曲线 = 一阶导数）
└── acceleration_traj_（加速度曲线 = 二阶导数）
    │ 50 Hz 采样
    ▼
PositionCommand（位置+速度+加速度+偏航角）
    │
    ▼
SO3Command（推力 + 姿态四元数）
    │
    ▼
Quadrotor 动力学积分 → 里程计
```

### 系统时序关系

| 模块 | 频率 | 说明 |
|------|------|------|
| FSM 主循环 | 10 Hz | 状态检查与规划触发 |
| 安全检测 | 20 Hz | 碰撞预测，触发紧急停止 |
| 轨迹服务器 | 50 Hz | 从 B 样条提取位置指令 |
| 深度图渲染 | 30 Hz | local_sensing 发布感知数据 |
| 里程计 | 100 Hz | 动力学仿真器输出 |
| 地图更新 | 实时 | 订阅深度图后即时更新 |

---

## 13. 依赖库

| 库 | 版本要求 | 用途 |
|----|----------|------|
| **ROS 2** | Humble | 机器人中间件框架 |
| **CycloneDDS** | `ros-humble-rmw-cyclonedds-cpp` | ROS 2 DDS 实现（替代 FastDDS）|
| **Eigen3** | ≥3.3 | 线性代数（矩阵、向量运算）|
| **PCL** | ≥1.10 | 点云处理与渲染 |
| **OpenCV** | ≥4.0 | 深度图处理 |
| **VTK** | — | PCL 的可视化依赖 |
| **Boost** | — | ODE 求解器（odeint）|
| **Armadillo** | — | 数值计算（odom_visualization）|
| **CERES** | ≥2.0 | 非线性优化（local_sensing 对齐）|

### 安装依赖

```bash
# CycloneDDS（必须，替换默认 FastDDS）
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# PCL（含 VTK）
sudo apt install libpcl-dev

# 其他依赖通过 rosdep 安装
cd ego-planner-swarm
rosdep install --from-paths src --ignore-src -r -y
```

### 编译

```bash
cd ego-planner-swarm
colcon build --symlink-install
source install/setup.bash
```

---

*本文档由代码结构分析自动生成，覆盖 EGO Planner Swarm 全部 20 个 ROS 2 包。*
