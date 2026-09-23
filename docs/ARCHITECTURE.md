# A300 系统架构说明

> 版本：v1.0 ｜ 代码基线：`fa4f72b`（"可验证版，初步实现了自动刹车"）**+ 当前工作区未提交改动**
> （避障绕行逻辑、状态灯坐标补偿、键盘遥控等改动尚未提交，本文档以工作区实际内容为准）
> 目标平台：Ubuntu 24.04 LTS + ROS 2 Jazzy + Gazebo Harmonic
> 文档定位：描述 A300 智能轮椅的**目标分层架构**、**当前仓库实现与目标的映射关系**，以及**模块间的数据契约**。

---

## 1. 文档范围

本文档回答三个问题：

1. A300 系统的**目标架构**由哪几层构成，每层的职责边界在哪里；
2. 当前仓库 `a300-r5` 已经实现了哪些层、哪些层是缺口；
3. 各模块之间的**话题契约**（谁发布、谁订阅、语义约定）是什么。

算法细节、参数取值、状态机内部逻辑见 [`DESIGN.md`](./DESIGN.md)；缺陷与风险清单见 [`REVIEW.md`](./REVIEW.md)；实施计划见 [`ROADMAP.md`](./ROADMAP.md)。

---

## 2. 系统目标架构

采用**六层纵向分层**结构。核心设计不变量是：**`/cmd_vel` 是整车唯一的运动速度出口**，任何上游模块（摇杆、键盘、Nav2）都不得直接写入该话题。

```
┌──────────────────────────────────────────────────────────────────┐
│ L0  传感层  Sensors                                              │
│     LDS-E110-R-5 360° LiDAR ｜ 超声波（后向倒车） ｜ 轮式编码器     │
└───────────────────────────────┬──────────────────────────────────┘
                                │ /scan  /range/*  /joint_states
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│ L1  人机操控解析层  Teleop Parsing                               │
│     摇杆 X/Y ｜ 键盘 ｜ （未来）App / 语音                        │
│     归一化 + 死区 + 限幅  →  /cmd_vel_desired（期望速度）         │
└───────────────────────────────┬──────────────────────────────────┘
                                │ /cmd_vel_desired
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│ L2  局部自主避障层  Local Avoidance                              │
│     ① 障碍物检测   ② 左右空间分析   ③ 通行能力判断                │
│     ④ 绕行方向决策 ⑤ 自动转向 + 航向恢复                          │
└───────────────────────────────┬──────────────────────────────────┘
                                │ /cmd_vel_avoided（内部）
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│ L3  安全控制层  Safety Guard                                     │
│     限速 ｜ 急停 E-STOP ｜ 看门狗 ｜ 雷达异常 ｜ 故障分级降级       │
└───────────────────────────────┬──────────────────────────────────┘
                                │ /cmd_vel（唯一出口，已约束）
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│ L4  底盘运动控制层  Differential Kinematics                       │
│     v / ω  →  左轮角速度 ω_L ｜ 右轮角速度 ω_R                    │
└───────────────────────────────┬──────────────────────────────────┘
                                │ CAN 帧
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│ L5  执行层  CAN 底盘控制器                                        │
│     左电机驱动器 ｜ 右电机驱动器 ｜ 心跳与故障码回传                │
└──────────────────────────────────────────────────────────────────┘
```

**分层原则**

| 原则 | 说明 |
|---|---|
| 单向数据流 | 上层只能通过约定的输入话题影响下层，禁止跨层直接调用 |
| 单一速度出口 | L3 之前的任何话题都是"期望值"，只有 `/cmd_vel` 是"已授权值" |
| 故障向下收敛 | 任一层的异常最终必须表现为速度收敛到 0 或进入可预测的受限状态 |
| 传感器抽象 | 应用层只依赖 `sensor_msgs/LaserScan`、`sensor_msgs/Range`，不依赖厂商报文格式 |

---

## 3. 当前实现与目标架构的映射

对照上节分层，仓库现状如下：

| 层 | 目标职责 | 当前承载包 | 状态 | 说明 |
|---|---|---|---|---|
| L0 传感 | 360° LiDAR | `a300_lidar` + `a300_gazebo` | ✅ 已实现 | 真机封装 BlueSea 官方 `bluesea2` 驱动；仿真用 `gpu_lidar` 模拟 R-5 参数 |
| L0 传感 | 超声波倒车停障 | — | ❌ **缺失** | 仓库内无任何超声波节点、话题或配置 |
| L0 传感 | 轮式编码器 | Gazebo `DiffDrive` 内置 | ⚠️ 仅仿真 | 真机编码器接口未定义 |
| L1 操控解析 | 摇杆 → 期望速度 | `a300_joystick` | ✅ 已实现 | `joy_node` + `a300_joystick_mapper`（C++，含死区） |
| L1 操控解析 | 键盘 → 期望速度 | `a300_visualization` | ✅ 已实现 | `teleop_keyboard.py`，带硬限速，用于仿真演示 |
| L2 局部避障 | ①~⑤ 全流程 | `a300_obstacle_avoidance` | ⚠️ **部分实现** | 五步均已编码，但存在扇区语义缺陷（见 REVIEW P1-1） |
| L2 局部避障 | 超声波融合 | — | ❌ 缺失 | 避障判据目前仅来自 LiDAR 单平面 |
| L3 安全控制 | 限速 | `a300_obstacle_avoidance` | ✅ 已实现 | `clamp(linear, ±vmax)` / `clamp(angular, ±wmax)` |
| L3 安全控制 | 雷达异常 | `a300_obstacle_avoidance` | ⚠️ 部分 | `SCAN_STALE` 仅禁止前进，仍允许转向（REVIEW P0-4） |
| L3 安全控制 | 看门狗 | — | ❌ **缺失（P0）** | 期望速度无超时，上游断连后保持前进 |
| L3 安全控制 | 急停 E-STOP | — | ❌ **缺失（P0）** | 无服务/话题/锁存机制 |
| L4 运动学 | v/ω → 左右轮速度 | Gazebo `DiffDrive` 插件内部 | ❌ **未实现** | 仓库内无自主运动学节点，真机无对应实现 |
| L5 执行 | CAN 底盘控制器 | `a300_base_controller`（空壳） | ❌ **未实现（P1-8）** | 目录内仅有 README，无 `package.xml` / `CMakeLists.txt` |
| 辅助 | 状态可视化 | `a300_visualization` | ✅ 已实现 | 以 Gazebo 世界内彩色球体表示安全状态 |
| 辅助 | SLAM | `a300_slam` | ⚠️ 配置存在但不可运行 | 启动文件路径错误、`use_sim_time` 冲突（REVIEW P1-4/P1-5） |
| 辅助 | Nav2 | `a300_nav`（空壳） | ❌ 未实现 | 规划阶段即要求其输出必须经过 L3 |

**结论**：L1 与 L2 具备可演示的端到端闭环（仿真中已验证"接近墙壁 → 减速 → 绕行"），但 L3 的安全完整性、L0 的超声波冗余、L4/L5 的真机执行链路**均为空白**，当前系统属于**功能原型**，不具备上车条件。

---

## 4. 数据流与话题契约

### 4.1 话题清单

| 话题 | 消息类型 | QoS | 发布者 | 订阅者 | 语义 |
|---|---|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 10 | `joy_node` | `a300_joystick_mapper` | 原始摇杆轴值 [-1,1] |
| `/cmd_vel_desired` | `geometry_msgs/Twist` | 10 | `a300_joystick_mapper`、`teleop_keyboard`、*未来 Nav2* | `a300_safety_controller` | **期望速度**，未经过安全约束 |
| `/scan` | `sensor_msgs/LaserScan` | SensorDataQoS | `bluesea2`（真机）/ `ros_gz_bridge`（仿真） | `a300_safety_controller`、`slam_toolbox` | 360° 平面距离，`frame_id=laser_link` |
| `/odom` | `nav_msgs/Odometry` | 10 | `gz DiffDrive` / *未来 base controller* | `a300_safety_controller`、`status_light`、`slam_toolbox` | 里程计位姿与航向（航向用于避障恢复） |
| `/cmd_vel` | `geometry_msgs/Twist` | 10 | `a300_safety_controller` | `gz DiffDrive` / *未来 base controller* | **已授权速度，整车唯一运动出口** |
| `/a300/obstacle_state` | `std_msgs/String` | 10 | `a300_safety_controller` | `status_light`、*未来 HMI* | 状态字符串：`SAFE`/`WARNING`/`AVOIDING`/`STOP`/`SCAN_STALE` |
| `/joint_states` | `sensor_msgs/JointState` | 10 | `gz JointStatePublisher` | `robot_state_publisher` | 轮关节角度 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 10 | `ros_gz_bridge` / `robot_state_publisher` | 全系统 | TF 树 |

### 4.2 运行时序（仿真演示闭环）

```
joy_node ──/joy──► a300_joystick_mapper ──/cmd_vel_desired──┐
                                                            │
gz gpu_lidar ──/scan──► a300_gz_bridge ──/scan──► a300_safety_controller
                                                            │
gz DiffDrive ◄──/cmd_vel── a300_safety_controller ──/a300/obstacle_state──► status_light
      │                ▲
      └──/odom──────────┘（用于避障航向恢复与可视化定位）
```

### 4.3 TF 树

```
map
 └── odom                （SLAM 未启用时缺失，Nav2 阶段由 SLAM/AMCL 提供）
      └── base_link      （odom→base_link 由 DiffDrive 插件发布）
           ├── left_wheel
           ├── right_wheel
           ├── caster
           └── laser_link
```

> **注意**：当前 `odom` 与 Gazebo 世界系之间存在 180° 镜像关系（`odom.x = -world.x`, `odom.y = -world.y`），根因是避障节点在发布前对线速度取反（`a300_safety_controller.cpp:174`）。该处理会被 `status_light` 反向补偿。详见 [`REVIEW.md`](./REVIEW.md) P0-2。

### 4.4 坐标系与符号约定

| 量 | 正方向 | 说明 |
|---|---|---|
| `linear.x` | 车体前方（`base_link` +X） | 前进为正，倒退为负 |
| `angular.z` | 逆时针（俯视） | 左转为正，右转为负 |
| LiDAR 角度 | 与 ROS 标准一致，逆时针为正，0 指向车体 +X | `angle_min=-π`，`angle_max=+π` |
| 摇杆轴 | `axis[1]` → 线速度，`axis[0]` → 角速度 | 待实测确认 |

---

## 5. 部署形态

### 5.1 仿真链路（可用于回归验证）

| 环节 | 真机实现 | 仿真替代 |
|---|---|---|
| LiDAR | LDS-E110-R-5 + `bluesea2` 驱动 | Gazebo `gpu_lidar`：360°、400 采样、10 Hz、0.05–12 m |
| 底盘 | CAN 电机驱动器 | Gazebo `DiffDrive` 插件 |
| 里程计 | 编码器 + 运动学积分 | `DiffDrive` 内置 odom |
| 避障/安全 | **同一份 `a300_safety_controller`** | 同左 |

**关键设计点**：仿真与真机共用同一避障/安全节点，保证算法行为可迁移；差异被隔离在传感器与底盘驱动两端。

### 5.2 启动入口

| 脚本 | 用途 |
|---|---|
| `start_demo.sh` | 一键演示：终止残留进程 → 启动 Gazebo 仿真 → 启动状态灯 → 进入键盘遥控 |
| `a300_gazebo/launch/sim.launch.py` | 仿真完整链路（世界 + 模型 + 桥接 + 安全节点 + 生成） |
| `a300_bringup/launch/r5_safety.launch.py` | 真机：LiDAR 驱动 + 安全节点 |
| `a300_bringup/launch/teleop_safety.launch.py` | 真机：摇杆 + 安全节点 |

---

## 6. A300 开发模型几何参数

当前全部为**开发占位值**，未取自实测：

| 参数 | 取值 | 来源 | 备注 |
|---|---|---|---|
| 车体尺寸 | 0.75 × 0.62 × 0.75 m | `a300.urdf.xacro` | 占位 |
| 车轮半径 | 0.18 m | `a300.urdf.xacro` / `a300.gazebo.xacro` | 占位 |
| 轮距 | 0.55 m | 同上（关节 y = ±0.275） | 占位 |
| 整车质量 | 60 kg | `a300.urdf.xacro` | 占位 |
| LiDAR 安装位置 | x = 0.20 m, z = **0.85 m** | `a300.urdf.xacro` | 与 `README.md` 记载的 0.70 m **不一致**（REVIEW P1-6） |
| 万向轮位置 | x = −0.28 m, r = 0.08 m | `a300.urdf.xacro` | 占位 |

> 上述任一参数进入实测数据前，仿真得到的制动距离、绕行半径均不具有工程参考价值。

---

## 7. 安全边界声明

1. 本仓库整体定位为**开发验证平台**，未取得任何功能安全认证，不得直接用于载人产品。
2. 所有距离阈值、速度上限均为开发默认值，真机使用前必须完成：制动距离实测、通信延迟实测、LiDAR 安装位姿标定、急停回路验证。
3. 仿真与真机共用的避障节点，其行为在真机上的有效性依赖 L4/L5 的运动学与 CAN 链路的正确实现——该部分当前为空。

---

## 8. 相关文档

- [`DESIGN.md`](./DESIGN.md) —— 分层详细设计与算法规格
- [`REVIEW.md`](./REVIEW.md) —— 现状评审、缺陷清单与修复建议
- [`ROADMAP.md`](./ROADMAP.md) —— 里程碑、交付物与验收标准
- [`SETUP.md`](./SETUP.md) —— 环境搭建
- [`SIMULATION.md`](./SIMULATION.md) —— 仿真说明
- [`REAL_LIDAR.md`](./REAL_LIDAR.md) —— LDS-E110-R-5 接入说明
