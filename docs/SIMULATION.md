# A300 仿真说明

> 本文件描述当前仿真链路的**实际状态**。分层架构与话题契约见 [`ARCHITECTURE.md`](./ARCHITECTURE.md)，算法规格见 [`DESIGN.md`](./DESIGN.md)。

---

## 1. 目标

仿真链路的设计目标是**在传感器与底盘两端替换真机硬件，而中间件与算法完全复用**：

| 环节 | 真机 | 仿真 |
|---|---|---|
| LiDAR | LDS-E110-R-5 + BlueSea `bluesea2` 驱动 | Gazebo `gpu_lidar`（参数对齐 R-5 手册） |
| 底盘 | CAN 电机驱动器 | `gz-sim-diff-drive-system` 插件 |
| 里程计 | 编码器 + 差速运动学积分 | `DiffDrive` 内置 odom |
| **避障 / 安全** | **同一份 `a300_safety_controller`** | **同左** |

因此仿真可用于**算法行为回归**，但不可用于**制动距离 / 停车余量标定**——后者必须使用实测的车轮几何、LiDAR 安装位姿与执行延迟（见 [`ROADMAP.md`](./ROADMAP.md) M3）。

---

## 2. 测试世界（`a300_gazebo/worlds/a300_test.sdf`）

| 实体 | 说明 |
|---|---|
| `ground` | 50 × 50 m 静态平面，重力 −9.81 m/s² |
| `wall_ahead` | 0.2 × 1.5 × 1.5 m 静态墙体，位于 (1.2, 0, 0.75)。用于验证"接近 → 减速 → 绕行 → 航向恢复" |
| `status_green` / `status_yellow` / `status_orange` / `status_red` | 半径 0.30 m 的彩色球体，初始隐藏于 (50, 50, 1.0)，由 `status_light` 节点移动到机器人上方表示当前安全状态 |

仿真插件：`Physics`、`UserCommands`、`SceneBroadcaster`、`Sensors`（渲染引擎 `ogre2`）。

---

## 3. 仿真 LiDAR 参数

| 参数 | 取值 | R-5 手册值 | 一致性 |
|---|---|---|---|
| 水平视场 | 360°（−π ~ +π） | 360° | ✅ |
| 采样数 | 400 | — | 等效角分辨率约 0.9° |
| 角分辨率 | 约 0.9° | 0.9° | ✅ |
| 更新率 | 10 Hz | 典型 10 Hz | ✅ |
| 最小距离 | 0.05 m | — | ✅ |
| 最大距离 | 12.0 m | — | ✅ |
| 帧 ID | `laser_link` | — | ✅ |

> 上述为**仿真参数**，不构成对实物标定的替代。

---

## 4. 机器人模型

`a300_description/urdf/a300.urdf.xacro`（仿真时通过 `use_sim:=true` 引入 `a300.gazebo.xacro`）。

| 部件 | 参数 |
|---|---|
| `base_link` | 0.75 × 0.62 × 0.75 m 箱体，质心 z = 0.40 m，质量 60 kg |
| `left_wheel` / `right_wheel` | r = 0.18 m，厚 0.05 m，质量 2.0 kg，关节 y = ±0.275 m |
| `caster` | r = 0.08 m 球体，位于 x = −0.28 m |
| `laser_link` | 位于 (0.20, 0, 0.85)，即**高于车体顶部**（车体顶面 z = 0.775 m） |

**全部为开发占位值。** 详见 [`REVIEW.md`](./REVIEW.md) P1-6 与 [`ROADMAP.md`](./ROADMAP.md) M3.2。

---

## 5. 话题桥接（`ros_gz_bridge`）

| 话题 | 方向 |
|---|---|
| `/clock`、`/scan`、`/odom`、`/tf`、`/joint_states` | Gazebo → ROS 2 |
| `/cmd_vel` | ROS 2 → Gazebo |

无双向桥接，不存在指令回环风险。

---

## 6. 已知行为特征与限制

### 6.1 朝向一致性（**待修正**）

当前仿真中 `/cmd_vel` 与 `/cmd_vel_desired` 的线速度**符号相反**，`odom` 系与世界系呈 180° 镜像。原因是安全节点在发布前对线速度取反（`a300_safety_controller.cpp:174`），属对 URDF 车轮关节轴向问题的补偿。`status_light` 需反向补偿坐标才能正确定位。

该行为必须按 [`ROADMAP.md`](./ROADMAP.md) M2 修正后才能接入 Nav2 或真机底盘。详见 [`REVIEW.md`](./REVIEW.md) P0-2。

### 6.2 无里程计噪声模型

当前 `DiffDrive` 输出的 odom 为真值，**无噪声、无轮滑**。因此仿真中的 SLAM 表现会显著优于真机实际表现，不能据此评估 SLAM 鲁棒性。

### 6.3 单平面感知

仿真 LiDAR 与真机同为单平面扫描，**无法探测低于或高于扫描平面的障碍物**。这正是需要引入超声波倒车停障的原因（见 [`DESIGN.md`](./DESIGN.md) §6）。

---

## 7. 启动与验证

```bash
ros2 launch a300_gazebo sim.launch.py
```

一键演示（含状态灯与键盘遥控）：

```bash
bash start_demo.sh
```

启动后依次拉起：

1. Gazebo 测试世界（`-r` 自动运行）
2. `robot_state_publisher`
3. `ros_gz_bridge`（上述话题桥接）
4. `a300_safety_controller`（避障 + 安全约束）
5. 机器人模型生成（延迟 2 s，落点 z = 0.20）
6. `status_light`（由 `start_demo.sh` 单独拉起）
7. `teleop_keyboard`（前台运行）

**验证命令**

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic hz /scan              # 期望约 10 Hz
ros2 topic echo /odom
ros2 topic echo /cmd_vel_desired
ros2 topic echo /cmd_vel
ros2 topic echo /a300/obstacle_state
ros2 run tf2_tools view_frames
```

**期望行为**：状态球依次变为绿 → 黄 → 橙，机器人减速并自主绕开 `wall_ahead`，随后回到原航向，状态回到绿。若两侧均无余量则转为红灯并停车。

---

## 8. 后续仿真增强项

1. 里程计噪声与轮滑模型（用于真实感 SLAM 测试）
2. 更丰富的测试场景：窄通道、L 形转角、斜向障碍、两面夹持
3. 后向障碍物与台阶模型（配合超声波倒车停障验证）
4. rosbag 自动化回归脚本（录制 `/scan`、`/cmd_vel`、`/a300/obstacle_state`）

---

## 相关文档

- [`ARCHITECTURE.md`](./ARCHITECTURE.md) —— 分层架构与话题契约
- [`DESIGN.md`](./DESIGN.md) —— 算法规格与参数表
- [`REVIEW.md`](./REVIEW.md) —— 缺陷清单
- [`ROADMAP.md`](./ROADMAP.md) —— 里程碑与验收标准
