# A300 现状评审与缺陷清单

> 版本：v1.0 ｜ 评审对象：`a300-r5` 工作区
> 代码基线：提交 `fa4f72b`（"可验证版，初步实现了自动刹车"）**+ 当前未提交的工作区改动**
> 评审范围：`src/` 全部包、`docs/`、`README.md`、`start_demo.sh` 及构建/启动配置
> 配套文档：[`ARCHITECTURE.md`](./ARCHITECTURE.md)、[`DESIGN.md`](./DESIGN.md)、[`ROADMAP.md`](./ROADMAP.md)

---

## 0. 评审结论摘要

当前仓库是一个**可运行的仿真功能原型**：从摇杆/键盘输入，经 LiDAR 感知与反应式避障，到 Gazebo 差速底盘，形成完整闭环，并具备"接近障碍 → 减速 → 自主绕行 → 航向恢复"的行为验证。这一部分的核心算法设计思路清晰、分层意图明确。

但相对于目标架构（六层安全链路），存在**四个 P0 级安全缺口**，且**真机执行链路（运动学 + CAN）完全空白**。按现状直接上车不具备可行性。

| 等级 | 数量 | 含义 |
|---|---|---|
| **P0** | 4 | 安全相关，可导致失控或无法保障人身安全，上车前必须修复 |
| **P1** | 8 | 功能缺陷或设计不一致，影响正确性与可维护性 |
| **P2** | 16 | 工程质量、文档一致性、可测试性问题 |

---

## 1. P0 级缺陷（安全阻断项）

### P0-1 期望速度无看门狗，上游断连后持续运动

**位置**：`src/a300_obstacle_avoidance/src/a300_safety_controller.cpp:66-68, 83-84`

**现象**：

```cpp
void cmdCb(const geometry_msgs::msg::Twist::SharedPtr c){
    std::lock_guard<std::mutex> l(cmd_m_); desired_=*c;      // 仅缓存，无时间戳
}
```

`loop()` 以 50 Hz 持续重发 `desired_` 的最后一次取值。上游节点（`a300_joystick_mapper`、`teleop_keyboard`、未来的 Nav2）一旦异常退出、崩溃或 ROS 通信中断，`desired_` 将**永久保持非零值**，机器人持续前进直至撞上障碍物或被 `stop_distance` 拦下。

**影响**：这是**安全链路上最严重的单点缺陷**。摇杆无线断连、节点崩溃、DDS 抖动都会直接引发不受控运动。

**修复**：为 `/cmd_vel_desired` 增加接收时间戳，超过 `desired_timeout`（建议 0.30 s）即输出零速。同时将全部 L1 模块的发布模型由**事件驱动**改为**周期驱动**（详见 `DESIGN.md` §4.3）。

---

### P0-2 `/cmd_vel` 发布前线速度取反，语义污染安全节点

**位置**：`src/a300_obstacle_avoidance/src/a300_safety_controller.cpp:174`

```cpp
out.linear.x = -out.linear.x;   // 驱动方向修正
safe_pub_->publish(out);
```

**现象**：安全节点对外发布的 `/cmd_vel` 与输入 `/cmd_vel_desired` **符号相反**。`/cmd_vel` 的 `linear.x > 0` 在语义上已不再是"前进"。

**连带影响**：

1. `src/a300_visualization/a300_visualization/status_light.py` 必须反向补偿坐标才能正确定位（`self.world_x = -p.position.x`），代码注释已记录该现象；
2. Gazebo 世界系与 `odom` 系呈 180° 镜像；
3. **与 Nav2 语义冲突**：Nav2 的 `controller_server` 输出的 `linear.x > 0` 表示前进，若直接接入，链路中多一次符号翻转将导致反向运动；
4. **真机致命**：L4 运动学节点按标准约定解算，会得到与预期相反的轮速。

**根因判断**：疑为 URDF 车轮关节旋转轴定义不当（`a300.urdf.xacro` 中 `left_wheel_joint`/`right_wheel_joint` 的 `rpy="1.5708 0 0"` 与 `axis="0 0 1"` 组合），导致关节正方向旋转产生反向行驶。详见 `DESIGN.md` §7。

**修复**：在 URDF 层面修正关节轴定义，随后**同时删除**：
- `a300_safety_controller.cpp:174` 的取反语句；
- `status_light.py` 中的坐标取负补偿。

在修正前，任何接入 Nav2 或真机底盘的工作都应视为**被阻断**。

---

### P0-3 无急停（E-STOP）通路

**位置**：全仓库范围。检索 `estop` / `e_stop` / `emergency` 均无结果。

**现象**：系统不具备任何**可锁存、可显式释放**的急停机制。现有 `STOP` 状态是**基于障碍距离的自动行为**，障碍消失即自动恢复行驶，不具备"人为强制停机"的语义。

**影响**：载人场景下的基本安全要求无法满足。任何测试异常（算法震荡、传感器误检、机构卡滞）都缺少最终的人工干预手段。

**修复**：新增 `std_srvs/SetBool` 服务 `/a300/estop`，实现锁存语义（详见 `DESIGN.md` §4.4）。急停触发后必须显式释放，禁止因障碍消失而自动恢复。

---

### P0-4 雷达失效时仍允许转向，降级为 fail-degraded

**位置**：`a300_obstacle_avoidance/src/a300_safety_controller.cpp:90-92`

```cpp
if((now()-t).seconds()>0.30){
    if(out.linear.x>0) out.linear.x=0;      // 仅禁止前进
    state="SCAN_STALE";
}
```

**现象**：LiDAR 数据失效（超时 0.30 s）时，只禁止纵向前进速度，**角速度不受任何限制**。机器人可能在完全无环境感知的状态下以最高 1.00 rad/s 持续旋转，或执行上游的原地转向指令。

**影响**：不满足 fail-safe 原则——故障状态下的行为应当是**可预测且能量受限**的，而非保留部分运动能力。

**修复**：将雷达失效归入 E2 级故障，默认策略下同时限制角速度（`DESIGN.md` §4.5），并提供 `scan_stale_policy` 参数供现场在 `stop_all` / `no_forward` / `allow_turn` 之间选择，真机建议 `stop_all`。

---

## 2. P1 级缺陷（功能与设计不一致）

### P1-1 扇区计算重复，`side_angle_min_deg` 参数失效

**位置**：`a300_safety_controller.cpp:57-65`

```cpp
fl_ = sector(*s, rad(front_half_), rad(side_max_));    // [30°, 100°]
fr_ = sector(*s, -rad(side_max_), -rad(front_half_));  // [-100°, -30°]
left_  = sector(*s, rad(side_min_),  rad(side_max_));  // [30°, 100°]  ← 与 fl_ 完全相同
right_ = sector(*s, -rad(side_max_), -rad(side_min_)); // [-100°, -30°] ← 与 fr_ 完全相同
```

由于 `front_half_ == side_min_ == 30.0`（`obstacle.yaml` 中两者均为 30.0），`left_` 与 `fl_`、`right_` 与 `fr_` 是**完全相同的计算**，每帧浪费约一半的扇区遍历。

**两个后果**：

1. 参数 `side_angle_min_deg` 对结果**没有任何影响**，是失效配置项；
2. **纯侧向盲区无人监测**：100°–180° 区间（车体正侧方至斜后方）完全没有检测。§3.8 的"侧向防擦挂"逻辑使用的是 `[30°, 100°]` 前侧带，对车体正侧方的擦挂无保护能力。

**修复**：按 `DESIGN.md` §3.2 重定义五个独立扇区（前 30°、前侧 30–60°、侧向 60–120°、后向 150–180°），使 `side_angle_min_deg` 恢复语义。

---

### P1-2 状态字符串 `SLOW` 在消费端定义但生产端从不发布

**位置**：`status_light.py:22-27` **vs** `a300_safety_controller.cpp:124-136`

```python
BALLS = {
    "SAFE": "status_green",
    "WARNING": "status_yellow",
    "SLOW": "status_orange",       # ← 生产端从不发布此值
    "AVOIDING": "status_orange",
    "STOP": "status_red",
}
```

C++ 侧慢速区（`slow_distance < f ≤ stop_distance`）中会设置 `avoiding = true`，随后 `state = "AVOIDING"` 覆盖，因此 `"SLOW"` **永远不会出现在 `/a300/obstacle_state` 上**。

**修复**：从 `BALLS` 中删除 `SLOW`，并补齐 `CMD_TIMEOUT`、`ESTOP`（P0-3 引入后）的映射。更根本的措施是按 `DESIGN.md` §9.3 将状态取值集中定义。

---

### P1-3 `reverse_allowed = false` 与"超声波倒车停障"目标冲突

**位置**：`obstacle.yaml:14`、`a300_safety_controller.cpp:171`

```cpp
if(!reverse_&&out.linear.x<0)out.linear.x=0;
```

当前实现**无条件禁止倒车**，这是对"后向感知能力缺失"的规避手段，代价是丧失狭小空间脱困能力。而目标架构明确要求"超声波 → 倒车停障"，即**倒车应在感知保护下被允许**。

**修复**：实现 L0 超声波节点后，将 `reverse_allowed` 语义从静态开关改为**条件性允许**（`reverse_allowed ∧ ultrasonic_healthy`），并引入独立的 `rear_stop_distance` / `rear_slow_distance` 阈值集（`DESIGN.md` §6.4）。

---

### P1-4 SLAM 启动文件使用相对路径，安装后不可用

**位置**：`src/a300_slam/launch/slam.launch.py:4`

```python
parameters=["config/mapper_params_online_async.yaml"]
```

相对路径在 launch 执行时相对于**当前工作目录**解析。`CMakeLists.txt` 已将配置安装到 `share/a300_slam/config/`，因此从任意目录启动都会因找不到文件而失败。

**修复**：

```python
parameters=[PathJoinSubstitution([
    FindPackageShare("slam_toolbox"), ...   # 或
    FindPackageShare("a300_slam"), "config", "mapper_params_online_async.yaml"
])]
```

---

### P1-5 SLAM 配置 `use_sim_time: false` 与仿真时钟冲突

**位置**：`src/a300_slam/config/mapper_params_online_async.yaml:16`

仿真链路的 `sim.launch.py` 对全部节点设置 `use_sim_time: True`，而 SLAM 配置显式设为 `false`。同时启用会导致：

- SLAM 节点使用系统墙钟时间戳，与 `/scan`（`/clock` 驱动的仿真时间戳）混用，引发 TF 时间外插错误、`MessageFilter` 丢弃全部扫描；
- 地图构建失败或位姿跳变。

**修复**：改为 `true`，或在 launch 中通过参数覆盖；同时建议与 `sim.launch.py` 的时钟策略统一到同一份配置源。

---

### P1-6 LiDAR 安装高度在文档与模型间不一致

| 来源 | 取值 |
|---|---|
| `README.md:119` | `LiDAR position: x=0.20 m, z=0.70 m` |
| `src/a300_description/urdf/a300.urdf.xacro` | `<origin xyz="0.20 0 0.85"/>` |

相差 0.15 m。该参数直接决定 LiDAR 扫描平面高度，进而决定能探测到哪些障碍物（也决定"哪些障碍物会被漏掉"），**不可含糊**。

**修复**：以实测值为唯一事实来源，同步修正文档与 URDF。

---

### P1-7 `fl_` / `fr_` 初始化值与同组变量不一致

**位置**：`a300_safety_controller.cpp:179`

```cpp
double front_{std::numeric_limits<double>::infinity()}, fl_{}, fr_{},
       left_{std::numeric_limits<double>::infinity()},
       right_{std::numeric_limits<double>::infinity()};
```

`front_` / `left_` / `right_` 初始化为 `+∞`（表示"开阔"），而 `fl_` / `fr_` 被值初始化为 **`0.0`**（表示"紧贴障碍"）。

**当前行为**：首帧扫描到来前，`fr_ = fl_ = 0.0 ≤ stop_distance`，故 `openR`/`openL` 均为 `false`，进入 `STOP`。**结果恰好是保守而安全的**，属于"因巧合正确"。

**风险**：这是隐式依赖默认值与阈值大小关系的脆弱设计。若 `stop_distance` 被调小到 0 以下或误判逻辑变更，将立刻演变为不安全行为。

**修复**：全部初始化为 `+∞`（表示未知即开阔但由 `SCAN_STALE` 兜底），或显式引入"未知"状态位。

---

### P1-8 `a300_base_controller` 与 `a300_nav` 为空壳目录，`colcon` 不识别为包

**位置**：`src/a300_base_controller/`、`src/a300_nav/`

两目录内**仅有 `README.md`**，缺少 `package.xml` 与 `CMakeLists.txt`。

**后果**：

1. `colcon build` 不构建、不报错，静默跳过；
2. `rosdep install --from-paths src` 不处理其依赖；
3. `README.md` 中"Packages"章节列出这两个包，与实际不符，造成**文档与仓库状态不一致**。

**修复**：补齐包骨架（`package.xml` + `CMakeLists.txt`），或在 README 中明确标注为"规划中，未创建"。

---

## 3. P2 级问题（工程质量与一致性）

| 编号 | 位置 | 问题 | 建议 |
|---|---|---|---|
| P2-1 | `a300_safety_controller.cpp:173` | `RCLCPP_INFO` 以 50 Hz 打印完整调试信息，产生约 50 行/秒日志。长时间运行会淹没日志、增加 I/O 负担 | 降为 `DEBUG`，或用 `RCLCPP_INFO_THROTTLE` 限流至 1–2 Hz |
| P2-2 | `a300_safety_controller.cpp:169` | 无花括号的 `if(...) avoid_phase_=0; avoid_dir_=0.0;` — `avoid_dir_=0.0` 实际**无条件执行**。当前语义可接受，但属典型易错模式 | 补花括号并明确意图 |
| P2-3 | `a300_safety_controller.cpp:100` | 在未持 `cmd_m_` 锁的情况下直接读 `desired_.angular.z`（第 84 行则持锁）。当前因 `rclcpp::spin` 单线程执行器而**无实际数据竞争**，但锁使用不一致 | 统一在锁内读取，或明确注释说明单线程假设 |
| P2-4 | `a300_safety_controller.cpp:76` | `odomCb` 复用 `scan_m_` 保护 `yaw_`，将里程计状态与扫描状态混置于同一互斥量 | 独立锁或合并为单一状态结构体 |
| P2-5 | `a300_safety_controller.cpp:90` | 扫描超时阈值 `0.30` 硬编码 | 参数化为 `scan_timeout` |
| P2-6 | `a300_safety_controller.cpp:100` | 驾驶员转向判定阈值 `0.05` 硬编码 | 参数化为 `driver_steer_eps` |
| P2-7 | `a300_visualization/launch/status_light.launch.py` | launch 文件未设置 `use_sim_time`，而节点订阅 `/odom`（仿真时间戳）；虽仅用位姿不使用时间戳，但语义不完整 | 显式设置 `use_sim_time` |
| P2-8 | `status_light.py:34, 40-52` | 通过 `subprocess` 调用 `gz service`，单次超时 **8 s**，在单线程 executor 中会**阻塞订阅回调**（含 `/a300/obstacle_state` 与 `/odom`）。服务不可用时节点近乎失能 | 改用 `ros_gz` 服务桥接的 ROS 接口，或缩短超时并异步化 |
| P2-9 | `status_light.py:11` | 世界名 `"a300_test"` 硬编码；且 `BALLS` 未覆盖新增状态 | 参数化 `world_name`；状态表与 C++ 侧共享定义 |
| P2-10 | `docs/SIMULATION.md` | 内容已过时：文中称"下一步实现 Gazebo 差速驱动系统、LiDAR 传感器"，而这些**均已完成** | 按实际状态重写，或合并入 `ARCHITECTURE.md` |
| P2-11 | `start_demo.sh:26` | `WS="/mnt/c/Users/夏青/Desktop/a300/a300-r5"` 硬编码 WSL 路径与用户名，换机器即失效 | 改为脚本自身路径推导（`WS="$(cd "$(dirname "$0")" && pwd)"`，注意 WSL 路径转换） |
| P2-12 | `start_demo.sh:29-33` | 用 `ps aux \| grep -E` + `kill -9` 清理进程，匹配模式包含 `ros2 launch`、`parameter_bridge` 等**通用串**，可能误杀同机其他 ROS 工作负载 | 改用 PID 文件记录自身启动的进程 |
| P2-13 | 仓库根目录 | `package.xml` 声明 `Apache-2.0`，但仓库**缺少 `LICENSE` 文件** | 补充 LICENSE |
| P2-14 | 全仓库 | 无任何单元测试、集成测试或 CI 配置 | 优先为扇区计算、阈值判定、绕行方向决策补充纯函数单元测试 |
| P2-15 | `src/a300_lidar/` | 依赖外部包 `bluesea2`，但仓库未提供 `*.repos` 或 `.rosinstall` 声明其来源与版本 | 增加 `dependencies.repos` 固定依赖版本 |
| P2-16 | `src/a300_description/urdf/a300.urdf.xacro` | `laser_link` 无 `collision` 定义，仿真中雷达可穿墙 | 补充 collision（或在模型上加保护罩）；同时确认 `base_link` 车体（顶部 z=0.775）与雷达（z=0.85）的悬空安装是否符合实际结构 |

---

## 4. 配置一致性核验（通过项）

以下设计点经核验与设计意图一致，予以确认：

| 项 | 核验结果 |
|---|---|
| Gazebo ↔ ROS 桥接方向 | `/clock`、`/scan`、`/odom`、`/tf`、`/joint_states` 单向 GZ→ROS；`/cmd_vel` 单向 ROS→GZ。无双向桥接导致的指令回环 |
| 仿真 LiDAR 参数 | 360°、400 采样、约 0.9°/点、10 Hz、0.05–12 m，与 `REAL_LIDAR.md` 记载的 LDS-E110-R-5 手册值一致 |
| 轮距一致性 | URDF 关节 `y = ±0.275` → 0.55 m，与 `DiffDrive` 的 `wheel_separation = 0.55` 一致 |
| 轮径一致性 | URDF 轮半径 0.18 = `DiffDrive` 的 `wheel_radius = 0.18` |
| 仿真与真机参数一致性 | `DiffDrive` 的 `max_linear_velocity = 0.80` / `max_angular_velocity = 1.00` 与避障层 `max_linear_speed` / `max_angular_speed` 一致 |
| 传感器 QoS | 避障节点对 `/scan` 使用 `SensorDataQoS()`，与 LiDAR 驱动的 best-effort QoS 匹配 |
| 仿真时间 | 避障节点通过 launch 注入 `use_sim_time: True`，`now()` 使用仿真时钟，与 `/scan` 时间戳同源 |
| 控制频率 | 20 ms（50 Hz）定时器，高于 10 Hz 扫描率，无帧丢弃 |

---

## 5. 修复优先级建议

**第一优先（上车前的硬性阻断项）**

1. P0-1 期望速度看门狗
2. P0-3 急停通路
3. P0-4 雷达失效降级策略
4. P0-2 朝向一致性修正（同时阻断 Nav2 接入）

**第二优先（正确性）**

5. P1-1 扇区重定义
6. P1-6 LiDAR 高度标定与文档对齐
7. P1-7 初始化一致性
8. P1-2 状态集合对齐

**第三优先（能力补齐）**

9. P1-3 + 超声波倒车停障（L0 新增）
10. L4 底盘运动控制层与 L5 CAN 链路
11. P1-8 包骨架补齐
12. P1-4 / P1-5 SLAM 链路修复

**第四优先（工程质量）**

13. P2 全部条目，其中 P2-1（日志限流）、P2-8（阻塞式服务调用）、P2-11/P2-12（脚本健壮性）优先

---

## 6. 相关文档

- [`ARCHITECTURE.md`](./ARCHITECTURE.md) —— 分层架构、话题契约与现状映射
- [`DESIGN.md`](./DESIGN.md) —— 各层算法规格与待建模块接口设计
- [`ROADMAP.md`](./ROADMAP.md) —— 按里程碑组织的实施计划与验收标准
