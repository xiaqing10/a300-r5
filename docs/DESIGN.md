# A300 详细设计方案

> 版本：v1.0 ｜ 代码基线：`fa4f72b` **+ 当前工作区未提交改动**
> 配套文档：[`ARCHITECTURE.md`](./ARCHITECTURE.md)（分层与契约）、[`REVIEW.md`](./REVIEW.md)（缺陷清单）、[`ROADMAP.md`](./ROADMAP.md)（实施计划）
> 本文档给出**算法规格、参数取值、状态机定义与待建模块的接口设计**，可直接作为编码依据。

---

## 1. 设计目标与非目标

### 1.1 目标

| 编号 | 目标 | 度量 |
|---|---|---|
| G1 | 操控意图可解析 | 摇杆/键盘输入到 `/cmd_vel_desired` 的映射可复现、参数可配 |
| G2 | 局部自主绕障 | 在静态走廊场景下，机器人可自主绕开前方正对障碍并回到原航向 |
| G3 | 安全约束不可旁路 | 任何上游模块均无法绕过 L3 直接驱动底盘 |
| G4 | 故障可预测收敛 | 任一单点故障（雷达掉线、上游断连、急停触发）→ 速度收敛到 0 |
| G5 | 仿真到真机可迁移 | 避障/安全节点在仿真与真机间零修改复用 |

### 1.2 非目标（当前阶段不做）

- 全局路径规划与自主导航（属 Nav2 阶段）
- 动态障碍物预测与避让（仅做反应式避障）
- 非结构化地形通过性判断（LiDAR 为单平面扫描，无法感知低矮/悬空障碍）
- 载人产品的功能安全认证

---

## 2. L1 人机操控解析层

### 2.1 职责

将物理操控设备（摇杆、键盘、未来的 App/语音）的输入，归一化为车体期望速度 `geometry_msgs/Twist`，发布到 `/cmd_vel_desired`。**该层不做任何避障或安全判断**。

### 2.2 摇杆映射（`a300_joystick_mapper`，C++）

**死区归一化函数**（消除摇杆回中漂移，同时保证零位连续）：

```
dz(x) = 0                                     , |x| < d
      = sign(x) · (|x| − d) / (1 − d)         , |x| ≥ d
```

其中 `d = deadzone`。该式将 `[d, 1]` 线性映射到 `[0, 1]`，在 `x = d` 处输出 0，保证单调且无跳变。

**输出**：

```
linear.x  = dz(axes[axis_linear])  × max_linear_speed
angular.z = dz(axes[axis_angular]) × max_angular_speed
```

**参数表**

| 参数 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `axis_linear` | 1 | — | `/joy` 中映射为线速度的轴索引 |
| `axis_angular` | 0 | — | `/joy` 中映射为角速度的轴索引 |
| `max_linear_speed` | 0.80 | m/s | 线速度上限（与 L3 一致，L3 仍会复检） |
| `max_angular_speed` | 1.00 | rad/s | 角速度上限 |
| `deadzone` | 0.08 | — | 死区半宽 |

**待补设计**

1. **轴索引必须实测确认**。不同摇杆设备（Xbox / 罗技 / 工业手柄）轴顺序不同，`README` 与 `SETUP.md` 已明确要求上车前验证。
2. **死区仅作用于线性轴，未作用于角速度轴**——当前实现统一应用，符合预期。
3. 建议新增：
   - `axis_linear_sign` / `axis_angular_sign`：支持反向安装的摇杆；
   - `deadman_button`：使能按钮，未按下时强制输出 0（防误触）；
   - `estop_button`：急停按钮 → 触发 L3 的 E-STOP 锁存（见 §4.4）。

### 2.3 键盘映射（`teleop_keyboard.py`）

沿用 `teleop_twist_keyboard` 键位，用于仿真演示。**与摇杆的差异是硬限速**：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `MAX_LINEAR` | 0.60 m/s | 硬上限，`q` 键增量也无法突破 |
| `MAX_ANGULAR` | 1.50 rad/s | 硬上限 |
| `speed` | 0.30 m/s | 初始线速度 |
| `turn` | 1.00 rad/s | 初始角速度 |

> **参数不一致提示**：键盘上限 0.6 m/s 与摇杆上限 0.8 m/s 不一致。L3 的 `max_linear_speed = 0.80` 是最终收敛值，因此键盘路径实际被限制在 0.6，而摇杆路径可达 0.8。建议统一为同一份配置源（如 `a300_bringup/config/speed_limits.yaml`）。

### 2.4 接口契约

| 项 | 值 |
|---|---|
| 输入 | `/joy`（`sensor_msgs/Joy`，仅摇杆路径） |
| 输出 | `/cmd_vel_desired`（`geometry_msgs/Twist`） |
| 频率 | 事件驱动（收到消息即发） |
| 失效行为 | **无输出**（不发零速）——超时归零由 L3 看门狗承担 |

---

## 3. L2 局部自主避障层

### 3.1 职责

基于单帧 LiDAR 扫描，判断车体周边可通行空间，在期望速度上叠加**减速 + 绕行转向**修正，输出 `Twist`。该层是**反应式（reactive）** 避障，不维护地图，不做路径规划。

### 3.2 扇区模型

以 `laser_link` 为原点，`base_link` +X 为 0°，逆时针为正。**设计定义的五个扇区**如下：

```
                    前方 f  [−30°, +30°]
                        ▲
        ┌───────────────┼───────────────┐
        │               │               │
   左前 fl             │            右前 fr
 [30°, 100°]           │          [−100°, −30°]
        │               │               │
   左侧 l              │             右侧 r
 [100°, 180°]          │          [−180°, −100°]
```

**扇区最小距离**：对每个扇区，取落在区间内所有**有限且有效**（`range_min ≤ r ≤ range_max`，`isfinite`）测距值的最小值；无有效点则记为 `+∞`。

```
sector(a, b) = min{ r_i | a ≤ angle_i ≤ b, isfinite(r_i), range_min ≤ r_i ≤ range_max }
```

**⚠️ 当前实现的偏差（REVIEW P1-1）**：代码中

```cpp
front_ = sector(-30°,  30°);   // 前方
fl_    = sector( 30°, 100°);   // 左前
fr_    = sector(-100°, -30°);  // 右前
left_  = sector( 30°, 100°);   // 左  ← 与 fl_ 完全相同
right_ = sector(-100°, -30°);  // 右  ← 与 fr_ 完全相同
```

`left_` 与 `fl_`、`right_` 与 `fr_` 互为**重复计算**，参数 `side_angle_min_deg` 未产生任何独立作用，纯侧向盲区（100°–180°）实际无人监测。

**修正设计**：将左右侧带收窄并真正独立：

```
fl_ = sector( 30°,  60°);   // 左前窄带：影响绕行方向决策
fr_ = sector(-60°, -30°);   // 右前窄带
l_  = sector( 60°, 120°);   // 左侧带：约束左转（防擦挂）
r_  = sector(-120°, -60°);  // 右侧带：约束右转
```

并新增可选的后向扇区 `rear_ = sector(150°, 180°) ∪ sector(-180°, -150°)`，用于 L0 超声波融合前的占位（见 §6）。

### 3.3 距离阈值

| 参数 | 默认值 | 语义 | 对应动作 |
|---|---|---|---|
| `warning_distance` | 1.20 m | 预警距离 | 线速度线性衰减至 50%~100% |
| `slow_distance` | 0.80 m | 减速距离 | 线速度按比例衰减，启动绕行转向 |
| `stop_distance` | 0.35 m | 停止距离 | 若两侧均无余量则停车 |

**阈值标定依据**：`stop_distance` 必须大于「车体前悬 + 制动距离 + 通信与执行延迟期间位移」三者之和。当前 0.35 m 为**开发默认值**，未做实测标定。

```
stop_distance > L_front + (v_max · t_latency) + v_max² / (2 · a_brake)
```

以 `v_max = 0.8 m/s`、`t_latency = 0.2 s`、`a_brake = 1.0 m/s²` 估算，仅制动与延迟两项即需 0.48 m。**因此 0.35 m 在 0.8 m/s 下不安全**，需按实测延迟重新标定，或引入速度相关的动态停止距离：

```
stop_distance(v) = d_0 + v · t_latency + v² / (2 · a_brake)
```

建议将此作为下一阶段的核心标定任务（ROADMAP M3）。

### 3.4 避障状态机

**外部状态**（发布于 `/a300/obstacle_state`）：

| 状态 | 触发条件 | 语义 |
|---|---|---|
| `SAFE` | 前方 `f > warning_distance`，且无避障进行中 | 正常通行 |
| `WARNING` | `slow_distance < f ≤ warning_distance` | 预警减速 |
| `AVOIDING` | `f ≤ slow_distance` 且存在可通行侧向空间 | 减速 + 自动绕行 |
| `STOP` | `f ≤ stop_distance` 且左右前向扇区均 `≤ stop_distance` | 无路可走，停车 |
| `SCAN_STALE` | 距上次有效扫描 > 0.30 s | 雷达数据失效 |

**内部相位** `avoid_phase`：

| 相位 | 含义 |
|---|---|
| 0 | 空闲，无避障行为 |
| 1 | 绕行中（正在施加 `avoid_dir_` 方向转向） |
| 2 | 恢复中（正在转回进入避障时的航向） |

**状态转移图**

```
                    f > warning                       f ≤ warning
        ┌───────────────────────────┐   ┌───────────────────────────────────┐
        │                           ▼   │                                   │
     ┌──────┐                    ┌─────────┐    f ≤ slow      ┌────────────┐
     │ SAFE │───────────────────►│ WARNING │─────────────────►│  AVOIDING  │
     └──────┘                    └─────────┘                  └─────┬──────┘
        ▲                             ▲                             │ f > slow
        │                             │ f > slow                    │ 且 phase==1
        │                             │                             ▼
        │                        ┌────┴─────┐                 ┌────────────┐
        └────────────────────────│  (恢复)   │◄────────────────│  phase=2   │
             |Δyaw| < 2°         └──────────┘                 │  航向恢复   │
                                  phase=2                    └────────────┘

     AVOIDING ──[两侧均 ≤ stop]──► STOP ──[一侧打开]──► AVOIDING
     ANY      ──[扫描超时 0.3s]──► SCAN_STALE ──[扫描恢复]──► SAFE
```

### 3.5 绕行方向决策（步骤 ④）

```
diff = fr − fl
if |diff| < avoid_sym_eps:   direction = avoid_default_dir   （默认 +1，左转）
elif diff > 0:               direction = −1                  （右前更开阔 → 右转）
else:                        direction = +1                  （左前更开阔 → 左转）
```

转向叠加（不做覆盖，保留驾驶员的角速度分量）：

```
ω_out = ω_desired + direction × avoid_gain
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `avoid_gain` | 0.80 rad/s | 绕行转向增益 |
| `avoid_default_dir` | +1.0 | 左右对称时的默认绕行方向（左转） |
| `avoid_sym_eps` | 0.03 m | 判定左右对称的距离差阈值 |
| `avoid_min_linear` | 0.10 m/s | 绕行期间的最低线速度（避免完全停死） |

**设计取舍**：`avoid_min_linear` 强制保留 0.10 m/s 前进速度，目的是避免"停车 → 无法转向 → 永久卡死"。代价是在极近距（`f` 略大于 `stop_distance`）时仍以 0.10 m/s 前进。该值必须与 `stop_distance` 联合标定，并保证 0.10 m/s 下的制动距离远小于剩余裕量。

### 3.6 驾驶员优先原则

```
if avoid_phase ≠ 0 and |ω_desired| > 0.05:
    avoid_phase = 0; avoid_dir = 0     # 放弃自动绕行，交还控制权
```

**设计意图**：自动绕行必须是**可被驾驶员随时打断**的辅助行为，而非夺权。0.05 rad/s 的阈值用于过滤摇杆抖动。

### 3.7 航向恢复（步骤 ⑤ 的后半段）

绕障后若不修正航向，机器人会持续朝侧向漂移。设计采用**航向记忆**策略：

```
进入 phase=1 时： start_yaw = current_yaw      （记录进入时的航向）
障碍清除后（f > slow，phase==1 → phase=2）：
    d = normPi(current_yaw − start_yaw)
    if |d| < recover_eps_deg:  phase = 0       （恢复完成）
    else:                      ω_out += clamp(d × recover_gain, ±w_max)
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `recover_gain` | 1.0 | 航向恢复比例增益 |
| `recover_eps_deg` | 2.0° | 恢复完成判定容差 |

**安全性论证**：恢复逻辑仅在 `f > slow_distance` 时执行（转移条件排在 `f ≤ slow` 分支之后），此时前方有足够余量。转向回目标航向的过程是"离开绕行侧障碍物"的方向，因此在障碍物静止的前提下不会引入新碰撞。

**已知限制**：
1. 依赖 `/odom` 提供航向。若里程计失效（`yaw = NaN`），恢复分支被跳过，机器人保持偏航行进。
2. 障碍物移动或场景为动态时，该安全性论证不成立。
3. 若驾驶员在绕行期间调整了意图航向，恢复目标仍是**进入避障时**的航向，可能与当前意图不符。建议改为：`avoid_target` 在驾驶员主动转向时同步更新为当前航向 + 驾驶员转向量的积分。

### 3.8 侧向防擦挂（步骤 ③ 的补充）

```
if ω_out > 0 and l < stop_distance:  ω_out = 0     # 左转被左侧障碍阻断
if ω_out < 0 and r < stop_distance:  ω_out = 0     # 右转被右侧障碍阻断
```

该约束直接作用于最终角速度，**优先级最高**，可覆盖驾驶员与自动绕行两路转向分量。

### 3.9 该层的输入输出

| 项 | 值 |
|---|---|
| 输入 | `/scan`（`SensorDataQoS`）、`/cmd_vel_desired`、`/odom` |
| 输出 | `/cmd_vel`（当前，见 REVIEW P0）、`/a300/obstacle_state` |
| 执行周期 | 20 ms（50 Hz） |
| 线程安全 | `scan_m_` 保护扫描与航向，`cmd_m_` 保护期望速度，两把独立互斥锁 |

> **控制频率说明**：50 Hz 的处理周期远高于 10 Hz 的 LiDAR 更新率，因此同一帧扫描会被处理约 5 次，等价于"扫描保持（zero-order hold）"。这在低速场景下可接受，但意味着**障碍物出现到被响应之间最坏有 100 ms 的采样延迟**，该延迟必须计入 §3.3 的动态停止距离。

---

## 4. L3 安全控制层

### 4.1 职责边界

| 归属 L3 的职责 | 归属 L2 的职责 |
|---|---|
| 速度硬限幅（最终、不可突破） | 基于障碍的距离减速 |
| 上游心跳看门狗 | 绕行方向决策 |
| 传感器失效检测与降级 | 航向恢复 |
| 急停锁存与释放 | 侧向擦挂抑制 |
| 故障分级与安全状态发布 | — |

**当前实现的状态**：L2 与 L3 的职责**全部耦合在 `a300_safety_controller.cpp` 单节点内**。这带来两个问题：

1. 无法独立测试避障逻辑（必须连带安全约束一起验证）；
2. 安全约束的优先级实现（限幅在循环开头、在避障逻辑之后又 clamp 一次、最后发布前沿反）分散在代码多处，难以审查。

### 4.2 拆分方案对比

| 方案 | 结构 | 优点 | 缺点 |
|---|---|---|---|
| **A. 单节点内部分层** | 一个节点内划分 `applySafety(avoid(cmd))` 两个纯函数 | 零额外延迟；改动最小 | 仍无法独立部署/测试；职责仍在同一进程 |
| **B. 拆为两个节点** | `a300_safety_controller`（避障）→ `/cmd_vel_avoided` → `a300_safety_guard`（安全）→ `/cmd_vel` | 职责隔离、可独立测试、安全层可复用于其他上游 | 增加一跳通信（本地约 0.2–1 ms，可接受） |
| **C. 单节点 + 插件化** | 一个节点，安全策略编译期/运行期可选 | 兼顾两者 | 实现复杂度最高 |

**推荐方案 B**。额外一跳延迟在本地 DDS（共享内存）下可忽略，而"安全层可独立于避障算法演进"带来的收益显著。安全层作为**唯一的 `/cmd_vel` 发布者**，也天然满足 §1.1 的 G3（不可旁路）。

> 若采用方案 B，则 `/cmd_vel_desired` → `/cmd_vel_avoided` 为内部话题，L3 的输入为 `/cmd_vel_avoided`（无避障时可直接透传 `/cmd_vel_desired`）。

### 4.3 看门狗设计（**当前缺失，P0**）

**风险**：`a300_safety_controller` 订阅 `/cmd_vel_desired` 后将其缓存于 `desired_`，并在 50 Hz 循环中**持续重发**。上游（摇杆节点、键盘、未来的 Nav2）一旦断开，`desired_` 保持最后值，机器人将**持续前进**。

**设计**：

```
若 now() − last_desired_rx > desired_timeout:
    out = Twist()          # 全零
    state = "CMD_TIMEOUT"
```

| 参数 | 建议值 | 说明 |
|---|---|---|
| `desired_timeout` | 0.30 s | 期望速度心跳超时 |

**对上游的约束**：所有 L1 模块必须**周期发布**（建议 ≥ 10 Hz），即使速度未变化。这要求修改现有实现：
- `a300_joystick_mapper`：改为定时器驱动，以 `publish_rate`（默认 20 Hz）周期发布缓存的摇杆指令；
- `teleop_keyboard`：改为定时器驱动 + 按键更新目标值；
- Nav2：`controller_server` 本身以控制周期发布，满足要求。

> 这是**架构性约束**：一旦引入看门狗，所有 L1 模块的发布模型都必须从"事件驱动"改为"周期驱动"。

### 4.4 急停设计（**当前缺失，P0**）

**接口**：`std_srvs/SetBool` 服务 `/a300/estop`，或锁存型 `std_msgs/Bool` 话题 `/a300/estop`。

**语义**：

| 行为 | 定义 |
|---|---|
| 触发 | 立即输出零速；**锁存**，不因障碍消失而自动释放 |
| 释放 | 必须显式调用（`data=false`）或按键组合，禁止自动恢复 |
| 覆盖范围 | 优先于所有其他逻辑，包括驾驶员指令 |
| 状态广播 | `/a300/obstacle_state` 输出 `ESTOP` |

**分级设计**（与 §4.5 故障分级配合）：

| 等级 | 触发源 | 动作 | 可自动恢复 |
|---|---|---|---|
| E0 | 正常 | 不干预 | — |
| E1 | 前方障碍 | 减速/绕行/停车 | 是 |
| E2 | 传感器失效 / 上游超时 | 禁止前进，允许转向与制动 | 是（故障消除后） |
| E3 | 急停按钮 / 电机故障 / 越界 | **全部速度归零并锁存** | 否，需显式释放 |

### 4.5 传感器失效检测（现状改进）

**当前实现**：

```cpp
if (now() - last_scan > 0.30s):
    if (out.linear.x > 0) out.linear.x = 0;    // 仅禁止前进
    state = "SCAN_STALE";
```

**问题**：雷达失效时**仍允许转向**，机器人可能在无感知状态下原地/低速旋转。属于 fail-degraded 而非 fail-safe。

**设计**：将雷达失效归入 **E2** 级，动作定义为：

```
若 雷达失效:
    linear.x  = 0                  # 禁止任何纵向运动
    angular.z = clamp(ω, ±w_safe)  # 允许受限转向以脱离，w_safe = 0.3 rad/s
```

并提供参数 `scan_stale_policy ∈ {stop_all, no_forward, allow_turn}` 供现场选择，默认 `no_forward`（当前行为），真机建议 `stop_all`。

| 参数 | 建议值 | 说明 |
|---|---|---|
| `scan_timeout` | 0.30 s | 扫描心跳超时（= 3 个扫描周期） |
| `scan_stale_policy` | `no_forward` | 失效策略 |

### 4.6 速度硬限幅

**最终限幅**必须在**所有修正之后、发布之前**执行，且限幅顺序固定为：

```
1. 解析层限幅        由 L1 完成（软限幅，用户体验层）
2. 避障修正          由 L2 完成（可增大 |ω|）
3. 安全层限幅        由 L3 完成（硬限幅，不可突破）
4. 故障降级          由 L3 完成（可强制归零）
5. 发布 /cmd_vel
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_linear_speed` | 0.80 m/s | 硬上限 |
| `max_angular_speed` | 1.00 rad/s | 硬上限 |

> **实现注意**：当前代码在避障分支中通过 `ω_out += direction × avoid_gain` 叠加转向，最大值可达 `w_max + avoid_gain = 1.80 rad/s`，依赖循环末尾的 `clamp` 收敛到 1.00。这在数学上正确，但**叠加与限幅分离**降低了可读性。建议改为：叠加后立即限幅，或在叠加上限中预留 `avoid_gain` 预算。

### 4.7 状态发布契约

`/a300/obstacle_state` 为 `std_msgs/String`，取值集合定义为：

```
SAFE | WARNING | AVOIDING | STOP | SCAN_STALE | CMD_TIMEOUT | ESTOP
```

| 消费方 | 用途 |
|---|---|
| `status_light` | 映射到颜色球（绿/黄/橙/红） |
| 未来 HMI | 状态显示与告警 |

> **当前不一致（REVIEW P1-2）**：`status_light.py` 的 `BALLS` 字典包含 `"SLOW": "status_orange"`，但 C++ 节点**从不发布 `SLOW`**（慢速区被 `AVOIDING` 覆盖）。该条目为死代码，且状态集合缺少 `CMD_TIMEOUT` / `ESTOP` 的映射。

---

## 5. L4 底盘运动控制层（**当前未实现**）

### 5.1 职责

将 `(v, ω)` 解算为左右轮角速度，写入 CAN 帧；同时由编码器反馈积分出里程计发布到 `/odom`。

### 5.2 差速运动学

**正解**（机器人速度 → 轮速）：

```
v_L = v − ω · L / 2            [m/s]     左轮线速度
v_R = v + ω · L / 2            [m/s]     右轮线速度

ω_L = v_L / r                  [rad/s]   左轮角速度
ω_R = v_R / r                  [rad/s]   右轮角速度
```

**反解**（轮速 → 机器人速度，用于里程计）：

```
v = (ω_L + ω_R) · r / 2
ω = (ω_R − ω_L) · r / L
```

**参数**

| 参数 | 当前值 | 单位 | 来源 |
|---|---|---|---|
| `wheel_radius` (r) | 0.18 | m | URDF 占位值 |
| `wheel_separation` (L) | 0.55 | m | URDF 占位值 |
| `ticks_per_rev` | 待定 | — | 编码器规格 |
| `gear_ratio` | 待定 | — | 减速比 |

> 轮径与轮距的**误差会直接放大为里程计方向漂移**。0.18 m 轮径若实际为 0.17 m，5.6% 的标定误差在 10 m 直行后表现为约 0.56 m 的位置偏差。**必须实测标定**（ROADMAP M5）。

### 5.3 约束与限幅设计

轮速解算后，需处理三类约束。**优先级顺序**如下（高优先级先满足）：

| 优先级 | 约束 | 处理方式 |
|---|---|---|
| 1 | 轮速物理上限 `ω_max` | 若任一 `\|ω_i\| > ω_max`，按比例缩减 `(v, ω)` 整体，保持转向半径不变 |
| 2 | 加速度上限 | 对 `v`、`ω` 分别做斜率限幅 |
| 3 | 电机死区 | 低于死区阈值的输出置 0，避免电机嗡鸣与过热 |

**统一比例缩减**（保持几何一致性）：

```
k = ω_max / max(|ω_L|, |ω_R|)
if k < 1:
    v ← v · k ;  ω ← ω · k
```

这样处理的原因：若分别钳位左右轮速，会**改变转向半径**（例如原意为左转，钳位后变成直行右偏），产生不可预测的运动。整体缩放只降低速度，保持轨迹形状。

### 5.4 加速度限幅

| 参数 | 建议值 | 说明 |
|---|---|---|
| `max_linear_accel` | 1.0 m/s² | 与仿真 `DiffDrive` 配置一致 |
| `max_angular_accel` | 2.0 rad/s² | 与仿真 `DiffDrive` 配置一致 |

**与仿真的差异**：仿真中该限幅由 `gz-sim-diff-drive-system` 插件内部完成。真机上必须由 L4 节点实现，否则速度阶跃会造成轮椅**冲击性加减速**，对乘员是明确的安全风险与体验缺陷。

### 5.5 接口设计

| 项 | 值 |
|---|---|
| 输入 | `/cmd_vel`（`geometry_msgs/Twist`）、CAN 反馈帧 |
| 输出 | `/odom`（`nav_msgs/Odometry`）、`/joint_states`、CAN 指令帧 |
| 控制周期 | 建议 50–100 Hz（与 CAN 总线周期对齐） |
| 输入超时 | 独立看门狗，超时 → 轮速归零（与 L3 看门狗形成纵深防御） |

---

## 6. L0 补充：超声波倒车停障（**当前缺失**）

### 6.1 需求分析

**为什么必须补**：

1. LiDAR 为**单平面**扫描（`laser_link` 高度 z ≈ 0.85 m），无法探测：
   - 低于扫描平面的障碍（台阶、路缘石、地面杂物）；
   - 高于扫描平面的悬空障碍（桌沿、扶手）。
2. 当前 `reverse_allowed = false` **硬禁止后退**——这是对上述盲区的"因噎废食"式规避，代价是失去了狭小空间脱困能力。
3. 载人轮椅场景中，倒车是高频且必要的操作。

**结论**：开放倒车的前提是**先建立后向感知**。超声波是成本与实时性最匹配的方案。

### 6.2 传感器布置

| 位置 | 数量 | 覆盖范围 | 主要用途 |
|---|---|---|---|
| 车体后向（左/右） | 2 | 各约 60° 锥角，覆盖后方 ≥ 2 m | 倒车停障、后方盲区 |
| 车体侧后向（可选） | 2 | 各约 60° 锥角 | 斜向倒车、转出车位 |

### 6.3 接口设计

**推荐**：统一封装为 `sensor_msgs/RangeArray` 或独立 `sensor_msgs/Range` 话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/range/rear_left` | `sensor_msgs/Range` | `radiation_type = ULTRASOUND`，`field_of_view` 按实际锥角 |
| `/range/rear_right` | `sensor_msgs/Range` | 同上 |
| `/ultrasonic/state` | `std_msgs/String` | 传感器健康状态 |

**融合规则**（在 L2 中执行）：

```
rear = min_valid(rear_left, rear_right)      # 最小值融合，取最保守
有效条件：消息时间戳在 0.5 s 内 且 测距值在 [range_min, range_max] 内
```

### 6.4 倒车安全逻辑设计

**当前**：

```cpp
if (!reverse_allowed && out.linear.x < 0) out.linear.x = 0;
```

**新设计**：将倒车纳入统一的三级阈值模型，但在**独立的阈值参数集**下运行：

```
若 linear.x < 0:                                  # 倒车分支
    if rear ≤ rear_stop:      linear.x = 0;  state = "REAR_STOP"
    elif rear ≤ rear_slow:    linear.x ×= (rear − rear_stop) / (rear_slow − rear_stop)
    if ultrasonic 无效:       linear.x = 0;  state = "REAR_SENSOR_INVALID"
```

| 参数 | 建议值 | 说明 |
|---|---|---|
| `rear_stop_distance` | 0.30 m | 后向停止距离 |
| `rear_slow_distance` | 0.60 m | 后向减速距离 |
| `rear_max_speed` | 0.30 m/s | 倒车速度上限（显著低于前进上限） |
| `reverse_allowed` | 由 `false` 改为 `true`，但**仅在超声波健康时生效** | 见下 |

**关键设计**：`reverse_allowed` 的语义应从"静态开关"改为"**条件性允许**"：

```
reverse_permitted = reverse_allowed ∧ ultrasonic_healthy
```

即：超声波故障时自动回退到禁止倒车（fail-safe），超声波正常时允许倒车受控倒退（fail-operational）。这样既恢复了脱困能力，又不降低安全等级。

### 6.5 与 LiDAR 的融合策略

| 场景 | 检测手段 | 决策 |
|---|---|---|
| 前进 | LiDAR 前方扇区 | §3.4 状态机 |
| 前进（低矮障碍） | 超声波**前向**（若安装） | 取 `min(LiDAR, 超声波)` |
| 后退 | 超声波后向 | §6.4 逻辑 |
| 转向 | LiDAR 侧向扇区 | §3.8 侧向防擦挂 |

**多传感器融合原则**：同类物理量取**最小值**（最保守），异类物理量按运动方向选用对应传感器。禁止"投票制"或"取平均"——在安全链路上，保守优先。

---

## 7. 贯穿性设计问题：朝向一致性（P0）

### 7.1 问题描述

`a300_safety_controller.cpp:174` 在发布前对线速度取反：

```cpp
out.linear.x = -out.linear.x;
safe_pub_->publish(out);
```

后果：

1. `/cmd_vel` 的符号与 `/cmd_vel_desired` **相反**，即"前进指令"在话题上表现为负值；
2. Gazebo 世界系与 `odom` 系**镜像 180°**（`odom.x = −world.x`, `odom.y = −world.y`）；
3. `status_light.py` 必须反向补偿才能把状态球放到正确位置（其注释已记录此现象）。

### 7.2 根因分析

迹象表明真实根因位于 **URDF 车轮关节的旋转轴定义**：

```xml
<joint name="left_wheel_joint" type="continuous">
  <origin xyz="0 0.275 0.18" rpy="1.5708 0 0"/>
  <axis xyz="0 0 1"/>            <!-- 疑点 -->
</joint>
```

`rpy="1.5708 0 0"` 将子链绕 X 轴旋转 90°，使圆柱体（默认轴为局部 Z）的几何轴指向 −Y（正确，轮轴应沿 Y）。但关节旋转轴 `<axis xyz="0 0 1"/>` 在旋转后的子链坐标系中解释，导致**关节正方向旋转产生负向行驶**。Gazebo 的 `DiffDrive` 插件按关节正方向定义"前进"，因此正向 `cmd_vel` 使机器人后退。

### 7.3 修正方案对比

| 方案 | 做法 | 优点 | 缺点 | 评价 |
|---|---|---|---|---|
| **A. 修正 URDF 关节轴** | `<axis>` 改为 `0 1 0`，或在 `DiffDrive` 中交换左右轮关节 | 根因修复，语义正确 | 需在仿真中回归验证 | **推荐** |
| B. 在桥接层加符号修正 | 桥接配置中加 `-` 前缀 | 改动最小 | 把缺陷推到桥接层，真机不适用 | 仅作临时手段 |
| C. 在避障节点内取反（现状） | 保持 `out.linear.x = -out.linear.x` | 无 | 污染安全节点语义；与 Nav2 语义冲突；真机致命 | **必须移除** |

### 7.4 验证方法

```
1. ros2 launch a300_gazebo sim.launch.py
2. ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
3. 观察：rviz2 中 odom 箭头方向；gz 中机器人实际移动方向
4. 判定：机器人应向 base_link +X 方向移动（与 rviz 前向箭头一致）
```

修正后，`a300_safety_controller.cpp:174` 的取反语句、`status_light.py` 的坐标取负补偿**均应删除**。

---

## 8. 参数汇总表

### 8.1 避障层（`obstacle.yaml`）

| 参数 | 当前值 | 单位 | 建议值 | 备注 |
|---|---|---|---|---|
| `scan_topic` | `/scan` | — | — | |
| `desired_cmd_topic` | `/cmd_vel_desired` | — | — | 拆分方案下改为 `/cmd_vel_avoided` |
| `safe_cmd_topic` | `/cmd_vel` | — | — | 拆分方案下改为发布 `/cmd_vel_avoided` |
| `odom_topic` | `/odom` | — | — | |
| `warning_distance` | 1.20 | m | 待标定 | |
| `slow_distance` | 0.80 | m | 待标定 | |
| `stop_distance` | 0.35 | m | **需增大**，见 §3.3 | |
| `front_half_angle_deg` | 30.0 | ° | 待验证 | |
| `side_angle_min_deg` | 30.0 | ° | **修正扇区后重定义**，见 §3.2 | 当前无实际作用 |
| `side_angle_max_deg` | 100.0 | ° | 待验证 | |
| `max_linear_speed` | 0.80 | m/s | 待标定 | |
| `max_angular_speed` | 1.00 | rad/s | 待标定 | |
| `reverse_allowed` | `false` | — | 改为条件性允许，见 §6.4 | |
| `avoid_gain` | 0.80 | rad/s | 待调参 | |
| `avoid_min_linear` | 0.10 | m/s | 待标定 | |
| `avoid_default_dir` | +1.0 | — | — | |
| `avoid_sym_eps` | 0.03 | m | — | |
| `recover_gain` | 1.0 | — | — | |
| `recover_eps_deg` | 2.0 | ° | — | |
| `desired_timeout` | **未实现** | s | 0.30 | 见 §4.3 |
| `scan_timeout` | 0.30（硬编码） | s | 0.30，改为参数 | 见 §4.5 |
| `scan_stale_policy` | **未实现** | — | `no_forward` | 见 §4.5 |

### 8.2 运动学层（待新建）

| 参数 | 值 | 单位 |
|---|---|---|
| `wheel_radius` | 0.18（占位） | m |
| `wheel_separation` | 0.55（占位） | m |
| `max_wheel_speed` | 待定 | rad/s |
| `max_linear_accel` | 1.0 | m/s² |
| `max_angular_accel` | 2.0 | rad/s² |

### 8.3 超声波层（待新建）

| 参数 | 值 | 单位 |
|---|---|---|
| `rear_stop_distance` | 0.30 | m |
| `rear_slow_distance` | 0.60 | m |
| `rear_max_speed` | 0.30 | m/s |
| `range_timeout` | 0.50 | s |

---

## 9. 编码规范约定

1. **符号约定**：所有节点内部使用 ROS 标准约定（前 +X、左转 +ω），**禁止在任何中间环节做符号翻转**。若需适配硬件，适配层须显式命名（如 `hardware_sign_adapter`）并单独成节点。
2. **参数不外置硬编码**：所有距离、速度、角度阈值必须来自 YAML，不在 C++ 内使用魔数。当前 `0.30 s` 扫描超时、`0.05 rad/s` 驾驶员转向阈值均为硬编码，应参数化。
3. **状态字符串集中定义**：`/a300/obstacle_state` 的取值应在头文件中以枚举/常量定义，避免生产端与消费端不一致（当前已出现 `SLOW` 分歧）。
4. **安全逻辑先写测试**：扇区计算、阈值判定、方向决策应可在无 ROS 环境下单元测试（逻辑与 rclcpp 解耦）。
5. **仿真与真机同一份算法**：任何"仅仿真"分支必须通过参数开关显式声明，不得依赖 `use_sim_time` 隐式分叉。

---

## 10. 相关文档

- [`ARCHITECTURE.md`](./ARCHITECTURE.md) —— 分层架构与话题契约
- [`REVIEW.md`](./REVIEW.md) —— 缺陷清单与修复优先级
- [`ROADMAP.md`](./ROADMAP.md) —— 里程碑与验收标准
