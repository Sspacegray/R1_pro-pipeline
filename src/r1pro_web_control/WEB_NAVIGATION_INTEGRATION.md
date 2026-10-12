# R1 Pro 网页端导航适配说明

本文面向网页 UI 和 `r1pro_web_server` 后端开发，说明如何让网页底盘手动控制与
Nav2 导航安全共存。

适配目标：

- 网页手动控制和导航不能同时向底盘发送速度；
- 切换控制权时必须等待 ROS service 确认；
- 导航空闲时不持续发送零速度，网页可以正常接管；
- 网页进入手动模式时，如果导航正在运动，先停车再释放控制权；
- 网页刷新、网络抖动或后端异常时不能意外启用导航。

## 1. 控制链路

```text
网页 UI
  │ REST / WebSocket
  ▼
r1pro_web_server
  ├── 手动速度 ───────────────► /motion_target/target_speed_chassis
  ├── 导航目标 ───────────────► /goal_pose
  ├── 导航控制权 service ─────► /cmd_vel_to_twist_stamped/set_nav_output_enabled
  └── 导航控制权状态 ◄──────── /cmd_vel_to_twist_stamped/nav_output_state

Nav2 /cmd_vel
  ▼
cmd_vel_to_twist_stamped
  ▼（仅获得控制权且正在运动时）
/motion_target/target_speed_chassis
  ▼
chassis_control_node
```

浏览器不应直接连接 ROS。模式切换、ROS service 调用和速度发布应集中在
`r1pro_web_server`，网页只调用后端 API。

## 2. ROS 接口

| 用途 | 名称 | 类型 | 方向 |
|---|---|---|---|
| 导航综合状态 | `/navigation/status` | `std_msgs/msg/String`（JSON） | 网页后端订阅 |
| 开始原地旋转定位 | `/navigation/start_localization_spin` | `std_srvs/srv/Trigger` | 网页后端调用 |
| 取消原地旋转定位 | `/navigation/cancel_localization_spin` | `std_srvs/srv/Trigger` | 网页后端调用 |
| 清除一次全局代价地图 | `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 网页后端调用 |
| 导航控制权开关 | `/cmd_vel_to_twist_stamped/set_nav_output_enabled` | `std_srvs/srv/SetBool` | 网页后端调用 |
| 导航控制权状态 | `/cmd_vel_to_twist_stamped/nav_output_state` | `std_msgs/msg/String` | 网页后端订阅 |
| 手动底盘速度 | `/motion_target/target_speed_chassis` | `geometry_msgs/msg/TwistStamped` | 网页后端发布 |
| 导航目标 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 网页后端发布 |
| 机器人里程计位姿 | `/odom` | `nav_msgs/msg/Odometry` | 网页后端订阅，并经 `map -> odom` TF 转为地图坐标 |
| 地图 | `/map` | `nav_msgs/msg/OccupancyGrid` | 网页后端订阅 |
| 可执行主路径 | `/plan` | `nav_msgs/msg/Path` | 规划器生成、导航执行使用，网页主路径显示 |
| 控制器局部预览 | `/transformed_global_plan` | `nav_msgs/msg/Path` | 可选局部预览；兼容 `/local_plan`、`/plan_smoothed` |

`nav_output_state` 的值：

| 状态 | 含义 | 网页行为 |
|---|---|---|
| `disabled` | 导航不会向底盘发布速度 | 允许手动底盘控制 |
| `armed_idle` | 导航已获得控制权，但当前没有运动输出 | 禁用手动控制，允许发送导航目标 |
| `active` | 导航正在发布非零速度 | 禁用手动控制，显示导航运行中 |

该状态话题使用 transient-local QoS，网页后端订阅时也必须请求
transient-local durability，才能在重启后立即读取最后状态。该话题只在状态变化时
发布，不是周期心跳；节点健康度应通过 service 可用性和 ROS graph 单独判断。

`/navigation/status` 是网页判断导航是否可用的首选接口，以 2 Hz 周期发布 JSON，
同时使用 transient-local QoS。网页使用 `/odom` 和 `map -> odom` TF 获取机器人定位：机器人
静止时 AMCL 可能长时间不发布新消息，但定位 TF 仍然有效。

状态示例：

```json
{
  "version": 1,
  "state": "ready",
  "message": "导航已就绪",
  "navigation_ready": true,
  "localization_ready": true,
  "navigation_lifecycle_active": true,
  "nav_output_service_available": true,
  "nav_output_state": "disabled",
  "inputs": {"map": true, "odom": true, "scan": true},
  "localization_spin": {
    "pending": false,
    "running": false,
    "progress_rad": 0.0,
    "target_yaw_rad": 6.2832,
    "last_result": "never_started"
  },
  "last_error": null
}
```

主要 `state` 值：

| 状态 | 含义 |
|---|---|
| `waiting_odom` | 等待里程计 |
| `waiting_map` | 等待地图 |
| `waiting_scan` | 等待激光扫描 |
| `waiting_localization` | 等待 `map -> body` TF |
| `waiting_nav2` | 定位完成，等待 Nav2 lifecycle active |
| `ready` | 导航和旋转定位功能均可用 |
| `localization_spinning` | 正在执行原地旋转定位 |

## 3. 网页状态机

网页和后端统一使用以下四种 UI 状态，不要只根据按钮颜色判断控制权：

```text
MANUAL      手动模式，期望 ROS 状态为 disabled
SWITCHING   正在交接控制权，所有运动按钮暂时禁用
NAVIGATION  导航模式，ROS 状态为 armed_idle 或 active
FAULT       切换过程中 service 消失、状态确认超时或 ROS 数据异常
```

后端是模式状态的唯一真源。多个浏览器同时连接时，所有页面显示同一个后端状态，
不得由每个浏览器分别维护模式。

## 4. “原地旋转定位”按钮

网页在收到 `/navigation/status` 且满足以下条件时显示或启用按钮：

- `navigation_ready == true`；
- `state == "ready"`；
- `localization_spin.running == false`；
- 当前没有网页手动速度输入。

点击按钮时：

1. 停止网页手动速度定时器并发送一帧零速度；
2. UI 进入“旋转定位中”，禁用摇杆、导航目标和模式切换；
3. 调用 `/navigation/start_localization_spin`，类型 `std_srvs/srv/Trigger`；
4. service 返回 `success: true` 后，持续显示
   `/navigation/status.localization_spin.progress_rad`；
5. `last_result` 变为 `succeeded`、`cancelled` 或 `failed` 后结束提示。

机器人端会自动完成以下动作，网页不需要再调用控制权 service：

- 临时启用导航底盘输出；
- 调用 Nav2 `/spin` action，目标旋转约 `2π`，即原地一整圈；
- 最大旋转速度限制为约 `0.5 rad/s`；
- 成功、失败或取消后自动将导航底盘输出恢复为 `disabled`。

网页必须提供“停止旋转”按钮，调用：

```text
/navigation/cancel_localization_spin
std_srvs/srv/Trigger
```

旋转前应提示用户确认机器人周围无人员和障碍物。旋转定位完成后网页仍处于手动
控制权状态；如需开始自主导航，再按正常流程切换到导航模式。

## 5. 切换到导航模式

必须按顺序执行：

1. UI 进入 `SWITCHING`，禁用摇杆、方向键和导航目标按钮。
2. 停止手动速度定时器。
3. 网页后端向手动速度话题发送一帧零速度。
4. 调用 `SetBool`，请求 `data: true`。
5. 必须等待 service 返回 `success: true`。
6. 等待 `/cmd_vel_to_twist_stamped/nav_output_state` 变为 `armed_idle` 或
   `active`，建议超时 1 秒。
7. 状态确认后进入 `NAVIGATION`，启用地图目标点功能。

service 请求：

```yaml
data: true
```

成功响应示例：

```yaml
success: true
message: "nav chassis output armed_idle; ready in 0.30s"
```

桥接节点内部还有约 0.3 秒交接延迟。网页不需要额外 sleep，但必须等待状态确认，
且在导航模式下停止发布所有手动速度，包括零速度。

## 6. 切换到手动模式

必须按顺序执行：

1. UI 进入 `SWITCHING`，停止接受新的导航目标。
2. 如果有活动导航任务，先发起取消。取消建议见第 9 节。
3. 调用 `SetBool`，请求 `data: false`。
4. 必须等待 service 返回 `success: true`。
5. 等待 `nav_output_state` 变为 `disabled`，建议超时 1 秒。
6. 状态确认后进入 `MANUAL`，再启用摇杆和方向键。

service 请求：

```yaml
data: false
```

如果导航正在运动，桥接节点会在 service 返回前发布一帧停止命令；如果导航本来
空闲，则不会发布多余零速度。因此网页必须等 service 完成后再开始发送非零手动
速度，不能“调用 service 的同时立即发速度”。

## 7. 手动速度发布规则

网页手动控制直接发布最终底盘控制话题，因此必须遵守以下规则：

- 只有后端模式为 `MANUAL` 且 ROS 状态为 `disabled` 时才能发布非零速度；
- 按住摇杆/方向键时建议以 10–20 Hz 发布；
- 松开控制、失去焦点或手动控制 WebSocket 断开时，立即发送一帧零速度；
- 松开后停止发布，不要持续以固定频率发布零速度；
- 在 `NAVIGATION`、`SWITCHING`、`FAULT` 状态下禁止发布手动速度；
- 发布器使用 volatile durability，避免新订阅者收到历史速度；
- `header.frame_id` 使用 `body`，时间戳使用 ROS 当前时间。

示例消息：

```yaml
header:
  frame_id: body
twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.3
```

网页后端还应实现手动控制 deadman：超过 200–300 ms 没收到新的摇杆输入，就发布
一帧零速度并结束当前手动运动。

## 8. 发送导航目标

兼容现有实现时，网页后端继续向 `/goal_pose` 发布 `PoseStamped`：

```yaml
header:
  frame_id: map
  stamp:
    sec: 0
    nanosec: 0
pose:
  position:
    x: 1.5
    y: 0.8
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

导航包会把 `/goal_pose` 过滤后转发到 `/goal_pose_nav`。只有 `map -> body` TF
可用时目标才会进入 Nav2。

网页允许发送目标前至少检查：

- 后端模式为 `NAVIGATION`；
- `nav_output_state` 为 `armed_idle` 或 `active`；
- `/navigation/status.navigation_ready == true`；
- 目标坐标和朝向均为有限数值；
- 四元数已归一化。

如果条件不满足，应在网页提示“导航尚未定位完成”，不能静默发送目标。

## 9. 取消导航

仅停止导航底盘输出并不会删除 Nav2 中的活动目标。若网页切到手动后又重新启用
导航，旧目标可能继续执行，因此进入手动模式前必须取消活动目标。

最低兼容方案是在后端增加 `CancelGoal` client：

```text
Service: /navigate_to_pose/_action/cancel_goal
Type:    action_msgs/srv/CancelGoal
```

发送全零 `goal_info` 表示取消所有活动的 `NavigateToPose` 目标。更推荐的长期方案是
网页后端直接使用 `nav2_msgs/action/NavigateToPose` ActionClient 发送目标，并保存
goal handle，用它获取反馈、结果和执行精确取消。

切换到手动模式时，即使取消请求失败，也必须继续执行导航输出禁用；UI 同时显示
“导航任务取消失败，底盘控制已切为手动”的告警。

## 10. 建议的网页后端 API

以下 HTTP/WS 接口是网页后端的建议封装，不是 ROS 已有接口：

```text
GET  /api/navigation/state
POST /api/navigation/mode       {"mode": "manual" | "navigation"}
POST /api/navigation/goal       {"x": 1.5, "y": 0.8, "yaw": 0.0}
POST /api/navigation/clear-global-costmap
POST /api/navigation/cancel
POST /api/navigation/stop
POST /api/navigation/localization-spin/start
POST /api/navigation/localization-spin/cancel
WS   /ws/navigation/state
```

`POST /api/navigation/clear-global-costmap` 调用 Nav2 的
`/global_costmap/clear_entirely_global_costmap`（`nav2_msgs/srv/ClearEntireCostmap`），
只清除一次全局代价地图中的临时障碍物，不修改静态 `/map`。导航页按钮需要先通过本页
安全锁；服务不可用时后端返回错误，不会伪造清除成功。

`GET /api/navigation/state` 建议返回：

```json
{
  "mode": "navigation",
  "transitioning": false,
  "nav_output_state": "armed_idle",
  "service_available": true,
  "localized": true,
  "map_ready": true,
  "active_goal": false,
  "last_error": null
}
```

模式切换 API 必须在后端使用互斥锁串行执行，避免两个浏览器同时触发相反的切换。

## 11. 急停按钮

网页急停与普通模式切换不同，应优先停车：

1. 立即向手动速度话题发送一帧零速度；
2. 同时调用导航输出 service，设置 `data: false`；
3. service 返回后再次发送一帧零速度；
4. 取消活动导航目标；
5. UI 保持 `FAULT` 或急停锁定状态，禁止自动恢复导航。

急停解除后只能由用户显式选择 `MANUAL` 或 `NAVIGATION`，不能自动启用导航。

## 12. 断线和异常处理

- 页面启动时导航输出 service 不存在：进入“仅手动模式”，禁用导航按钮，但基础
  底盘健康时仍允许手动控制；
- 已处于 `NAVIGATION` 或 `SWITCHING` 时 service 突然消失：进入 `FAULT`，发送
  一帧手动零速度并禁止新的非零速度；
- 后端启动后 2 秒内既未收到 transient-local 状态，也无法确认 service：按导航
  未启动处理，不要猜测或自动启用导航；
- service 成功但 1 秒内状态未切换：进入 `FAULT`，禁止所有非零速度；
- 手动控制 WebSocket 断开：触发 deadman，发送一帧零速度；
- 普通浏览器刷新不应直接改变后端模式；模式由后端保存并通过状态接口恢复；
- 后端进程重启后，先读取 transient-local 状态，再决定 UI 模式，禁止默认启用导航；
- 收到未知 `nav_output_state` 值时按 `FAULT` 处理。

## 13. 前端显示建议

页面至少增加：

- “手动控制 / 导航”模式切换；
- 当前控制权标签：`手动`、`导航待命`、`导航运动中`、`切换中`、`故障`；
- 定位状态和最后更新时间；
- 导航目标、取消导航和急停按钮；
- 切换失败、定位未完成、目标被拒绝等明确错误信息。

控件使能规则：

| UI 状态 | 手动摇杆 | 地图发目标 | 取消导航 | 急停 |
|---|---:|---:|---:|---:|
| `MANUAL` | 开 | 关 | 关 | 开 |
| `SWITCHING` | 关 | 关 | 关 | 开 |
| `NAVIGATION` | 关 | 开 | 开 | 开 |
| `FAULT` | 关 | 关 | 视情况 | 开 |

## 14. 后端伪代码

```python
async def switch_to_navigation():
    async with mode_lock:
        set_ui_state("SWITCHING")
        stop_manual_publish_timer()
        publish_manual_zero_once()

        response = await set_nav_output_enabled(True, timeout=1.0)
        if not response.success:
            raise NavigationSwitchError(response.message)

        await wait_nav_state({"armed_idle", "active"}, timeout=1.0)
        set_backend_mode("NAVIGATION")


async def switch_to_manual():
    async with mode_lock:
        set_ui_state("SWITCHING")
        block_new_navigation_goals()

        try:
            await cancel_active_navigation(timeout=1.0)
        except Exception as exc:
            report_warning(f"cancel failed: {exc}")

        response = await set_nav_output_enabled(False, timeout=1.0)
        if not response.success:
            raise NavigationSwitchError(response.message)

        await wait_nav_state({"disabled"}, timeout=1.0)
        set_backend_mode("MANUAL")
        enable_manual_controls()
```

## 15. 验收清单

1. 导航未启动时，网页显示导航不可用，手动控制仍可使用。
2. 导航启动后的初始状态为 `disabled`，网页手动控制正常。
3. 切换导航模式后，网页停止发布手动速度，状态变为 `armed_idle`。
4. 发送目标后状态变为 `active`，机器人按 Nav2 路径运动。
5. 导航空闲时，网页手动速度话题没有持续零速度输出。
6. 导航运动中切手动，机器人先停车，状态变为 `disabled`，然后手动控制生效。
7. 手动模式松开摇杆只发送一帧零速度，之后停止发布。
8. 从手动重新切导航时，0.3 秒交接期间没有速度冲突。
9. 取消导航后重新启用导航，不会恢复旧目标。
10. 两个浏览器同时切换模式时，后端能串行处理且最终状态一致。
11. 网页断线时手动运动能被 deadman 停止。
12. 急停后导航不会自行恢复。
13. 导航状态 `ready` 后可以触发原地旋转，进度持续更新。
14. 旋转完成或取消后 `nav_output_state` 自动恢复为 `disabled`。

## 16. 命令行联调

查看控制权状态：

```bash
ros2 topic echo /cmd_vel_to_twist_stamped/nav_output_state
```

查看网页综合导航状态：

```bash
ros2 topic echo /navigation/status std_msgs/msg/String \
  --qos-durability transient_local
```

开始/取消原地旋转定位：

```bash
ros2 service call /navigation/start_localization_spin std_srvs/srv/Trigger '{}'
ros2 service call /navigation/cancel_localization_spin std_srvs/srv/Trigger '{}'
```

切换到导航控制：

```bash
ros2 service call \
  /cmd_vel_to_twist_stamped/set_nav_output_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

切换到网页手动控制：

```bash
ros2 service call \
  /cmd_vel_to_twist_stamped/set_nav_output_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

检查底盘控制话题的发布者：

```bash
ros2 topic info /motion_target/target_speed_chassis -v
```

导航模式下可以同时看到网页和导航发布端存在，但网页后端必须停止发送手动速度；
真正的互斥由模式切换流程和 `nav_output_state` 保证。
