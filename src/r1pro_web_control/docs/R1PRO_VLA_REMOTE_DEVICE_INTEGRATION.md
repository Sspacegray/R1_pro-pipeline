# R1 Pro VLA 跨设备控制接入说明

本文说明另一台设备如何实现与当前 R1 Pro Web 页面相同的 VLA 控制能力。内容以当前代码中的 `r1pro.v2` 协议为准，覆盖网络、HTTP、OpenPI WebSocket、ROS 2 机器人接口、数据格式、控制循环和安全要求。

适用代码：

- `r1pro_web_control/r1pro_web_control/server_node.py`
- `r1pro_web_control/r1pro_web_control/vla_helper.py`
- `r1pro_web_control/web/app.js`
- `vla_openpi/client_example.py`
- `vla_openpi/openpi/packages/openpi-client`

> 重要：`vla_openpi/r1pro_robot_runtime.md` 和 `vla_openpi/main.py` 仍包含旧的 `r1pro.v1` 字段。新设备不要使用其中的 `observation/...`、`prompt`、`load_r1pro_task()` 或 RTC 接口。当前 Web 只接受 `r1pro.v2`。

## 1. 结论与推荐方案

推荐让现有 `r1pro_web_server` 继续运行在能够访问机器人 ROS 2 网络和 Policy Server 的主机上，另一台设备只调用其 HTTP API：

```text
另一台设备
  │ HTTP JSON，端口 8080
  ▼
r1pro_web_server（机器人侧网关）
  ├─ 采集 ROS 2 三相机和关节反馈
  ├─ WebSocket 调用 OpenPI Policy Server，端口 18081
  ├─ 校验 inference_info 和动作语义
  └─ 发布 ROS 2 JointState 目标到 R1 Pro
```

这种方式不要求另一台设备安装 ROS 2、Fast DDS 或 OpenPI，也复用了现有的相机同步、状态历史、动作维度映射、100 Hz 插值发布、订阅者检查和停止逻辑。

只有在不能继续运行现有 Web 网关时，才需要按第 9 节在另一台设备上完整实现 ROS 2 + OpenPI 客户端。

## 2. 部署参数与网络

不要在程序中写死地址。建议使用下列配置项：

| 配置项 | 含义 | 当前代码/文档中的值 |
|---|---|---|
| `WEB_BASE_URL` | `r1pro_web_server` HTTP 地址 | `http://<Web主机IP>:8080`；进程监听 `0.0.0.0:8080` |
| `POLICY_HOST` | OpenPI Policy Server 地址 | 代码默认 `10.30.20.1`；当前 README 启动示例覆盖为 `10.30.20.73` |
| `POLICY_PORT` | Policy Server WebSocket/health 端口 | `18081` |
| `ROS_DOMAIN_ID` | 真机 ROS 2 Domain | `102` |
| `ROS_RMW` | ROS 2 中间件 | `rmw_fastrtps_cpp` |
| Web 主机机器人网卡 | 连接 R1 Pro DDS 网络的网卡 | 当前记录为 `192.168.3.5` |
| `CONTROL_TOKEN` | Web 写接口令牌 | Web 主机 `~/.ros/r1pro_web_control/token` |

当前拓扑通常要求网关同时能访问：

- 机器人 DDS 网络：`192.168.3.x`；
- Policy Server 网络：`10.30.20.x`；
- 另一台设备到网关的 HTTP 网络。

接入前确认：

```bash
# 在另一台设备上
curl http://<Web主机IP>:8080/api/state

# 在能够访问 Policy Server 的设备上
curl http://<Policy主机IP>:18081/healthz
nc -vz <Policy主机IP> 18081
```

HTTP 和 Policy WebSocket 默认均为明文协议，只应开放在可信局域网或 VPN 中。不要把 8080、18081 和 DDS 端口直接暴露到公网。

## 3. 方案 A：通过 Web HTTP 网关接入

### 3.1 鉴权和通用规则

`GET /api/state` 不需要 token。所有 `POST` 请求都必须携带：

```http
Content-Type: application/json
X-Control-Token: <CONTROL_TOKEN>
```

token 在 Web 主机上读取：

```bash
cat ~/.ros/r1pro_web_control/token
```

接口成功时返回 HTTP 200 和 `{"ok":true,...}`；业务失败通常返回 HTTP 400 和 `{"ok":false,"message":"..."}`；token 错误返回 HTTP 401。

浏览器里的“安全锁”只存在于前端。直接调用 API 会绕过该 UI 锁，因此另一台设备必须自己实现“确认现场安全后才允许执行”的互锁。

### 3.2 状态接口

```http
GET /api/state
```

重点读取：

- `active_control_profile`：应为 `real`；
- `control_profiles.profiles.real.online`：真机反馈是否在线；
- `feedback.<group>.positions`：ROS 原始值，关节为弧度，夹爪为 0–100；
- `feedback.<group>.age_s`：反馈年龄；
- `images`、`image_hz`、`image_meta`：相机年龄、频率和尺寸；
- `vla.remote_tasks`：Policy Server 返回的任务；
- `vla.task_loaded`、`vla.loaded_task`：当前加载状态；
- `vla.task_metadata`：经过网关校验的推理/执行契约；
- `vla.last_error`：最近一次 VLA 错误；
- `topics.robot_feedback`、`topics.robot_targets`：当前实际使用的话题。

建议执行前要求每个将被控制的反馈组均存在且足够新。当前 Web 用 1 秒作为在线判断阈值；实际控制程序可采用更严格的 0.3–0.5 秒阈值。

### 3.3 连接 Policy Server

```http
POST /api/vla/connect

{
  "host": "10.30.20.73",
  "port": 18081,
  "timeout_s": 30
}
```

网关会：

1. 建立 WebSocket；
2. 检查握手中的 `protocol_version == "r1pro.v2"`；
3. 发送 `{"command":"list_tasks"}`；
4. 将服务端任务写入响应的 `remote_tasks`。

任务列表必须以服务端响应为准。不要在客户端维护固定任务名、prompt 或 checkpoint 路径。

### 3.4 加载任务并读取契约

```http
POST /api/vla/load_task

{
  "task": "<list_tasks 返回的 task id>",
  "host": "10.30.20.73",
  "port": 18081,
  "force_reload": true,
  "timeout_s": 300
}
```

加载 checkpoint 可能需要数分钟。响应中的以下字段必须保存：

- `inference_info`：Policy Server 原始契约；
- `task_metadata.server_control_schema`；
- `task_metadata.state_schema`；
- `task_metadata.action_horizon`；
- `task_metadata.control_frequency_hz`；
- `task_metadata.action_dim_names`；
- `task_metadata.action_group_indices`；
- `task_metadata.available_execution_schemas`；
- `execution_allowed` 和 `execution_disabled_reason`。

只有 `execution_allowed == true` 时才能把模型结果发给真机。契约不兼容时仍可能允许推理和画图，但必须禁止执行。

### 3.5 发起一次在线推理

```http
POST /api/vla/infer

{
  "task": "<task id>",
  "input_source": "live",
  "state_source": "feedback",
  "profile": "real",
  "execution_schema": "server",
  "host": "10.30.20.73",
  "port": 18081
}
```

客户端不传图像和 state。网关会从 ROS 2 采集三路图像和关节反馈，按契约组成 observation。任务刚加载或尚未 warmup 时，第一次 `/infer` 会先自动执行 2 次 warmup，再执行正式推理，因此超时至少应设为 120 秒。

主要响应字段：

```json
{
  "ok": true,
  "result": {
    "actions": [[0.0]],
    "action_shape": [10, 8],
    "action_schema": "left_arm_8",
    "selected_task": "...",
    "checkpoint_step": 123,
    "server_timing": {},
    "policy_timing": {},
    "task_metadata": {}
  },
  "observation": {
    "state": [],
    "state_meta": {},
    "image_meta": {},
    "action_schema": "left_arm_8"
  }
}
```

`actions` 的具体 shape 必须以本次响应和契约为准，上例的 `[10,8]` 只是示意。

### 3.6 执行第一步或整个 chunk

执行模型输出时必须设置 `model_actions:true`，让网关按已加载契约校验绝对动作语义：

```http
POST /api/vla/execute

{
  "actions": [[0.0]],
  "mode": "chunk",
  "profile": "real",
  "action_schema": "left_arm_8",
  "execution_schema": "server",
  "model_actions": true,
  "observation_state": []
}
```

其中：

- `actions`：直接复制 `/infer` 返回的完整二维数组；
- `action_schema`：复制 `/infer.result.action_schema`；
- `observation_state`：复制 `/infer.observation.state`；
- `mode:"first"`：只执行第一行动作，并在默认 3 秒内持续发布；
- `mode:"chunk"`：执行整个 action chunk；
- `execution_schema:"server"`：执行服务端声明的全部控制组；
- 也可从 `available_execution_schemas` 中选 `left_arm_8`、`right_arm_8` 等，只影响 ROS 下发组，不裁剪模型输入和推理输出。

对于模型动作，网关忽略客户端传入的 `rate_hz` 和 `duration_s`，严格采用契约的 `control_frequency_hz`，chunk 时长为：

```text
duration_s = action_horizon / control_frequency_hz
```

### 3.7 完整任务循环

当前 Web 的“执行完整任务”逻辑是 action-chunk 闭环，不是 RTC：

```text
重置任务 episode 状态
  -> 采集同步图像和反馈
  -> 推理一个 action chunk
  -> 执行整个 chunk
  -> 等待 chunk 时长 + settle_s
  -> 用新反馈继续下一次推理
  -> 达到 max_chunks、用户停止或发生错误时退出
```

第一轮 `/infer` 增加：

```json
{"history_mode":"previous_infer","reset_history":true}
```

后续轮增加：

```json
{"history_mode":"previous_infer","reset_history":false}
```

默认 Web 参数是 `max_chunks=20`、`settle_s=0.5`。这只是上限，不是模型成功判据；如任务有可靠完成检测，应提前结束。

可直接参考以下 Python 流程：

```python
import os
import time
import requests

WEB = os.environ["WEB_BASE_URL"].rstrip("/")
TOKEN = os.environ["R1PRO_CONTROL_TOKEN"]
POLICY_HOST = os.environ["R1PRO_POLICY_HOST"]
POLICY_PORT = int(os.environ.get("R1PRO_POLICY_PORT", "18081"))
TASK = os.environ["R1PRO_TASK"]
HEADERS = {"X-Control-Token": TOKEN, "Content-Type": "application/json"}


def post(path, body, timeout):
    response = requests.post(WEB + path, headers=HEADERS, json=body, timeout=timeout)
    response.raise_for_status()
    result = response.json()
    if not result.get("ok", False):
        raise RuntimeError(result.get("message", "request failed"))
    return result


post("/api/vla/connect", {
    "host": POLICY_HOST, "port": POLICY_PORT, "timeout_s": 30,
}, 35)

loaded = post("/api/vla/load_task", {
    "task": TASK, "host": POLICY_HOST, "port": POLICY_PORT,
    "force_reload": True, "timeout_s": 300,
}, 305)
if not loaded.get("execution_allowed", False):
    raise RuntimeError(loaded.get("execution_disabled_reason", "execution disabled"))

base_infer = {
    "task": TASK,
    "input_source": "live",
    "state_source": "feedback",
    "profile": "real",
    "execution_schema": "server",
    "host": POLICY_HOST,
    "port": POLICY_PORT,
    "history_mode": "previous_infer",
}

try:
    for chunk_index in range(20):
        inferred = post("/api/vla/infer", {
            **base_infer, "reset_history": chunk_index == 0,
        }, 125)
        result = inferred["result"]
        metadata = result["task_metadata"]
        if not metadata.get("execution_allowed", False):
            raise RuntimeError(metadata.get("execution_disabled_reason", "execution disabled"))

        post("/api/vla/execute", {
            "actions": result["actions"],
            "mode": "chunk",
            "profile": "real",
            "action_schema": result["action_schema"],
            "execution_schema": "server",
            "model_actions": True,
            "observation_state": inferred["observation"]["state"],
        }, 15)

        horizon = int(metadata["action_horizon"])
        rate_hz = float(metadata["control_frequency_hz"])
        time.sleep(horizon / rate_hz + 0.5)
finally:
    # 软件停止；不替代机器人硬件急停。
    post("/api/control/stop", {}, 5)
```

### 3.8 停止

```http
POST /api/control/stop

{}
```

它会停止所有 Web 侧关节/夹爪发布线程、清空 VLA 状态历史并停止底盘命令。它是软件停止，不等价于切断驱动器使能或机器人硬件急停。人员可能进入机器人工作区时必须使用硬件急停和现场安全流程。

## 4. 当前 `r1pro.v2` OpenPI 协议

本节供方案 B 或其他语言客户端实现者使用。方案 A 的调用者不需要直接处理该协议。

### 4.1 传输

- 地址：`ws://<POLICY_HOST>:18081`；
- health：`http://<POLICY_HOST>:18081/healthz`；
- WebSocket compression 关闭；
- 消息使用 MessagePack，不是 JSON；
- 连接建立后，服务端发送的第一帧是 metadata；
- NumPy 数组由 `openpi_client.msgpack_numpy` 编解码；
- Python 客户端优先直接使用 `openpi_client.websocket_client_policy.WebsocketClientPolicy`。

官方 `WebsocketClientPolicy` 在连接拒绝时会持续重试。生产程序应在外层增加 TCP 探测、总超时和取消机制。

### 4.2 命令顺序

```python
from openpi_client import websocket_client_policy

client = websocket_client_policy.WebsocketClientPolicy(
    host=POLICY_HOST, port=POLICY_PORT
)

metadata = client.get_server_metadata()
assert metadata["protocol_version"] == "r1pro.v2"

tasks = client.infer({"command": "list_tasks"})
loaded = client.infer({"command": "load_task", "task": selected_task})
inference_info = loaded["inference_info"]

result = client.infer({
    "images": {
        "cam_high": head_rgb,
        "cam_left_wrist": left_rgb,
        "cam_right_wrist": right_rgb,
    },
    "state": state_float32,
})
actions = result["actions"]
```

每次显式 `load_task` 都可能重新加载 checkpoint。不要在实时循环中切换任务。连接中断后重新建立连接、重新加载同一任务、重新读取 `inference_info` 并重新 warmup。

### 4.3 禁用的旧字段

`r1pro.v2` observation 只能包含顶层 `images` 和 `state`。不要发送：

- `prompt`；
- `observation/head_image`；
- `observation/left_wrist_image`；
- `observation/right_wrist_image`；
- `observation/state`；
- `prev_chunk_left_over`；
- `inference_delay` 或其他 RTC 字段。

当前 v2 明确不支持 RTC。

## 5. `inference_info` 是唯一动作契约

客户端不能根据任务名、checkpoint 名或数组长度猜测控制含义。至少校验：

| 字段 | 要求 |
|---|---|
| `control_schema` | `left_arm_8`、`right_arm_8`、`bimanual_16`、`upper_body_20` 之一 |
| `controlled_groups` | 必须与 schema 对应组完全一致且顺序一致 |
| `controlled_arm` | 单臂为 `left`/`right`；双臂通常为 `both` |
| `control_frequency_hz` | 有限且大于 0；执行 chunk 必须使用此频率 |
| `action_dim_names` | 长度等于 action 维度，名称唯一 |
| `gripper_indices` | 必须与 `action_dim_names` 中夹爪位置一致 |
| `joint_action_space` | 当前真机执行只接受 `absolute_position_radians` |
| `gripper_action_space` | 当前真机执行只接受 `absolute_0_to_100` |
| `server_output_transform` | 当前真机执行只接受 `absolute_actions` |
| `client_must_unnormalize` | 必须为 `false` |
| `client_must_add_state` | 必须为 `false` |
| `inputs.state.shape` | 一维正整数 shape |
| `inputs.state.dim_names` | 长度和顺序必须与 state shape 一致 |
| `outputs.actions.shape` | `[horizon, action_dim]` |
| `outputs.actions.dtype` | 通常为 `float32` |

同一字段可能同时出现在顶层和 `outputs.actions` 中；若两处都有，必须一致。

### 5.1 标准控制布局

| schema | 维度顺序 |
|---|---|
| `left_arm_8` | 左臂 joint1–7，左夹爪 |
| `right_arm_8` | 右臂 joint1–7，右夹爪 |
| `bimanual_16` | 左臂 joint1–7，左夹爪，右臂 joint1–7，右夹爪 |
| `upper_body_20` | 躯干 joint1–4，左臂 8 维，右臂 8 维 |

但服务端允许通过 `action_dim_names` 改变组的排列。因此执行时应按名称建立索引，不要永远假设左臂在 0–7。`left_arm_joint_1` 与 `left_arm_joint1` 在当前 Web 中视为同一名称。

### 5.2 动作单位

- 躯干和手臂：绝对关节位置，单位 rad；
- 夹爪：绝对开度，0–100；执行前 clamp 到 `[0,100]`；
- 服务端已经完成反归一化和需要的输出变换；
- 不得把 action 加到当前 state；
- 不得把模型输出从度转换成弧度。

## 6. 图像输入

### 6.1 ROS 2 相机接口

| 模型字段 | ROS 2 topic | 消息类型 |
|---|---|---|
| `images.cam_high` | `/hdas/camera_head/left_raw/image_raw_color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `images.cam_left_wrist` | `/hdas/camera_wrist_left/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `images.cam_right_wrist` | `/hdas/camera_wrist_right/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |

头部使用左目 raw 图像，不使用右目，也不使用 Web 的 rectified 深度图。

### 6.2 预处理

三路图像全部必需：

1. 解码 `CompressedImage.data`；
2. 转为 RGB；
3. 保持宽高比缩放并用黑色 padding 到 `224×224`；
4. 转为 `numpy.uint8`，shape 为 `(224,224,3)`；
5. 以 `cam_high`、`cam_left_wrist`、`cam_right_wrist` 三个键发送。

不要直接拉伸到正方形。当前实现使用：

```python
image_tools.convert_to_uint8(
    image_tools.resize_with_pad(rgb_uint8, 224, 224)
)
```

### 6.3 真机相机同步

当前 Web 保存每路最近 60 帧，并按 ROS header 时间戳选择三路最接近的一组。真机要求三路最大时间差严格小于 `0.08 s`；找不到时最多等待 3 轮新帧，每轮最长 0.5 秒，仍不同步则拒绝推理。

独立客户端应采用相同规则。不要简单拿三路“各自最新帧”而忽略时间戳。

## 7. 机器人状态接口

| 控制组 | 反馈 topic | 消息类型 | `position` 数量/单位 |
|---|---|---|---|
| torso | `/hdas/feedback_torso` | `sensor_msgs/msg/JointState` | 4，rad |
| left arm | `/hdas/feedback_arm_left` | `sensor_msgs/msg/JointState` | 7，rad |
| left gripper | `/hdas/feedback_gripper_left` | `sensor_msgs/msg/JointState` | 1，0–100 |
| right arm | `/hdas/feedback_arm_right` | `sensor_msgs/msg/JointState` | 7，rad |
| right gripper | `/hdas/feedback_gripper_right` | `sensor_msgs/msg/JointState` | 1，0–100 |

`/joint_states` 只作为 Web 在专用反馈暂时不新鲜时的手臂/躯干 fallback，不建议作为新控制器的主输入。夹爪仍应读取专用反馈话题。

状态顺序以 `inference_info.inputs.state.dim_names` 为准。先按 schema 从反馈组构造物理状态，再按 `dim_names` 重排为模型输入。所有值必须为有限 `float32`，shape 必须精确匹配。

### 7.1 带历史的 state

若契约声明：

```text
state_history_enabled = true
state_encoding = current_and_previous
state_component_order = [current, previous]
```

则当前 Web 的兼容行为是：

- episode 第一次推理：`state = concat(current, current)`；
- 后续推理：`state = concat(current, previous_infer_current)`；
- `previous` 是上一次正式推理时的 current feedback，不是 warmup state，也不是从连续缓存中按墙上时间查到的一秒前样本；
- 切任务、重连、全局 STOP 或新 episode 时清空历史；
- warmup 使用首帧 padding，不推进正式 episode 历史。

例如当前代码对 `water_potted_plant_02` 额外要求：左臂 8 维基础状态、16 维输入、`[10,8]` 动作、历史标称 1.0 秒。其他任务仍必须完全按服务端实际契约处理。

## 8. 机器人目标接口与发布方式

### 8.1 ROS 2 目标话题

| 控制组 | 目标 topic | 消息类型 | `position` 数量/单位 |
|---|---|---|---|
| torso | `/motion_target/target_joint_state_torso` | `sensor_msgs/msg/JointState` | 4，绝对 rad |
| left arm | `/motion_target/target_joint_state_arm_left` | `sensor_msgs/msg/JointState` | 7，绝对 rad |
| left gripper | `/motion_target/target_position_gripper_left` | `sensor_msgs/msg/JointState` | 1，绝对 0–100 |
| right arm | `/motion_target/target_joint_state_arm_right` | `sensor_msgs/msg/JointState` | 7，绝对 rad |
| right gripper | `/motion_target/target_position_gripper_right` | `sensor_msgs/msg/JointState` | 1，绝对 0–100 |

Web 发布的消息格式：

```python
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = []
msg.position = [float_value, ...]
msg.velocity = []
msg.effort = []
```

目标 publisher 的 QoS 必须兼容当前 Web：

```text
reliability = RELIABLE
history     = KEEP_LAST
depth       = 10
durability  = TRANSIENT_LOCAL
```

反馈和压缩相机订阅使用 ROS 2 `qos_profile_sensor_data`。

### 8.2 Web 的 chunk 执行方式

当前 Web 不只以模型频率发布一次。它在 `action_horizon / control_frequency_hz` 的总时长内，对相邻 waypoint 做 smoothstep 插值，并以 `command_rate_hz=100 Hz` 持续发布，最后再发布一次最终 waypoint。

独立控制器若要尽量复现 Web 行为，应：

1. 按 `action_dim_names` 拆成各控制组；
2. 对夹爪 clamp 到 `[0,100]`；
3. 在整个 chunk 时长内以约 100 Hz 插值并重复发布；
4. 同一控制组的新 stream 到来时取消旧 stream；
5. chunk 结束后再发一次最后目标；
6. 不向未选中的执行组发布“保持当前位置”，否则会改变 `execution_schema` 的语义。

## 9. 方案 B：另一台设备直接接 ROS 2 和 Policy Server

### 9.1 ROS 2 / DDS 环境

当前真机环境是 ROS 2 Humble、Python 3.10、Fast DDS：

```bash
source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_workstation_udp.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
```

仓库模板：

```text
r1pro_web_control/config/fastdds_workstation_udp.xml
```

该模板的 `interfaceWhiteList` 当前写死 `192.168.3.5`。复制到另一台设备后，必须把它改成该设备连接机器人 LAN 的实际网卡 IPv4 地址；直接照搬会导致 ROS graph 看不到机器人。不要使用 `LARGE_DATA` profile，机器人侧当前是 UDP-only Fast DDS。

### 9.2 必需组件

独立控制器至少需要：

- ROS 2 Humble `rclpy`；
- `sensor_msgs/msg/JointState` 和 `sensor_msgs/msg/CompressedImage`；
- NumPy、Pillow 或 OpenCV；
- 仓库 `vla_openpi/openpi/packages/openpi-client`；
- 能同时访问机器人 DDS LAN 与 Policy Server；
- 机器人端已经启动对应 arm joint tracker。

机器人端 joint tracker 是 aarch64 实机组件，例如 `r1_pro_jointTrackerdemo_launch.py`，不能在 x86 外部设备上代替运行。

### 9.3 独立状态机

```text
INIT
  -> 检查硬件急停和软件控制权
  -> 等待三相机、所有所需反馈、所有目标话题订阅者
  -> 连接 Policy，校验 r1pro.v2
  -> list_tasks，由操作者选择任务
  -> load_task，完整校验 inference_info
  -> 清空 action/history buffer
  -> 2 次 warmup（不执行、不计入 history）
  -> 采集同步 observation
  -> infer
  -> 校验 actions shape、有限值和契约未变化
  -> 100 Hz 插值发布一个 chunk
  -> 等待新反馈并循环
  -> 任一异常：停止发布、清空历史、进入 FAULT，等待人工恢复
```

任务切换、Policy 重连、契约变化、反馈超时、相机超时、shape 不符、NaN/Inf、目标话题无订阅者都必须停止当前执行。

## 10. 真机启动和接口检查

Web 主机的已验证环境配置示例：

```bash
source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash
source /home/motrix/galaxea/galaxea_overlay/install/setup.bash

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/motrix/galaxea/galaxea_overlay/install/r1pro_web_control/share/r1pro_web_control/config/fastdds_workstation_udp.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

ros2 run r1pro_web_control r1pro_web_server --ros-args \
  -p vla_policy_host:=10.30.20.73 \
  -p vla_policy_port:=18081
```

检查 ROS 接口：

```bash
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed
ros2 topic hz /hdas/camera_wrist_left/color/image_raw/compressed
ros2 topic hz /hdas/camera_wrist_right/color/image_raw/compressed

ros2 topic echo --once /hdas/feedback_arm_left
ros2 topic echo --once /hdas/feedback_gripper_left

ros2 topic info -v /motion_target/target_joint_state_arm_left
ros2 topic info -v /motion_target/target_position_gripper_left
```

每个将被执行的目标话题都必须至少有一个真机订阅者。若夹爪能动但左右臂不动，优先检查机器人端 arm joint tracker；若订阅数为 0，不要继续发布。

## 11. 联调顺序

1. 断开电机执行或使用仿真，只验证 `/api/state`、三相机和反馈。
2. 连接 Policy，执行 `list_tasks` 和 `load_task`，人工检查完整契约。
3. 推理但不执行，检查三路图像方向、state 顺序、actions shape、数值范围和 finite。
4. 使用仿真 profile 验证动作组映射。
5. 真机保持硬件急停可触达，清空工作区，只执行 `mode:"first"`。
6. 验证左右臂、夹爪方向和绝对位置语义。
7. 再执行单个 chunk。
8. 最后启用完整任务循环，并设置 chunk 上限、超时和人工停止入口。

## 12. 安全要求

- 同一时间只允许一个上层控制器向 `/motion_target/*` 发布。启动新设备前停止 Web 页面、示教器或其他 VLA/轨迹发布源，避免目标竞争。
- API token 只表示调用者有写权限，不表示现场安全。
- 所有关节目标必须为有限值；除夹爪 `[0,100]` 外，还应按机器人 URDF/控制器限制增加关节限位和单步变化检查。
- 反馈、相机或 Policy 连接超时时立即停止新目标发布，不得继续执行缓存 chunk。
- 软件 STOP 不替代硬件急停。
- 首次真机联调必须低风险姿态、低速、单臂、单步进行，并由现场人员看护。

## 13. 常见故障

| 现象 | 检查项 |
|---|---|
| `expected r1pro.v2` | 连错端口或仍在运行 v1 Policy Server |
| `Policy Server ... unreachable` | 路由、防火墙、host/port、服务 health |
| `no tasks` | `list_tasks` 返回格式或服务端任务配置 |
| `missing ... cameras` | 三路图像必须全部存在；检查 ROS topic 和 DDS |
| 相机不同步 | ROS header 时间、相机时间源、最大 skew 0.08 s |
| state shape 错误 | 重新读取 `inputs.state.shape/dim_names`；检查历史 state |
| execution disabled | 查看 `execution_disabled_reason`，不要绕过契约校验 |
| action shape 错误 | Policy checkpoint/契约变化；重载任务并重建 buffer |
| target topic has no subscriber | 机器人端 joint tracker 未启动或 DDS 配置错误 |
| 夹爪动、手臂不动 | 优先检查 arm target topic 的订阅者数量 |
| ROS topic 完全不可见 | `ROS_DOMAIN_ID=102`、Fast DDS UDP profile、网卡白名单 |
| 动作整体偏移或突然越界 | 错把绝对动作当 delta，或重复反归一化/角度转换 |
| 完整任务第二轮输入异常 | 未使用 `history_mode=previous_infer`，或 episode 未重置 |
| `Data corruption detected` | Policy Server checkpoint 损坏或拷贝不完整 |

## 14. 上线前需要现场确认的值

下列值属于部署信息，代码本身无法唯一确定，上线前必须由现场负责人确认并写入设备配置：

- 新设备访问 `r1pro_web_server` 的实际 IP；
- 当前 Policy Server 是 `10.30.20.1` 还是 README 示例中的 `10.30.20.73`；
- 新设备是否能同时路由到 `192.168.3.x` 和 `10.30.20.x`；
- 新设备机器人 LAN 网卡的实际 IPv4，用于 Fast DDS `interfaceWhiteList`；
- 当前要运行的 task id 和完成判据；
- 真机 joint tracker 的启动方式与责任人；
- 控制权互斥、硬件急停和现场看护流程。
