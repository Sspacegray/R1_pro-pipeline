# Windows MotrixSim Render Bridge Plan

本文档用于转发给 Windows 主机侧 AI/开发者，目标是在局域网/本机环境中实现：

- WSL 继续运行 ROS 2 Web 控制面板。
- Windows 原生运行 MotrixSim 物理仿真和 GPU 渲染窗口。
- Web 端选择 `仿真` 后，控制 Windows MotrixSim 里的 R1 Pro。

## 前因后果

R1 Pro Web 控制已经接入了 MotrixSim 仿真：WSL 中可以启动
`r1pro_web_control_with_sim.launch.py`，Web 端能选择 `真机`、`仿真`、`同时控制`。
在 `仿真` profile 下，Web 命令会发布到 `/motrixsim/r1pro/target_*`，再由
`r1pro_web_bridge` 转成 `/motrixsim/r1pro/joint_commands` 控制仿真机器人。

问题出在渲染性能。用户机器是 Windows + WSL2，Windows 本机有 NVIDIA RTX 5070，
CUDA 和驱动正常，YOLO 可以使用 GPU。但 WSL2 里 MotrixSim 渲染窗口很卡。排查后发现
这不是 CUDA 问题，而是 WSL2 的 Vulkan 图形渲染没有接到独显：

```text
nvidia-smi: OK, can see RTX 5070
nvcc: OK
vulkaninfo --summary: only llvmpipe CPU device
```

MotrixSim render 走 Vulkan/图形渲染，而 YOLO 走 CUDA compute。CUDA 正常不代表
Vulkan 渲染正常。因此 WSL 中即使能跑仿真，打开 render window 也会落到
`llvmpipe` CPU 软件渲染，表现为窗口严重卡顿。

尝试方向有三种：

1. **继续修 WSL Vulkan**：需要更新 WSLg/Mesa/D3D12 Vulkan 路径，风险是会改系统图形栈，
   且不一定一次成功。
2. **原生 Linux 跑 MotrixSim**：渲染最稳，但需要换运行环境。
3. **Windows 原生 MotrixSim 渲染**：利用 Windows 上已有 NVIDIA 图形驱动，让 WSL 继续
   保持 ROS/Web，不动当前机器人开发环境。

本方案选择第 3 种。核心思路是：不要让 WSL 负责渲染；让 Windows 原生 MotrixSim 成为
唯一仿真物理和渲染源，WSL 通过 TCP 发送控制命令、接收 joint state，再继续把状态发布回
ROS/Web。这样既能保留现有 Web 控制链路，也能获得 Windows 原生 GPU 渲染性能。

本方案不是简单地在 Windows 上“镜像显示”WSL 仿真。镜像方案需要同步完整仿真状态，
容易漂移；推荐让 Windows 端成为 authoritative sim，WSL 端只做代理。

## 背景

当前 WSL2 内 CUDA 正常，YOLO/CUDA compute 可以使用，但 Vulkan 图形渲染没有接到独显。

`vulkaninfo --summary` 只看到：

```text
deviceName = llvmpipe (LLVM 15.0.7, 256 bits)
deviceType = PHYSICAL_DEVICE_TYPE_CPU
driverName = llvmpipe
```

因此 WSL 中直接开启 MotrixSim render 会走 CPU 软件渲染，窗口很卡。Windows 原生 MotrixSim 可以使用 Windows/NVIDIA 图形驱动，适合作为渲染和仿真主进程。

## 当前环境

WSL:

```text
WSL_DISTRO_NAME=Ubuntu-22.04
WSL repo=/home/motrix/galaxea/galaxea_overlay
Web port=8080
Suggested TCP sim port=8765
```

当前网络信息：

```text
WSL IPv4: 192.168.3.5
Windows WLAN IPv4: 192.168.3.5
Default route: 192.168.3.1
```

Windows conda 环境：

```text
robot_sdk = C:\Users\MOTPHYS-OMEN\anaconda3\envs\robot_sdk
```

Windows 端先验证：

```powershell
conda activate robot_sdk
python -c "import motrixsim; print(motrixsim.__file__)"
```

## 关键路径

WSL 源资产：

```text
/home/motrix/galaxea/galaxea_overlay/src/sim/sim/r1_pro_blue_basket_joint_gui_bundle
```

Windows UNC 路径：

```text
\\wsl.localhost\Ubuntu-22.04\home\motrix\galaxea\galaxea_overlay\src\sim\sim\r1_pro_blue_basket_joint_gui_bundle
```

R1 Pro 蓝框模型 XML：

```text
/home/motrix/galaxea/galaxea_overlay/src/motrixsim-ros2/src/motrixsim_ros2_robots/assets/r1pro_blue_basket/r1_pro_blue_basket_grasp_v1_task.xml
```

已安装后的 XML：

```text
/home/motrix/galaxea/galaxea_overlay/install/motrixsim_ros2_robots/share/motrixsim_ros2_robots/assets/r1pro_blue_basket/r1_pro_blue_basket_grasp_v1_task.xml
```

建议复制到 Windows 本地，避免从 UNC 实时读资产导致 IO 卡顿：

```powershell
mkdir C:\motrixsim-r1pro

xcopy "\\wsl.localhost\Ubuntu-22.04\home\motrix\galaxea\galaxea_overlay\src\sim\sim\r1_pro_blue_basket_joint_gui_bundle" `
  "C:\motrixsim-r1pro\r1_pro_blue_basket_joint_gui_bundle" /E /I /Y
```

Windows 本地模型路径建议使用：

```text
C:\motrixsim-r1pro\r1_pro_blue_basket_joint_gui_bundle\motrixsim-python\motrixsim-docs\examples\rm65_dual_arm_transport_v1\rm65_dual_arm_transport\r1_pro_blue_basket_grasp_v1_task.xml
```

## 推荐架构

Windows MotrixSim 作为 authoritative sim，WSL 作为 ROS/Web 代理。

```text
Browser / LAN client
        |
        v
WSL: r1pro_web_control
        |
        | /motrixsim/r1pro/target_*
        v
WSL: r1pro_web_bridge
        |
        | /motrixsim/r1pro/joint_commands
        v
WSL: windows_sim_proxy_node
        |
        | TCP JSON Lines
        v
Windows: r1pro_windows_sim_server.py
        |
        v
Windows MotrixSim physics + GPU render
        |
        | joint_state over TCP
        v
WSL: /joint_states
        |
        v
WSL: r1pro_web_bridge publishes /motrixsim/r1pro/feedback_*
```

不要同时运行 WSL 内的 `motrixsim_ros2_node/sim_node`，否则会有两个仿真源抢 `/joint_states`。

## 需要实现的文件

### Windows 端

建议路径：

```text
C:\motrixsim-r1pro\r1pro_windows_sim_server.py
```

职责：

- 加载 R1 Pro 蓝框 XML。
- 开启 Windows 原生 MotrixSim render。
- 监听 TCP `0.0.0.0:8765`。
- 接收 WSL 发来的 joint command。
- 应用机械臂、夹爪、躯干、底盘命令。
- 以 30-60 Hz 回传完整 joint state。
- 支持 heartbeat 和断线重连。

启动示例：

```powershell
conda activate robot_sdk

python C:\motrixsim-r1pro\r1pro_windows_sim_server.py `
  --model C:\motrixsim-r1pro\r1_pro_blue_basket_joint_gui_bundle\motrixsim-python\motrixsim-docs\examples\rm65_dual_arm_transport_v1\rm65_dual_arm_transport\r1_pro_blue_basket_grasp_v1_task.xml `
  --bind 0.0.0.0 `
  --port 8765 `
  --render
```

Windows 防火墙：

```powershell
netsh advfirewall firewall add rule name="MotrixSim TCP 8765" dir=in action=allow protocol=TCP localport=8765
```

### WSL 端 ROS2 proxy

建议路径：

```text
/home/motrix/galaxea/galaxea_overlay/src/motrixsim-ros2/src/motrixsim_ros2_node/motrixsim_ros2_node/windows_sim_proxy_node.py
```

职责：

- 订阅 `/motrixsim/r1pro/joint_commands`。
- 连接 Windows server。
- 将 `sensor_msgs/msg/JointState` 转 TCP JSON Lines。
- 接收 Windows 回传状态，发布 `/joint_states`。
- TCP 断线后自动重连。
- 连接失败时让 sim feedback stale/offline，不伪造状态。

### 新 launch

建议路径：

```text
/home/motrix/galaxea/galaxea_overlay/src/r1pro_web_control/launch/r1pro_web_control_with_windows_sim.launch.py
```

启动组件：

- `robot_state_publisher`
- `r1pro_web_bridge`
- `windows_sim_proxy_node`
- `r1pro_web_server`

不启动：

- `motrixsim_ros2_node/sim_node`

启动示例：

```bash
cd ~/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
source install/setup.bash

ros2 launch r1pro_web_control r1pro_web_control_with_windows_sim.launch.py \
  default_control_profile:=sim \
  windows_sim_host:=127.0.0.1 \
  windows_sim_port:=8765 \
  enable_rviz:=false
```

如果 `127.0.0.1` 不通，尝试：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_windows_sim.launch.py \
  default_control_profile:=sim \
  windows_sim_host:=192.168.3.5 \
  windows_sim_port:=8765 \
  enable_rviz:=false
```

连接测试：

```bash
nc -vz 127.0.0.1 8765
nc -vz 192.168.3.5 8765
```

## TCP 协议

使用 TCP JSON Lines，一行一个 JSON 对象，UTF-8 编码。

### WSL -> Windows

Joint command:

```json
{"type":"joint_command","seq":1,"names":["left_arm_joint1"],"position":[0.1],"velocity":[],"effort":[]}
```

字段说明：

- `names`: joint names。
- `position`: position commands。没有命令的位置可为空或使用 NaN。
- `velocity`: velocity commands。底盘轮子可使用 velocity。
- `effort`: 预留。

Heartbeat:

```json
{"type":"ping","time":123.456}
```

### Windows -> WSL

Joint state:

```json
{"type":"joint_state","time":123.456,"names":["torso_joint1","left_arm_joint1"],"position":[0.0,0.1],"velocity":[0.0,0.0],"effort":[0.0,0.0]}
```

Heartbeat response:

```json
{"type":"pong","time":123.456}
```

Error:

```json
{"type":"error","message":"unknown joint left_arm_joint9"}
```

## R1 Pro joint names

Web bridge 当前期望的仿真 joint names：

```text
torso_joint1
torso_joint2
torso_joint3
torso_joint4

left_arm_joint1
left_arm_joint2
left_arm_joint3
left_arm_joint4
left_arm_joint5
left_arm_joint6
left_arm_joint7

right_arm_joint1
right_arm_joint2
right_arm_joint3
right_arm_joint4
right_arm_joint5
right_arm_joint6
right_arm_joint7

left_gripper_finger_joint1
right_gripper_finger_joint1

steer_motor_joint1
steer_motor_joint2
steer_motor_joint3

wheel_motor_joint1
wheel_motor_joint2
wheel_motor_joint3
```

控制模式：

```text
torso_joint*: position
left_arm_joint*: position
right_arm_joint*: position
left_gripper_finger_joint1: position
right_gripper_finger_joint1: position
steer_motor_joint*: position
wheel_motor_joint*: velocity
```

## 初始姿态

建议 Windows server 启动后设置如下初始 joint positions，与当前 WSL profile 保持一致：

```python
startup_joint_positions = {
    "torso_joint1": 0.5658,
    "torso_joint2": -0.7469,
    "torso_joint3": -0.8308,
    "torso_joint4": -0.0152,
    "left_arm_joint1": -0.059,
    "left_arm_joint2": 0.123,
    "left_arm_joint3": -0.115,
    "left_arm_joint4": -1.563,
    "left_arm_joint5": 0.082,
    "left_arm_joint6": 0.009,
    "left_arm_joint7": 0.027,
    "right_arm_joint1": -0.147,
    "right_arm_joint2": -0.018,
    "right_arm_joint3": -0.051,
    "right_arm_joint4": -1.476,
    "right_arm_joint5": 0.008,
    "right_arm_joint6": 0.009,
    "right_arm_joint7": 0.154,
    "left_gripper_finger_joint1": 0.045,
    "right_gripper_finger_joint1": 0.045,
}
```

注意：MJCF 中 gripper 可能还有 passive/mimic finger，例如 `finger_joint2`，Windows server 只需要控制 active finger，状态回传可同时包含 passive finger，或由 WSL bridge 继续做 alias。

## WSL 清理旧进程

启动新 Windows-sim launch 前，建议先清掉旧的 WSL MotrixSim/Web 进程：

```bash
pkill -f "ros2 launch r1pro_web_control"
pkill -f "r1pro_web_server"
pkill -f "r1pro_web_bridge"
pkill -f "motrixsim_ros2_node/lib/motrixsim_ros2_node/sim_node"

sleep 2

pgrep -af "r1pro_web_server|r1pro_web_bridge|sim_node|ros2 launch r1pro_web_control"
```

如果还有残留：

```bash
pkill -9 -f "ros2 launch r1pro_web_control"
pkill -9 -f "r1pro_web_server"
pkill -9 -f "r1pro_web_bridge"
pkill -9 -f "motrixsim_ros2_node/lib/motrixsim_ros2_node/sim_node"
```

## 局域网访问 Web

本机浏览器：

```text
http://127.0.0.1:8080
```

如果局域网其他设备要访问 Windows 主机上的 WSL Web，可以在 Windows 管理员 PowerShell 中转发 8080 到 WSL IP。

查看 WSL IP：

```powershell
wsl hostname -I
```

添加端口转发，假设 WSL IP 为 `192.168.3.5`：

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8080 connectaddress=192.168.3.5 connectport=8080
netsh advfirewall firewall add rule name="R1Pro Web 8080" dir=in action=allow protocol=TCP localport=8080
```

局域网访问：

```text
http://<Windows主机IP>:8080
```

## 验收标准

1. Windows 端 MotrixSim 渲染窗口流畅，不使用 WSL llvmpipe。

2. Windows server 打印 TCP client connected。

3. WSL 可看到 `/joint_states`：

```bash
ros2 topic echo /joint_states --once
```

4. WSL Web bridge 可发布仿真反馈：

```bash
ros2 topic echo /motrixsim/r1pro/feedback_arm_right --once
ros2 topic echo /motrixsim/r1pro/feedback_gripper_right --once
```

5. Web 页面打开：

```text
http://127.0.0.1:8080
```

控制目标选择 `仿真`，右臂、夹爪、底盘动作可以驱动 Windows MotrixSim 窗口中的机器人。

6. 关闭 Windows server 后，WSL proxy 自动重连，Web 仿真反馈变 stale/offline，不应继续显示假在线。

## 实现顺序建议

1. Windows server 单机加载 XML 并开启 render，确认不卡。
2. Windows server 暴露 TCP，先只回传 joint state。
3. WSL proxy 接收 joint state 并发布 `/joint_states`。
4. 接入 `/motrixsim/r1pro/joint_commands`，打通手臂和夹爪控制。
5. 接入底盘 swerve 轮速/转向控制。
6. 新增 launch 和 README 使用说明。
7. 做断线、重连、心跳、错误日志。

## 注意事项

- 不要在 WSL 同时启动 `sim_node`。
- Windows server 应该是唯一的仿真物理源。
- TCP 命令需要容忍 NaN/空字段。
- Windows 端资产建议复制到本地磁盘，不建议长期从 `\\wsl.localhost` 实时读。
- Web 的 `sim` profile 继续使用现有 `/motrixsim/r1pro/target_*` 和 `/motrixsim/r1pro/feedback_*`，前端不需要改交互。
