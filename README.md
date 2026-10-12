# R1 Pro Web Control Stack

This repository is a source snapshot of the R1 Pro web-control stack and its
local companion packages. The directories are kept together because the web
server starts several of them dynamically and expects sibling source trees at
runtime.

## 在 Docker 中启动 Web（当前工作站）

下面的命令适用于当前已经配置好的 Docker 容器：

```text
容器名称：ros2_humble_gpu
ROS 2：Humble / Python 3.10
ROS_DOMAIN_ID：102
Web 端口：8080
默认控制模式：real（只启动真机 Web）
```

`real` 会连接真机反馈和控制话题。启动前必须确认机器人周围安全、急停可用且
控制链路正确。一键脚本不会启动 MotrixSim，也不建议使用 `both` 同时控制真机和
仿真。需要调试仿真时再使用后面的手动 MotrixSim 启动命令。

### 推荐：一条命令启动真机 Web

当前默认启动入口是宿主机脚本。它会自动启动 Docker、停止旧 Web 和 MotrixSim、
配置 ROS 2/Fast DDS，然后只启动真机 Web：

```bash
cd '/home/pc10/桌面/r1pro-web-control-stack'
./scripts/start_r1pro_web_real.sh
```

看到 `Web UI listening on http://0.0.0.0:8080` 后打开：

```text
http://127.0.0.1:8080
```

脚本以前台方式运行，按 `Ctrl+C` 停止。它不会启动 MotrixSim；网页解锁后的控制
指令会发送给真实机器人。下面第 1～8 节保留了手动启动与排查方法。

### 1. 启动并进入容器

以下命令在宿主机终端运行：

```bash
cd '/home/pc10/桌面/r1pro-web-control-stack'

docker start ros2_humble_gpu
docker exec -it ros2_humble_gpu bash
```

提示 `groups: cannot find name for group ID 992` 时可以忽略。它表示容器内没有该
宿主机组名，不影响 ROS 2、GPU 或 Web 启动。

### 2. 检查 GPU

以下命令开始均在 `ros2_humble_gpu` 容器内运行：

```bash
nvidia-smi
```

如果这里看不到显卡，不要继续启动识别和深度模型，应先检查宿主机 NVIDIA 驱动、
NVIDIA Container Toolkit，以及容器是否使用了 `--gpus all`。

### 3. 加载 ROS 2 环境

```bash
source /opt/ros/humble/setup.bash
source /home/pc10/r1pro_web_colcon/install/setup.bash

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

连接真机相机和 HDAS 话题时，建议继续使用当前工作站的 UDP-only Fast DDS 配置：

```bash
export R1PRO_FASTDDS_PROFILE=/home/pc10/r1pro_web_colcon/install/r1pro_web_control/share/r1pro_web_control/config/fastdds_workstation_udp.xml
export FASTRTPS_DEFAULT_PROFILES_FILE="$R1PRO_FASTDDS_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$R1PRO_FASTDDS_PROFILE"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

该 XML 当前绑定工作站网卡地址 `192.168.3.23`。如果工作站 IP 改变，需要同步修改：

```text
src/r1pro_web_control/config/fastdds_workstation_udp.xml
```

### 4. 停止旧 Web 进程

同一时间只能运行一套 Web Server，否则会发生 8080 端口冲突、重复 ROS 节点或
反馈来源混乱。

```bash
pkill -INT -f '[r]os2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py' || true
sleep 2

ss -lntp | grep ':8080' || true
```

如果最后一条命令没有输出，表示 8080 端口已经释放。

### 5. 启动 Web 和 MotrixSim

下面是当前工作站已经验证过的完整命令。默认关闭 MotrixSim 图形窗口和 RViz，
Web 控制模式为 `sim`：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py \
  default_control_profile:=sim \
  enable_render:=false \
  enable_rviz:=false \
  motrixsim_pythonpath:='/home/pc10/桌面/r1pro-web-control-stack/src/motrixsim-ros2/.venv310/lib/python3.10/site-packages' \
  full_urdf_path:='/home/pc10/桌面/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf' \
  foundation_pose_root:='/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/FoundationPose' \
  foundation_pose_assets_root:='/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects' \
  foundation_pose_python:=/home/pc10/miniconda3/envs/foundationpose/bin/python \
  vision_python:=/home/pc10/miniconda3/envs/foundationpose/bin/python \
  head_depth_python:=/home/pc10/miniconda3/envs/foundationpose/bin/python \
  foundation_pose_ld_library_path:=/home/pc10/miniconda3/envs/foundationpose/lib \
  head_depth_bundle_root:='/home/pc10/桌面/r1pro-web-control-stack/src/stereo_depth_offline_bundle'
```

看到下面的信息表示 Web 已经监听成功：

```text
Web UI listening on http://0.0.0.0:8080
```

保持这个终端运行。停止 Web 时在该终端按 `Ctrl+C`。

### 6. 打开网页

在当前电脑浏览器打开：

```text
http://127.0.0.1:8080
```

同一局域网的其他电脑打开：

```text
http://192.168.3.23:8080
```

如果工作站 IP 已改变，可在宿主机使用以下命令查看当前地址：

```bash
ip -brief -4 address show
```

查看页面和状态不需要 token。执行关节、底盘、夹爪、识别和工作流等操作需要
控制 token。token 保存在容器内：

```bash
cat /root/.ros/r1pro_web_control/token
```

不要把 token 提交到 Git 或发送到不可信设备。

### 7. 启动后检查

另开一个宿主机终端执行：

```bash
docker exec -it ros2_humble_gpu bash

source /opt/ros/humble/setup.bash
source /home/pc10/r1pro_web_colcon/install/setup.bash
export ROS_DOMAIN_ID=102

curl -s http://127.0.0.1:8080/api/state | python3 -m json.tool | head -40
ros2 topic hz /hdas/feedback_arm_left
ros2 topic info -v /joint_states
ros2 node list | grep motrixsim || true
```

使用默认真机脚本时，正确结果应为：

- API 中的 `active_control_profile` 为 `real`；
- `/hdas/feedback_arm_left` 能持续收到真机反馈；
- `/joint_states` 只有真机关节跟踪节点发布；
- `ros2 node list | grep motrixsim` 没有输出，即没有启动 MotrixSim。

如果使用第 5 步手动启动了仿真，再执行 `ros2 topic info -v
/motrixsim/joint_states` 检查仿真话题隔离。

### 8. 常见问题

#### 网页打不开

```bash
ss -lntp | grep ':8080'
docker ps --filter name=ros2_humble_gpu
curl http://127.0.0.1:8080/api/state
```

如果 8080 被旧进程占用，先按第 4 步停止旧进程，再重新启动。

#### 网页显示旧数据或修改没有生效

浏览器按 `Ctrl+F5` 强制刷新。如果刚修改了 ROS/Python 源码，还需要重新构建相关
包并重新启动 Web。

#### ROS 2 看不到真机话题

确认：

```bash
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
ip -brief -4 address show
ros2 topic list | grep '^/hdas/' | head
```

`ROS_DOMAIN_ID` 应为 `102`，工作站网卡应位于机器人使用的 `192.168.3.0/24`
网络中，Fast DDS XML 里的白名单 IP 必须与工作站实际 IP 一致。

#### 相机 FPS 为 0

先检查真机是否确实发布原始图像：

```bash
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed
ros2 topic hz /hdas/camera_head/right_raw/image_raw_color/compressed
```

如果提示 `does not appear to be published yet`，问题在相机或真机 ROS 发布端，
不是浏览器本身。

#### 修改源码后重新构建

```bash
cd /home/pc10/r1pro_web_colcon
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --base-paths \
    '/home/pc10/桌面/r1pro-web-control-stack/src/motrixsim-ros2/src' \
    '/home/pc10/桌面/r1pro-web-control-stack/src/r1pro_web_control' \
  --packages-select \
    motrixsim_ros2_lib \
    motrixsim_ros2_node \
    motrixsim_ros2_bringup \
    motrixsim_ros2_robots \
    r1pro_web_control

source /home/pc10/r1pro_web_colcon/install/setup.bash
```

构建完成后按照第 4、5 步重新启动 Web。

## 躯干 1–4 含义与双钩搬箱标定

Web“关节控制”页中的 torso 1–4 是四个旋转关节，不是四个独立的
升降轴。它们的定义来自当前真机启动时加载的 R1 Pro URDF：

| Web 关节 | URDF 轴 | 范围（约） | 对搬箱的主要影响 |
| --- | --- | --- | --- |
| torso 1 | 下段躯干绕 `+Y` 俯仰 | `-65°～105°` | 下躯干前后弯，改变肩部前伸和高度 |
| torso 2 | 中段躯干绕 `+Y` 俯仰 | `-160°～145°` | 与 torso 1 配合形成腰部曲线 |
| torso 3 | 上段躯干绕 `-Y` 俯仰 | `-105°～90°` | 调整胸口和双肩俯仰，符号方向与 1、2 相反 |
| torso 4 | 上躯干绕 `+Z` 偏航 | `-175°～175°` | 左右转动胸口和双肩，不负责升降 |

前三轴的上躯干合成俯仰可近似理解为：

```text
上躯干俯仰 ≈ torso1 + torso2 - torso3
```

因此，不要把 torso 3 的变化简单理解为“机器人下降了多少”。例如
`[20.31, -43.61, -30.00, 0.01]°` 的合成俯仰约为 `6.70°`，是下段弯曲、
上段补偿的 S 形姿态，不是四轴全零的直立姿态。

### Web 中记录钩箱躯干姿态

1. 先把双臂放到不会与躯干、箱体和桌面碰撞的安全位。
2. 打开“关节控制”，点击“读当前到目标”，只勾选 torso 1–4。
3. 每次建议按 `1°～2°` 小步调整并“执行选中”，同时检查双钩可达、
   肘部余量、左右对称性和全路径净空。
4. 正面居中搬箱时 torso 4 通常保持接近 `0°`；只有斜向作业才需标定偏航。
5. 姿态确定后，点击“保存选中为原子动作”，例如命名为
   `钩箱躯干姿态_v1`。该动作必须且只能包含 torso 1–4，不得混入手臂或夹爪。

搬运时使用保存的四关节姿态作为固定躯干姿态：先慢速复现并等待反馈
到位，再基于该姿态进行双臂全路径 IK。箱子已悬挂时不应再改变躯干姿态。

下列数据仍需独立标定，不能由躯干四关节代替：

- 双臂张开量：由左右挂点间距和左右钩尖 TCP 偏移共同确定，不是一个
  独立的“手臂宽度”参数。
- 挂入高度：由源料盆位姿、挂点局部 Z 和钩尖 TCP 确定。
- 搬运高度：由 `carry_z_m` 单独标定；源位与目标位的高度差由双臂轨迹处理。

交互标定程序会读取 Web 动作库里仅包含 torso 1–4 的动作，并将其目标和
其他钩子几何一起写入标定报告：

```bash
cd '/home/pc10/桌面/r1pro-web-control-stack'
PYTHONPATH=src/r1pro_web_control python3 -m \
  r1pro_web_control.calibration.hook_transport_calibration \
  --api-url http://127.0.0.1:8080
```

向导本身只采集和保存数据，不发送机器人指令。真实搬运前必须先通过
`dry-run`/全路径 IK，再确认双臂安全位、躯干运动净空、急停和挂钩反馈。

### 双夹爪与双钩子搬盆的统一工作流

`dual_box_pick_and_place` 和 `dual_hook_pick_and_place` 共用同一条搬运主链路：
源位预抓取/预挂入 → 抬升 → 搬运 → 目标位下降 → 释放/脱钩 → 退出；差别只在
末端工具和对应标定数据。双夹爪使用开合值和夹取点，双钩子使用钩尖 TCP、挂入/
试抬升轨迹和挂载反馈。

两种 workflow 都支持可选的 Web 保存导航原子动作：

- `nav_to_place_action_name`：带载导航到目标点；
- `nav_to_home_action_name`：放置后返回初始点；
- `nav_stationary_settle_s`：导航到位后的底盘稳定等待。

同时填写两个导航动作时，料盆会在抬升确认后由底盘带载移动，抵达目标后执行放置，
随后返回初始点。两个字段都为空时，保持机械臂 `base_link` 工作空间内的固定点
搬运模式。钩子模式优先使用标定报告中的 `hook_nav_to_place_action_name` /
`hook_nav_to_home_action_name`。

头部深度分拣的 `tray_tool_mode=gripper` 和 `tray_tool_mode=hook` 使用同一套料盆
导航字段：`tray_nav_to_place_action_name`、`tray_nav_to_home_action_name`、
`tray_nav_stationary_settle_s`；只有料盆的抓取/挂载步骤不同。真实运行前仍需完成
dry-run、IK、空载和低高度带载测试。

## Layout

```text
src/
  r1pro_web_control/          ROS 2 web server, browser UI, workflows, and tests
  foundation_pose_ros2/      ROS 2 FoundationPose wrapper
  r1pro_aruco_box_pose/       ArUco pose process
  r1pro_aruco_fine_approach/  ArUco chassis fine-approach process
  stereo_depth_offline_bundle/ Head stereo-depth pipeline and vendored source
  FoundationPose/            FoundationPose source and local object manifests
  vla_openpi/                OpenPI and LeRobot source snapshots
  motrixsim-ros2/            MotrixSim ROS 2 bridge and R1 Pro simulation assets
scripts/                      Workstation launch scripts used by this stack
runtime_data/r1pro_web_control/ Current atomic actions and saved workflows
```

## Build boundary

The ROS packages can be discovered from this repository with both source roots:

```bash
colcon build --symlink-install --base-paths src src/motrixsim-ros2/src
```

The current launch files and workstation scripts retain deployment-specific
defaults for the original R1 Pro workstation. Override their ROS parameters and
environment variables when deploying to a different path, host, DDS domain, or
policy server.

The navigation maps are stored under `src/r1pro_web_control/` as matching
YAML/PGM pairs, with PNG previews. The Web UI's current atomic actions and saved
workflows are stored under `runtime_data/r1pro_web_control/`; see the README in
that directory for validation and restore instructions.

## Deliberately excluded

This repository stores source and the small runtime assets needed for the R1 Pro
configuration. It does not store Python virtual environments, downloaded package
trees, colcon output, logs, debug captures, MCAP datasets, training checkpoints,
or neural-network weights. Use the lock files, model manifests, and download
scripts kept beside each component to restore those artifacts.

Never commit the runtime authentication token normally stored under
`~/.ros/r1pro_web_control/`.

## Licensing

This is a mixed-license source collection. Component license files and notices
remain beside their source. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
before redistributing or using the perception components outside the company.

## 一键执行 Web demo 的 ArUco 离散微调

在 pc10 宿主机执行：

```bash
bash '/home/pc10/桌面/r1pro-web-control-stack/scripts/start_r1pro_aruco_fine.sh'
```

该入口复用 `http://127.0.0.1:8080` 的 Web 后端和 demo 工作流
`aruco_id1_chassis_fine_adjust`，不再启动或启用独立连续微调控制器。
先启动 Web 后端（`scripts/start_r1pro_web_real.sh`）。工作流启动识别、
关闭连续微调，真实执行时切换至手动模式、关闭导航输出并确认底盘停稳，
然后分段运动、等待新位姿并重复调整。任一交接或停稳检查失败都会终止。
默认目标 x=0.60m、y=0、yaw=-90°，到位区间与 Web demo 相同。
命令保持前台显示工作流日志，Ctrl+C 请求停止并发送零速度。
计算首轮动作但不运动：在命令末尾加 `--dry-run`。
从另一个终端停止当前 ArUco 离散微调：在命令末尾加 `--stop`。
Web 状态/API 与 Mac 前端显示同一工作流执行状态。
