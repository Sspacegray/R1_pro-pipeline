# r1pro_web_control

R1 Pro 的 ROS 2 Web 控制面板，用于替代旧 Tk GUI。原
`arm_joint_gui`、`foundation_pose_ros2` 源码不参与本项目改造。

## 地图裁剪与坐标系旋转

包内提供了独立的浏览器地图编辑器，不需要启动 ROS 节点。它可以拖框裁剪占据地图、
预览并修改 `origin[2]` 的 yaw，并按原地图分辨率及原始 yaw 自动补偿裁剪后的原点。
原始地图不会被覆盖；默认同时导出新的 PGM、预览 PNG 和 YAML。

在源码目录直接启动：

```bash
cd ~/galaxea/galaxea_overlay/src/r1pro_web_control
python3 -m r1pro_web_control.map_editor mx_map.yaml
```

浏览器会自动打开 `http://127.0.0.1:8091/`。停止程序按 `Ctrl+C`。如需从局域网
其他电脑访问或指定输出目录：

```bash
python3 -m r1pro_web_control.map_editor mx_map.yaml \
  --host 0.0.0.0 --port 8091 --no-browser \
  --output-dir ~/maps
```

构建并 source 工作空间后也可使用 ROS 入口：

```bash
ros2 run r1pro_web_control r1pro_map_editor \
  ~/galaxea/galaxea_overlay/src/r1pro_web_control/mx_map.yaml
```

编辑器的“旋转坐标系”只修改 YAML yaw，不会重采样栅格图。角度使用度数显示，
写入 YAML 时自动转换为 ROS 使用的弧度。

## 快速开始

第一次使用或代码更新后，先构建 Web 和仿真相关包：

```bash
cd ~/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
colcon build --packages-up-to \
  motrixsim_ros2_node \
  motrixsim_ros2_robots \
  motrixsim_ros2_bringup \
  r1pro_web_control
source install/setup.bash
```

```bash
pkill -TERM -f r1pro_web_server
sleep 2

# 如果旧进程仍占用 8080，再强制结束
if ss -lntp | grep -q ':8080 '; then
  pkill -KILL -f r1pro_web_server
fi

source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash
source /home/motrix/galaxea/galaxea_overlay/install/setup.bash

# The robot can send more than 50 MiB/s of fragmented UDP camera/point-cloud
# traffic. Ubuntu 22.04 defaults are too small and cause RcvbufErrors and
# ReasmFails, which appear in the Web UI as an intermittent or 0-FPS camera.
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=16777216
sudo sysctl -w net.core.netdev_max_backlog=10000
sudo sysctl -w net.ipv4.ipfrag_high_thresh=268435456
sudo sysctl -w net.ipv4.ipfrag_low_thresh=201326592
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.core.netdev_budget=600
sudo sysctl -w net.core.netdev_budget_usecs=8000

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DDS_INTERFACE_IP=192.168.3.5
export R1PRO_FASTDDS_PROFILE=/home/motrix/galaxea/galaxea_overlay/install/r1pro_web_control/share/r1pro_web_control/config/fastdds_workstation_udp.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=$R1PRO_FASTDDS_PROFILE
export FASTDDS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

ros2 run r1pro_web_control r1pro_web_server \
  --ros-args \
  -r __node:=r1pro_web_server \
  -p vla_policy_host:=10.30.20.73 \
  -p vla_policy_port:=18081 \
  -p head_depth_python:=/home/motrix/galaxea/FoundationPose/.venv/bin/python3 \
  -p head_depth_bundle_root:=/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle \
  -p head_depth_output_timeout_s:=2.0
```

如果本机还没有 Python 3.10 版 MotrixSim runtime，先创建一次专用环境：

```bash
cd ~/galaxea/galaxea_overlay/src/sim/sim
uv venv .venv310 --python python3.10
uv pip install --python .venv310/bin/python --index-url http://pypi.mp/simple motrixsim numpy
```

验证：

```bash
PYTHONPATH=~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages \
  python3 -c "import motrixsim; print(motrixsim.__file__)"
```

然后按需要选择一种启动方式。

## 启动真机 Web

只连接真机、不启动 MotrixSim 时用这个入口。默认控制目标是“真机”，控制话题是
`/motion_target/*`，反馈话题是 `/hdas/feedback_*`。

推荐使用包含头部在线深度配置的 domain 102 一键脚本：

```bash
/home/motrix/start_r1pro_web_head_depth_domain102.sh
```

脚本会先以 `TERM`、再以 `KILL` 清理旧的 Web、头部深度、YOLO、
FoundationPose 和 ArUco 感知进程，确认 `8080` 未被其他程序占用，然后 source
ROS/工作空间，设置 `ROS_DOMAIN_ID=102`、`rmw_fastrtps_cpp`、
`192.168.3.5` 网卡和本包的 UDP-only Fast DDS profile，最后以前台方式启动
`r1pro_web_server`。按 `Ctrl+C` 停止。

该脚本只配置在线深度使用的 Python、`stereo_depth_offline_bundle` 和 2 秒输出超时，
不会自动占用 GPU 启动深度模型。打开网页后仍需在“相机”页打开独立安全锁，再打开
“头部相机深度处理”开关并等待状态变为“运行”。首次使用前需已按“快速开始”完成
构建，否则脚本 source 的 install 空间可能仍是旧版本。

额外参数会原样追加到 `ros2 run ... --ros-args`，例如临时把深度输出超时改为 3 秒：

```bash
/home/motrix/start_r1pro_web_head_depth_domain102.sh \
  -p head_depth_output_timeout_s:=3.0
```

当前机器人恢复出厂后的通信配置使用 `ROS_DOMAIN_ID=102`、`rmw_fastrtps_cpp`
和 `192.168.3.5` 这块外部电脑网卡。不要直接使用普通 `ros2 launch
r1pro_web_control r1pro_web_control.launch.py`，否则 Web 进程可能启动成功但收不到真机 DDS 话题。
机器人使用 UDP-only Fast DDS profile，因此 Web 必须使用本包安装的
`fastdds_workstation_udp.xml`。不要使用 startup_config 中的 `LARGE_DATA` profile；该配置
会让导航节点可见，但 HDAS 反馈在跨机器 ROS 图中消失。

# 启动web+仿真
cd /home/motrix/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash
colcon build --packages-select r1pro_web_control
source install/setup.bash

ros2 launch r1pro_web_control r1pro_web_control_with_windows_sim.launch.py   default_control_profile:=sim   windows_sim_host:=127.0.0.1   windows_sim_port:=8765   enable_rviz:=false

### 真机控制链路检查

Web 只负责发布 `/motion_target/*` 目标；真机端还需要对应控制节点订阅这些话题。
如果夹爪可以动，但左右臂不动，通常是 R1 PRO 的 joint tracker 没有在机器人端启动。

在 Web 所在机器上可以检查：

```bash
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node

topics = [
    "/motion_target/target_joint_state_arm_left",
    "/motion_target/target_joint_state_arm_right",
    "/motion_target/target_position_gripper_left",
    "/motion_target/target_position_gripper_right",
]

class N(Node):
    def __init__(self):
        super().__init__("r1pro_target_subscriber_check")
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        for topic in topics:
            subs = self.get_subscriptions_info_by_topic(topic)
            pubs = self.get_publishers_info_by_topic(topic)
            print(f"{topic}: publishers={len(pubs)} subscriptions={len(subs)}")

rclpy.init()
n = N()
n.destroy_node()
rclpy.shutdown()
PY
```

左右臂正常应至少各有 1 个订阅者。若
`/motion_target/target_joint_state_arm_left/right` 的 `subscriptions=0`，
需要在机器人端启动 `mobiman` 的 R1 PRO joint tracker，例如对应的
`r1_pro_jointTrackerdemo_launch.py`。这类节点是 aarch64 实机二进制，不能在
x86 上位机直接运行。

## 只启动仿真

只想打开 MotrixSim R1 Pro 蓝框场景、不启动 Web 控制面板时用这个入口。这个模式
只启动仿真本体、`robot_state_publisher` 和 MotrixSim ROS2 基础桥，不启动
`r1pro_web_server`，网页也不会打开。

启动前同样可以确认 Python 3.10 环境能导入 MotrixSim runtime：

```bash
PYTHONPATH=~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages \
  python3 -c "import motrixsim; print(motrixsim.__file__)"
```

ROS Humble 这里使用 Python 3.10，所以 MotrixSim runtime 也必须是 Python 3.10
可导入的版本。只包含 `motrixsim.cpython-312-*.so` 的 Python 3.12 venv 不能给
这个 ROS 节点使用。

然后启动仿真：

```bash
cd ~/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
source install/setup.bash
ros2 launch motrixsim_ros2_bringup bringup_robot.launch.py \
  robot:=r1pro_blue_basket \
  profile_package:=motrixsim_ros2_robots \
  enable_render:=true \
  enable_rviz:=false \
  motrixsim_pythonpath:=~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages \
  enable_trajectory_bridge:=false \
  enable_follow_joint_trajectory_adapter:=false
```

无显示环境或只想后台跑物理仿真时，把 `enable_render:=false`。

只启动仿真时，直接控制入口是：

```text
/motrixsim/r1pro/joint_commands
```

消息类型是 `sensor_msgs/msg/JointState`。机械臂、躯干、转向和夹爪用
`position`，底盘轮子用 `velocity`；不控制的字段可以填 `NaN`。Web 里的
`/motrixsim/r1pro/target_*` 话题需要 `r1pro_web_bridge`，只启动纯仿真时默认
不会有这个桥。

## 启动仿真 + Web

这个入口会同时启动 MotrixSim R1 Pro 蓝框场景、R1 Pro Web 仿真桥接节点和 Web
控制面板。默认控制目标是“仿真”。

组合 launch 默认使用
`~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages`。
启动前可以确认 Python 3.10 环境能导入 MotrixSim runtime：

```bash
PYTHONPATH=~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages \
  python3 -c "import motrixsim; print(motrixsim.__file__)"
```

ROS Humble 这里使用 Python 3.10，所以 MotrixSim runtime 也必须是 Python 3.10
可导入的版本。只包含 `motrixsim.cpython-312-*.so` 的 Python 3.12 venv 不能给
这个 ROS 节点使用。

如果这里报 `ModuleNotFoundError: No module named 'motrixsim'`，先回到“快速开始”
创建 `.venv310`。之后启动组合 launch：

```bash
cd ~/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
source install/setup.bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py
```

可选参数：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py \
  default_control_profile:=sim \
  enable_render:=true \
  enable_rviz:=false
```

如果要临时使用其他 MotrixSim Python 3.10 环境，可以覆盖：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py \
  motrixsim_pythonpath:=/path/to/python3.10/site-packages
```

## 打开网页

启动后浏览器打开：

```text
http://127.0.0.1:8080
```

同一局域网设备打开：

```text
http://<robot-ip>:8080
```

查看页面、状态、图像流不需要 token；关节、底盘、夹爪、EE、抓取、
YOLO、FoundationPose、Action/Sequence 写入和执行都需要 token。token
会打印在节点日志里，也会保存到：

```text
~/.ros/r1pro_web_control/token
```

## 相机与识别页面

原“相机与识别”已拆成两个页面，并使用互不继承的安全锁：

- “相机”页负责头部深度开关、四路可独立选择的视频、拍照和点击取 3D 点。
- “识别”页负责 YOLO、锁存分割、FoundationPose、ArUco、识别结果和状态日志。

离开“相机”页时，浏览器会断开四路原始 MJPEG；离开“识别”页时会断开 YOLO、
FoundationPose 和 ArUco 派生 MJPEG。返回对应页面后，仍处于订阅/运行状态的流会
自动重连。这只影响浏览器图像连接，不会因为切页停止后端识别进程。

选择头部左相机做 3D 取点或识别前，先进入“相机”页，打开本页安全锁，再打开
“头部相机深度处理”。只有状态变为“运行”、后端
`head_depth.processor.ready=true` 后，`head_left_rgb`、aligned depth 和
CameraInfo 才作为一组有效输入。此时“点击取点”会同时显示图像坐标、深度、相机
坐标和 `base_link` 坐标；未 ready 时头部取点、YOLO 和 FoundationPose 会被门禁。
腕部相机仍使用各自原有 RGB-D 输入，不依赖这个开关。

### 严格 YOLO → 锁存 → FoundationPose 流程

识别页顶部会显示三个步骤、当前门禁原因、后端错误和状态日志。正常顺序是：

1. 打开“识别”页安全锁，选择相机、模型、提示词和阈值，启动 YOLO。使用
   `head_left` 时必须先让头部深度进入 ready。
2. 等待检测结果，点击“锁存最佳分割”。锁存结果必须是当前相机、config 和 worker
   代次的 verified/frozen mask，并带有效来源时间戳与分割面积；手填文件路径不能
   绕过此步骤。“Initial mask”是只读状态，只显示当前
   `latched_yolo.mask_file`。页面不提供独立 BBox segmentation 模式。
3. FoundationPose 相机必须与锁存目标相同，锁存类别也必须包含在 FoundationPose
   `Classes` 中。满足这些条件且 YOLO 仍在运行时，才能启动 FoundationPose。
4. FoundationPose 在 `waiting/registering` 阶段依赖 YOLO 持续发布同时间戳的
   color/mask。出现首个 `object_pose`，或 debug 进入 `pose_ready/track` 之前，不要
   停止 YOLO；页面会禁用“停止 YOLO”并显示原因。
5. 首次注册完成后可以停止 YOLO，FoundationPose 会继续独立跟踪。此后可读取
   `base_link` 下的 6D 位姿。若在首次注册完成前意外停止 YOLO，应先停止
   FoundationPose，再重新启动 YOLO、重新检测和锁存。

切换 YOLO 相机或重新发布模型、提示词等配置会产生新的 config/worker 代次，旧检测
和旧锁存不能用于新的 FoundationPose 注册，必须重新锁存。浏览器刷新时，如果
YOLO 或 FoundationPose 已在后端运行，表单会从 `state.vision.config` 和
`state.foundation` 的活动配置恢复真实相机、模型、提示词、阈值、Mesh 和 Classes。
同一进程代次内的 SSE 状态刷新不会反复覆盖用户尚未提交的表单编辑。

### 娃哈哈瓶测试预设

识别流程面板的“填入娃哈哈瓶测试预设”只填写参数，不会启动任何进程。它会设置：

- YOLO 和 FoundationPose 相机：`head_left`；
- YOLO 提示词和 FoundationPose Classes：`bottle`；
- YOLO confidence：`0.4`；
- segmentation：`external_mask`；
- Mesh：
  `/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects/drink_Wahaha/assets/V_wahaha_mi.obj`。

使用预设后仍需先确认头部深度 ready，再依次执行“启动 YOLO → 锁存最佳分割 →
启动 FoundationPose”，并在首次注册完成前保持 YOLO 运行。

## 控制目标

Web 仪表盘上方有“控制目标”选择：

- `真机`：命令发到 `/motion_target/*`，反馈读 `/hdas/feedback_*`。
- `仿真`：命令发到 `/motrixsim/r1pro/target_*`，反馈读
  `/motrixsim/r1pro/feedback_*`。
- `同时控制`：同一条命令同时发给真机和仿真。

真机底盘的 Web 目标话题是 `/motion_target/target_speed_chassis`
(`geometry_msgs/msg/TwistStamped`)；Web 还会发布
`/motion_target/chassis_acc_limit`。实际下发到底盘电机的是
`mobiman` 的 `chassis_control_node`，它订阅
`/motion_target/target_speed_chassis`，再发布
`/motion_control/control_chassis`。如果 ROS graph 中没有
`chassis_control_node` 订阅 `/motion_target/target_speed_chassis`，
只启动 Web 不会让真机底盘动；此时需要启动：

```bash
ros2 launch mobiman r1_pro_chassis_control_launch.py
```

切换控制目标时，后端会停止当前关节/底盘流式控制并发送底盘零速度；前端会清空
各页面安全锁，需要重新打开安全锁后才能继续下发动作。

`同时控制` 模式下，反馈、IK、等待到位判断只使用真机反馈。真机反馈缺失或超时
会按缺失/超时报错，不会用仿真反馈兜底。

## 基本操作流程

1. 打开网页，确认仪表盘右上角连接状态正常。
2. 在“控制目标”里选择 `真机`、`仿真` 或 `同时控制`。
3. 进入“关节控制”“末端 6D”或“底盘控制”页面。
4. 打开当前页面的安全锁。
5. 下发关节、夹爪、末端或底盘动作。
6. 停止测试时点网页里的 `STOP`。

夹爪在 Web 里使用 `open 0-100`。真机仍按原有话题下发；仿真桥会把
`0-100` 转成单侧 finger 行程 `0-0.05 m`，反馈再转回 `0-100`。

### 躯干 1–4 与双钩搬箱姿态

Web 里 torso 1–4 都是旋转关节，不是独立的升降轴：

| Web 关节 | 机械含义 | URDF 范围（约） |
| --- | --- | --- |
| torso 1 | 下段躯干绕 `+Y` 俯仰 | `-65°～105°` |
| torso 2 | 中段躯干绕 `+Y` 俯仰 | `-160°～145°` |
| torso 3 | 上段躯干绕 `-Y` 俯仰，符号方向与 1、2 相反 | `-105°～90°` |
| torso 4 | 上躯干绕 `+Z` 偏航，改变胸口和双肩朝向 | `-175°～175°` |

上躯干合成俯仰可近似理解为
`torso1 + torso2 - torso3`。双钩搬箱时，应在 Web 里只勾选 torso 1–4，
以 `1°～2°` 小步调整并检查双钩可达、肘部余量和全路径净空。姿态确定后，
用“保存选中为原子动作”记录一个仅含四轴的动作，例如
`钩箱躯干姿态_v1`。正面居中搬箱时 torso 4 通常保持接近 `0°`。

搬运 pipeline 先慢速复现该原子动作、等待躯干反馈到位，然后基于实际躯干姿态
做双臂全路径 IK。箱子悬挂期间保持躯干不变。双臂张开量由左右挂点和钩尖
TCP 共同确定；挂入高度由料盆位姿、挂点 Z 和 TCP 确定；搬运安全高度由
`carry_z_m` 单独标定。这些数据都不能用单个 torso 关节值代替。

完整标定步骤、交互命令和安全门禁见仓库根目录
[`README.md`](../../README.md#躯干-14-含义与双钩搬箱标定)。

## 重启和端口占用

如果 `8080` 被旧 Web 服务占用，日志会出现：

```text
OSError: [Errno 98] Address already in use
```

可以先停止旧实例，再重新启动：

```bash
cd ~/galaxea/galaxea_overlay

PIDS="$(pgrep -f '[r]os2 launch r1pro_web_control|[r]os2 run r1pro_web_control|[r]1pro_web_server|[r]1pro_web_bridge|[r]1pro_web_left_wrist_camera_tf_bridge|[r]1pro_web_right_wrist_camera_tf_bridge|[s]tatic_transform_publisher.*r1pro_web' || true)"
if [ -n "$PIDS" ]; then
  kill -TERM $PIDS
  sleep 2
fi

PIDS="$(pgrep -f '[r]os2 launch r1pro_web_control|[r]os2 run r1pro_web_control|[r]1pro_web_server|[r]1pro_web_bridge|[r]1pro_web_left_wrist_camera_tf_bridge|[r]1pro_web_right_wrist_camera_tf_bridge|[s]tatic_transform_publisher.*r1pro_web' || true)"
if [ -n "$PIDS" ]; then
  kill -KILL $PIDS
  sleep 1
fi
```

`start_r1pro_web_head_depth_domain102.sh` 会固定检查 `8080`，发现占用时主动拒绝
启动，避免误杀不属于本项目的服务。如果确实要保留旧实例并临时换端口，不要使用
一键脚本；在已经按“启动真机 Web”设置好 domain 102 和 Fast DDS 环境的 shell 中
直接启动第二个节点，例如：

```bash
ros2 run r1pro_web_control r1pro_web_server \
  --ros-args \
  -r __node:=r1pro_web_server_8081 \
  -p port:=8081 \
  -p head_depth_python:=/home/motrix/galaxea/FoundationPose/.venv/bin/python3 \
  -p head_depth_bundle_root:=/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle
```

重新编译并启动仿真组合入口：

```bash
cd ~/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source ~/galaxea/install/setup.bash
colcon build --packages-up-to \
  motrixsim_ros2_node \
  motrixsim_ros2_robots \
  motrixsim_ros2_bringup \
  r1pro_web_control
source install/setup.bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py
```

## 验证仿真接入

组合 launch 正常启动后，可以用下面命令确认仿真和 Web 桥接都在工作：

```bash
ros2 topic list | grep -E '/motrixsim/r1pro|/joint_states'
ros2 topic echo --once /motrixsim/r1pro/feedback_arm_right
ros2 topic echo --once /joint_states
```

从 Web 选择“仿真”后，下发右臂、夹爪或底盘动作。正常现象是：

- `/motrixsim/r1pro/target_*` 会收到 Web 目标。
- `/motrixsim/r1pro/joint_commands` 会收到带 joint name 的仿真命令。
- `/joint_states` 会更新。
- Web 里的 `/motrixsim/r1pro/feedback_*` 反馈会同步变化。

从 Web 选择“真机”后，同样按钮会改为发布到 `/motion_target/*`。选择“同时控制”
后，两个目标都会收到命令，但 Web 显示和 IK 等待只看真机反馈。

## 常见问题

### No module named motrixsim

现象：

```text
ModuleNotFoundError: No module named 'motrixsim'
```

处理：先让当前 shell 能导入 `motrixsim`，再启动组合 launch。

```bash
python3 -c "import motrixsim; print(motrixsim.__file__)"
```

如果这条命令失败，创建或指定 Python 3.10 版 MotrixSim 环境：

```bash
cd ~/galaxea/galaxea_overlay/src/sim/sim
uv venv .venv310 --python python3.10
uv pip install --python .venv310/bin/python --index-url http://pypi.mp/simple motrixsim numpy
```

注意：ROS Humble 这里使用 Python 3.10，路径里必须有 Python 3.10 可导入的
MotrixSim 包。只有 `motrixsim.cpython-312-*.so` 的 Python 3.12 包不能使用。

然后启动时直接传，或使用组合 launch 的默认值：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py \
  motrixsim_pythonpath:=~/galaxea/galaxea_overlay/src/sim/sim/.venv310/lib/python3.10/site-packages
```

### 8080 端口被占用

现象：

```text
OSError: [Errno 98] Address already in use
```

处理：执行“重启和端口占用”里的停止命令，或者换端口：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py port:=8081
```

### KDL root link inertia warning

如果看到：

```text
The root link base_link has an inertia specified in the URDF
```

这是 `robot_state_publisher`/KDL 对 root link inertia 的警告，通常不影响 Web
控制和 MotrixSim 启动。

### 页面能打开但不能下发动作

检查三件事：

- 是否填了 token。token 在日志里打印，也在 `~/.ros/r1pro_web_control/token`。
- 当前页面安全锁是否打开。
- 控制目标是否选对；“真机”需要 `/hdas/feedback_*`，“仿真”需要
  `/motrixsim/r1pro/feedback_*`。

## Defaults

- 用户层目标坐标系：`base_link`。
- 机器人反馈话题：`/hdas/feedback_*`。
- 机器人控制话题：`/motion_target/*`。
- Web/感知自有 namespace：`/r1pro_web/*`。
- 头部深度处理默认关闭且不持久化。页面开关启动独立 Fast-FoundationStereo
  子进程；Web 重启、全局 STOP 或退出都会停止它。
- 处理器订阅左右 1920×1080 raw JPEG，在 5 ms 内配对并 latest-only 推理；输出
  `/r1pro_web/camera_head/left/rectified/image_raw/compressed`、
  `/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw`（`32FC1` 米，NaN
  无效）和 `/r1pro_web/camera_head/left/rectified/camera_info`。
- 校正图 frame 为 `camera_head_left_rectified_optical`；处理器发布
  `camera_head_left_link -> camera_head_left_rectified_optical` 静态 TF，Web 再沿
  现有 TF 链输出 `base_link` 坐标。
- 头部双目点云输入：`/hdas/camera_head/points2`。
- 电量输入：`/hdas/bms`，`hdas_msg/msg/Bms`，字段
  `voltage/current/capital`。如果运行环境没有 source 到 `hdas_msg` 所在
  underlay，或本机没有可加载的 `hdas_msg` Python type support，Web 会降级用
  `ros2 topic echo /hdas/bms` 解析同样字段；页面会显示电量来源
  `rclpy` 或 `ros2_topic_echo` 以及具体原因。
- 夹爪真实控制单位仍是 `open 0-100`；Web 同时按
  `gripper_max_open_m` 显示米值，默认 `0.10 m`。
- 控制目标 profile：`real` 使用 `/hdas/feedback_*` 和 `/motion_target/*`；
  `sim` 使用 `/motrixsim/r1pro/feedback_*` 和
  `/motrixsim/r1pro/target_*`；`both` 同时下发到二者，反馈只取 `real`。
- 数据目录：`~/.ros/r1pro_web_control/`。
- 旧动作数据从 `~/.ros/arm_joint_gui/` 复制导入，新项目不共享、不回写旧文件。
- FoundationPose 默认使用 `/home/motrix/galaxea/galaxea_overlay/src/FoundationPose/FoundationPose` 和
  `/home/motrix/galaxea/FoundationPose/.venv/bin/python3`。
- YOLO vision worker 默认也使用
  `/home/motrix/galaxea/FoundationPose/.venv/bin/python3`，复用其中的
  `ultralytics` 环境。
- 在线深度也使用这个 Python 3.10 环境；首次开启会通过 bundle 内脚本把锁定的
  轻量依赖安装到 `stereo_depth/.online_deps`，不会修改其 Python 3.11 venv。

## Launch Parameters

常用覆盖示例：

```bash
ros2 launch r1pro_web_control r1pro_web_control.launch.py \
  host:=0.0.0.0 \
  port:=8080 \
  base_frame:=base_link \
  web_namespace:=/r1pro_web \
  data_dir:=~/.ros/r1pro_web_control \
  token_file:=~/.ros/r1pro_web_control/token \
  head_rgb_topic:=/r1pro_web/camera_head/left/rectified/image_raw/compressed \
  head_left_raw_rgb_topic:=/hdas/camera_head/left_raw/image_raw_color/compressed \
  head_depth_topic:=/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw \
  head_left_camera_info_topic:=/r1pro_web/camera_head/left/rectified/camera_info \
  head_left_tf_frame:=camera_head_left_rectified_optical \
  head_depth_python:=/home/motrix/galaxea/FoundationPose/.venv/bin/python3 \
  head_depth_bundle_root:=/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle \
  head_depth_output_timeout_s:=2.0 \
  head_pointcloud_topic:=/hdas/camera_head/points2 \
  battery_topic:=/hdas/bms \
  gripper_max_open_m:=0.10
```

组合仿真 launch 常用覆盖示例：

```bash
ros2 launch r1pro_web_control r1pro_web_control_with_sim.launch.py \
  default_control_profile:=sim \
  enable_render:=true \
  enable_rviz:=false
```

完整 URDF 默认路径：

```text
/home/motrix/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf
```

## API

- `GET /`：Web UI。
- `GET /api/state`：完整状态快照。
- `GET /api/events`：SSE 状态流。
- `GET /stream/{key}`：MJPEG 图像流。常用 key 包括
  `head_left_rgb`、`head_left_raw_rgb`、`head_right_rgb`、`head_depth`、`wrist_left_rgb`、
  `wrist_left_depth`、`wrist_right_rgb`、`wrist_right_depth`、`yolo`、
  `foundation`、`aruco`。
- `GET /snapshot/{key}`：读取某个图像流的当前 JPEG 帧。
- `GET /api/pointcloud/head`：头部点云状态和抽样预览点。
- `GET /api/navigation/state`：读取后端唯一的导航控制权状态，包括
  `MANUAL`、`SWITCHING`、`NAVIGATION`、`FAULT`、ROS 输出状态、`/navigation/status`
  综合状态、原地旋转进度和控件使能条件。
- `GET /api/navigation/map`：导航页按需读取地图、costmap 和 plan 图层。
- `POST /api/control/*`：控制接口，需要 token。
- `POST /api/control_profile/select`：切换控制目标，需要 token，payload
  为 `{"profile": "real"}`、`{"profile": "sim"}` 或
  `{"profile": "both"}`。
- `POST /api/control/joint_targets`：单关节或多关节目标下发，需要 token。
- `POST /api/control/ee_step`：末端 6D 小步点动，需要 token。
- `POST /api/vision/*`：相机/YOLO/点击/latch，需要 token。
- `POST /api/vision/set_head_depth`：按需启停头部在线深度，payload 为
  `{"enabled":true}` 或 `{"enabled":false}`。`GET /api/state` 的
  `head_depth.processor` 返回 requested/running/ready/phase/FPS/推理延迟/丢帧、
  错误、退出码和日志尾部。只有 fresh depth 与 TF 同时可用时 `ready=true`。
- `POST /api/vision/save_snapshot`：保存某个视频流当前帧到 Web 服务所在机器，
  需要 token，payload 形如
  `{"stream_key":"head_left_rgb","path":"~/Pictures/r1pro_web_control"}`。
  `path` 可以是目录，也可以是 `.jpg`、`.jpeg` 或 `.png` 文件名；目录会自动生成
  带时间戳的 `.jpg` 文件。
- `POST /api/navigation/mode`：安全切换底盘控制权，需要 token，payload 为
  `{"mode":"manual"}` 或 `{"mode":"navigation"}`。后端会串行调用
  `/cmd_vel_to_twist_stamped/set_nav_output_enabled`，并等待
  `/cmd_vel_to_twist_stamped/nav_output_state` 确认。
- `POST /api/navigation/goal`：发布导航目标到 `/goal_pose`，需要 token，payload 可为
  `{"x_m":1.0,"y_m":2.0,"yaw":1.57}`（弧度）或使用兼容字段 `yaw_deg`。仅在导航
  控制权有效且 `/navigation/status.navigation_ready=true` 时允许发布，消息时间戳固定
  为零。定位状态不再根据 AMCL 消息年龄判断，避免机器人静止时误报定位失效。
- `POST /api/navigation/cancel`：取消当前 Nav2 导航目标，需要 token。普通取消不会从
  Web 手动速度发布器发送零速度，避免与导航输出冲突。
- `POST /api/navigation/stop`：结束网页按住式手动底盘控制并只发送一帧零速度；非手动
  模式且没有活动手动命令时不会发布。
- `POST /api/navigation/emergency_stop`：导航急停，依次发送手动零速度、禁用导航输出、
  再次发送零速度并取消目标，后端保持 `FAULT`，只能由用户显式重新选择模式解除。
- `POST /api/navigation/localization-spin/start`：确认周围安全后开始原地旋转一整圈定位。
  后端停止网页手动速度并调用 `/navigation/start_localization_spin`；机器人端负责临时
  接管导航输出，Web 不额外切换控制权。
- `POST /api/navigation/localization-spin/cancel`：调用
  `/navigation/cancel_localization_spin` 停止旋转。进度和最终结果来自 transient-local
  的 `/navigation/status` JSON；旋转结束后网页保持手动模式。
- `POST /api/foundation_pose/*`：FoundationPose 启停，需要 token。
- `POST /api/aruco/*`：ArUco 位姿识别和底盘微调启停，需要 token。常用动作为
  `start_pose`、`stop_pose`、`start_fine`、`stop_fine`、`enable_fine`、
  `disable_fine`。
- `POST /api/actions/*`、`POST /api/sequences/*`：动作和流程，需要 token。

### VLA / OpenPI

需要让另一台设备实现与本页面相同的 VLA 真机控制时，请使用
[`docs/R1PRO_VLA_REMOTE_DEVICE_INTEGRATION.md`](docs/R1PRO_VLA_REMOTE_DEVICE_INTEGRATION.md)。
该文档同时给出推荐的 HTTP 网关接入方式，以及独立实现 ROS 2 + OpenPI 客户端所需的
机器人话题、QoS、单位、相机同步、状态历史、动作契约和安全检查。

- `POST /api/vla/connect`：连接仅支持 `r1pro.v2` 的 Policy Server，然后发送
  `{"command":"list_tasks"}`；页面任务列表完全来自服务端。
- `POST /api/vla/load_task`：发送 `{"command":"load_task","task":"..."}`；响应包含
  完整 `inference_info`、模型控制布局和客户端可选执行布局。
- `POST /api/vla/infer`：严格按服务端 state/action shape 推理。发送给策略的 observation
  只包含 `images.cam_high`、`images.cam_left_wrist`、`images.cam_right_wrist` 和
  `state`，图像使用 `resize_with_pad(..., 224, 224)`。
- `POST /api/vla/execute`：`execution_schema` 可为
  `server`、`left_arm_8`、`right_arm_8`、`bimanual_16` 或
  `upper_body_20`。推理始终保留服务端完整维度，只有 ROS 下发会按该选项选择
  控制组；未选择的手臂或躯干不会发布保持目标。v2 不支持 RTC，旧 RTC API
  会返回明确错误。
- `POST /api/vla/mcap_channels`、`POST /api/vla/mcap_dry_run`：检查 MCAP channel 和同步帧。
- `POST /api/vla/mcap_eval_start`、`POST /api/vla/mcap_eval_stop`、
  `POST /api/vla/mcap_eval_result`：使用远端
  Policy Server 进行 action-horizon 开环评估。结果只保存在当前 Web 进程内存并
  在页面显示，不写入 NPZ、JSON 或 PNG。

VLA 默认连接 `10.30.20.1:18081`。头部图像使用
`/hdas/camera_head/left_raw/image_raw_color/compressed`；
左右腕分别使用 `/hdas/camera_wrist_left/color/image_raw/compressed` 和
`/hdas/camera_wrist_right/color/image_raw/compressed`。当前可执行契约要求服务端返回
绝对关节目标和 `absolute_0_to_100` 夹爪目标；客户端不反归一化、不叠加 state。
若相邻的 `vla_openpi/openpi/.venv` 存在，Web 会默认使用其中的 Python 和官方
`openpi-client` 启动 helper；也可通过 ROS 参数 `vla_helper_command` 显式覆盖。

默认模板动作可在网页“原子动作工作流”页面生成，也可调用：

```bash
TOKEN="$(cat ~/.ros/r1pro_web_control/token)"
curl -X POST http://127.0.0.1:8080/api/actions/generate_templates \
  -H "Content-Type: application/json" \
  -H "X-Control-Token: ${TOKEN}" \
  -d '{}'
```

生成的模板包含中文备注：

- `ee_relative_template`：左右手可配置的末端相对平移/旋转。
- `ee_pose_template`：左右手可配置的末端 6D 绝对位姿。
- `ee_axial_motion_template`：末端绕局部 L（默认 -Z）轴按角速度旋转，并在
  `base_link` 中同步线性平移；`start_angle_deg`、`end_angle_deg`、
  `speed_deg_s`、`axis_rotation_direction` 和 `delta_xyz_m` 均可编辑。
- `chassis_velocity_template`：按 `vx/vy/omega/duration_s` 下发底盘速度，
  到时自动停止。
- `navigate_to_pose_template`：切换到导航控制，发送 `map` 坐标系目标并等待进入
  位置/朝向容差；到位后自动切回手动控制。超时、工作流停止或导航故障时会取消
  活动目标并尝试恢复手动控制。

## Notes

所有可见 object、click、YOLO、FoundationPose、EE、grasp 坐标都解释为
`base_link`。IK 读取完整 R1 Pro URDF，锁定当前反馈里的躯干 4 轴，只求解
并下发选中手臂 7 轴。

“相机”页有四个独立显示槽，每个槽都有自己的下拉框，可以任意组合相机流，例如
同时查看 `Head Left Color`、`Head Depth` 和两个腕部 RGB。每个槽都有独立的
“保存路径”“拍照”和“点击取点”区域；先选择视频流并点击“确定”开始订阅，再
点击“拍照”即可把当前帧保存到指定路径。默认路径是
`~/Pictures/r1pro_web_control`，保存动作发生在运行 Web 服务的 Linux/WSL
主机上，不是浏览器下载目录。头部左相机的 Web/YOLO 使用处理器发布的
rectified RGB、`32FC1` aligned depth 和配套 CameraInfo；`Head Left Raw`
和 `Head Right Raw` 只作为显示流保留。“识别”页分别显示 YOLO overlay、
FoundationPose overlay 和 ArUco annotated image。ArUco 面板会启动
`r1pro_aruco_box_pose`，订阅 `/r1pro_aruco_box_pose/marker_pose_base_link_1`，
识别输入仍使用 `/hdas/camera_head/left_raw/image_raw_color/compressed`，
并通过微调开关控制 `/r1pro_aruco_fine_approach/enable`；微调速度最终发布到
`/motion_target/target_speed_chassis`，因此建议只在 Nav2 停稳后打开。
头部点云接口保留给调试/API 使用，默认界面不再显示点云画布：

```bash
ros2 topic echo --once /r1pro_web/camera_head/left/rectified/aligned_depth/image_raw --field encoding
ros2 topic hz /r1pro_web/camera_head/left/rectified/aligned_depth/image_raw
ros2 topic echo /r1pro_web/camera_head/depth_processor/status
ros2 topic hz /hdas/camera_head/points2
```

关闭开关或输出超过 2 秒未更新时，后端会清除头部 RGB-D、点击和识别缓存，并
停止仅配置为 `head_left` 的 YOLO/FoundationPose；腕部相机不受影响。Fast 模型及
权重仅限其许可证允许的内部研究/评估用途。当前双目标定质量仍需现场复核，正式
测量前必须用已知距离目标检查方向和尺度。

`导航` 页面默认对齐 `rviz2 -d ~/galaxea/galaxea_overlay/src/nav2.rviz`：
读取 `/map`、`/map_updates`、`/global_costmap/costmap`、`/local_costmap/costmap`、
`/plan`、`/transformed_global_plan` 和 `/odom`，向 `/goal_pose` 发布目标。打开本页安全锁后，
其中 `/odom` 会通过 `map -> odom` TF 转到地图坐标；局部代价地图也按自身
`frame_id`、`origin` 和分辨率叠加，不会再被拉伸成整幅静态地图。
网页地图图层按 Foxglove 风格分开显示：静态地图、橙色全局代价边界、青色局部
代价区域、`/plan` 全局执行路径、控制器局部路径，以及由 `map -> base_link` TF
驱动的机器人轮廓和朝向轴。
导航页的“清除全局代价地图”按钮调用
`/global_costmap/clear_entirely_global_costmap` 一次，清除移动人员等临时障碍物的
全局代价；静态 `/map` 不会被修改。发送导航目标前也会自动执行同一清除操作。
可以在地图上按下并拖拽选择目标方向，也可以手动输入 `x_m/y_m/yaw_deg` 后发送。
目标位姿旁可设置动作超时、位置容差、朝向容差和到位稳定时间，点击“保存导航原子
动作”即可生成 `navigate_to_pose` 动作并加入工作流。该动作占用 `chassis` 资源，
因此不能与同一步中的底盘速度或其他底盘动作并行。
取消导航会调用 `/navigate_to_pose/_action/cancel_goal` 并额外发送底盘零速度。

停止测试时可以在网页点 `STOP`，或使用：

```bash
TOKEN="$(cat ~/.ros/r1pro_web_control/token)"
curl -X POST http://127.0.0.1:8080/api/control/stop \
  -H "Content-Type: application/json" \
  -H "X-Control-Token: ${TOKEN}" \
  -d '{}'
```
