# R1 Pro 定位 TF 恢复与 DDS 通信优化

本文记录 2026-07-31 对 R1 Pro 导航定位链、外部导航接口和 Fast DDS 配置的修复，以及
2026-08-04 整机重启后的高负载复验和点云实时性优化。
当前实际部署范围是机器人 `192.168.3.4`；外部 Web 主机 `192.168.3.5` 的配置包已经
准备，但远端源码和进程尚未修改。

## 1. 修复结果

导航依赖的定位链为：

```text
map --AMCL--> odom --localization_to_odom--> body
```

本次增加了以下保护：

- AMCL 进程存活但 DDS/TF 接收线程失效时，自动停止导航输出、取消旧目标并重启 AMCL；
- 新 AMCL 由 launch respawn；watchdog 先请求 lifecycle manager `STARTUP`，超时后直接
  `configure/activate` 兜底；
- 使用最后可信的 `map -> odom` 和当前 `odom -> body` 续接位姿；
- 无法证明续接安全时不猜测位置，进入 `needs_initial_pose`；
- 外部导航可直接订阅 `/navigation/pose`，不需要消费整条 `/tf`；
- 点云自过滤与 PointCloud-to-LaserScan 都只保留最新一帧，CPU 忙时不再回放 5--8 帧
  历史点云；Livox FLOAT32 坐标直接逐轴计算，避免每帧构造 FLOAT64 Nx3 临时矩阵；
- DDS lease 调整为 30 秒、announcement 调整为 5 秒，关闭 Data Sharing 但保留本机 SHM
  transport，并提供 SIMPLE、CLIENT、SUPER_CLIENT 三种明确入口；
- Discovery Server 归一为 `robot_startup` 管理的 tmux 实例；2026-08-04 14:08
  已将旧 systemd unit 禁用并停止，避免失败重启风暴和双启动器争抢端口。
  生产当前仍默认 SIMPLE。

已有的 `robot_tf_watchdog` 继续独立负责 `robot_state_publisher`。它与 AMCL watchdog
不会互相杀进程；前者恢复关节/相机等机器人 TF，后者只恢复 `map -> odom`。

机器人厂包未提供 `mobiman` 源码，本机的 RSP 自动恢复暂依赖两个安装产物运维补丁：

- `mobiman/.../r1_pro_eepose_launch.py` 为唯一 RSP 设置 `respawn=True`、2 秒延迟和 15 Hz；
- `startup_config/.../start_mobiman_r1pro_joint_pid.sh` 保留原躯干 enable/disable 与 key
  加载逻辑，只额外向 joint tracker launch 传 `start_robot_state_publisher:=false`。

厂包重装可能覆盖它们。完整启动在停止任何组件前会检查这些必要内容，第 13 节也记录了
这一部署边界。

## 2. 原问题为什么表现为“几十 MB 积压”

故障时采集到：

- AMCL PID 仍存在，但 `/amcl_pose` publisher、`/scan` subscriber 和 `/tf` publisher
  已从 ROS 图消失；
- 最后一帧 `map -> odom` 停在 `1785491223.986816`，而 `odom -> body` 继续更新；
- 因两段 TF 不再有共同时间，Nav2 持续报告 past extrapolation；
- AMCL UDP `32900` 接收队列约 36 MB，`32956` 约 27 MB，而用户数据端口 `32957`
  仅约 81 KB。

Fast DDS 在 domain 102 的端口基址为：

```text
7400 + 250 * 102 = 32900
```

因此 `32900` 和偶数 participant 端口主要是 discovery/metatraffic，不是几十 MB 的
机器人位姿。接收线程停住后，发现报文继续进入内核 socket，直到接近系统
`rmem_max=40960000`。增大缓冲区只会推迟填满，不会修复线程，也不会恢复 TF。

正常运行时真正的大流量来自远端常驻订阅。现网 `/r1pro_web_server` 约有 54 个订阅，
包括 RELIABLE `/tf`、`/tf_static`、地图、代价地图、路径、点云、相机和深度图。历史
测得双腕 raw depth 每路约 `614 KB * 15 Hz = 9.22 MB/s`，两路约 18.4 MB/s；现场 WLAN
还观察到约 70.9 Mbps 发送。发现流量积压和正常用户数据大流量是两个不同问题。

另发现 enabled 的 `fastdds-discovery.service` 引用了不存在的 XML profile，持续每 3 秒
失败重启。它在创建有效 DDS server 前已经退出，所以不是 AMCL UDP 数十 MB 队列的直接
来源，但会造成无意义的进程创建和 journal 写入；正常监听 `11811` 的其实是 tmux 中的
另一个 server。第 7、10 节将两个入口归一。

### 2.1 高负载下的点云旧帧排队

2026-08-04 整机按最终 DDS 配置重启后，WLAN 在约 6 分钟内发送约 8.09 GB，持续约
20 MB/s 以上；这与远端尚未迁移的双腕 raw depth 常驻订阅吻合。8 核 CPU 长时间接近
饱和，最初 3 分钟同口径采样得到：

| 链路 | 频率 | 最大到达间隔 | 消息年龄均值 / 最大值 |
|---|---:|---:|---:|
| `/hdas/lidar_chassis_left` | 9.061 Hz | 0.393 s | 0.106 / 0.393 s |
| `..._left_filtered` | 3.989 Hz | 2.569 s | 0.809 / 3.018 s |
| `/scan` | 3.928 Hz | 3.557 s | 1.455 / 4.458 s |

原始 Livox 一直稳定，明显延迟从 Python `lidar_self_filter` 开始；其订阅原来
`KEEP_LAST(5)`，每帧又把 xyz 三次转换为 FLOAT64、堆成 Nx3 矩阵并再次生成变换矩阵。
下游 `pointcloud_to_laserscan` 的默认 `queue_size` 是 CPU 核数，本机即 8，又继续保留
旧点云。积压期间 watchdog 收不到新 `/scan`，所以按设计进入 `waiting_scan`、停车并停发
`/navigation/pose`；这不是 pose 定时器参数错误。

本次改为：

- self-filter 输入和输出 `BEST_EFFORT / VOLATILE / KEEP_LAST(1)`；
- `pointcloud_to_laserscan queue_size=1`；
- xyz 沿三个 body 轴分别用输入原精度计算；当前 Livox FLOAT32 不再升级到 FLOAT64，
  FLOAT64 输入仍保持 FLOAT64；NaN/Inf、包围盒闭区间、frame、stamp、点顺序及其它字段
  语义不变；
- self-filter、watchdog 和 Web 状态节点的动态 TF history 从默认 100 缩短到 10；仅用于
  新鲜度标记的 scan/odom reader 改为 `KEEP_LAST(1)`。

这些设置有意允许在处理不过来时丢弃旧传感器帧。导航需要的是最近观测；扩大 DDS 或
MessageFilter 队列只会让数据更旧。远端深度图的真实用户数据带宽仍需按第 11 节迁移
`192.168.3.5` 才能消除，机器人端队列优化不能替代远端退订。

## 3. AMCL watchdog

节点：`/amcl_watchdog`

### 3.1 判定条件

只有同时满足以下条件才允许重启 AMCL：

1. 地图已经收到；
2. AMCL lifecycle 最近明确为 active；
3. `/scan` 和 `/odom` 最近 2 秒内仍有数据；
4. `/odom` frame 必须是 `odom -> body`，位姿和四元数必须有限；
5. 系统曾经收到至少 5 帧有效 `map -> odom`，即首次定位确实成功过；
6. 直接 `map -> odom` 超过 3 秒没有新时间戳，并继续确认 5 秒；
7. 距离上次重启至少 60 秒；
8. `/proc` 中精确匹配到唯一 `amcl --ros-args -r __node:=amcl`。

以下情况会把导航标记为不安全，但不会杀 AMCL：首次定位尚未完成、地图/scan/odom
缺失、AMCL 被显式 deactivate、里程计 frame 错误、PID 为零或出现多个 AMCL。

### 3.2 重启与位姿续接

```text
map->odom 断流
    │ 3 s
    ▼
degraded / localization_safe=false
    │ 再确认 5 s
    ▼
SIGTERM 唯一 AMCL PID
    │ 最多 5 s
    ├── 正常退出
    └── 未退出 -> SIGKILL
                   │
                   ▼
             launch 2 s 后 respawn
                   │
                   ▼
       lifecycle manager STARTUP（最多等待 2 s）
                   │
          ┌────────┴────────┐
          │ active          │ 未恢复
          │                 ▼
          │       direct configure/activate
          └────────┬────────┘
                   │
                   ▼
  map->body = cached(map->odom) * current(odom->body)
                   │
                   ▼
         /initialpose 发布 3 次，每次间隔 0.5 s
                   │
                   ▼
       新 map->odom 连续稳定 5 帧 -> healthy
```

恢复协方差使用最后一帧 `/amcl_pose`：所有有限元素（包括负的交叉协方差）原样保留，
NaN/Inf 清零，并保证 x/y 方差至少 `0.25 m²`、yaw 方差至少 `0.06854 rad²`。以下情况
禁止自动发初始位姿：

- `map -> odom` 缓存超过 60 秒；
- `/odom` 时间戳倒退；
- 相邻有效 `/odom` 接收间隔超过 2 秒，无法证明发布器没有在断流期间重置；
- 相邻里程计在 2 秒内跳变超过 2 m 或 90°；
- odom frame、数值或四元数无效；
- 直接 `map -> odom` 数值非有限或四元数无效；
- 故障前没有可信定位缓存。

禁止自动续接时状态为 `needs_initial_pose`，机器人保持导航输出 disabled。此时需要
人工发布 `/initialpose`，或显式调用全局定位并重新完成定位旋转。

### 3.3 lifecycle 设置

- AMCL：`respawn=True`，`respawn_delay=2.0`；
- localization manager：`attempt_respawn_reconnection=True`；
- `bond_respawn_max_duration=60.0`；
- `bond_timeout=4.0`；watchdog 已先做 3+5 秒保守确认，因此无需再等待 20 秒；
- respawn 后 watchdog 先调用 `/lifecycle_manager_localization/manage_nodes` 的 `STARTUP`；
- manager service/响应或 active 状态等待默认最多 2 秒；未恢复时以 0.6 秒超时、0.2 秒
  间隔查询 `/amcl/get_state`，service 尚未发现或状态仍为 unknown 时继续短周期轮询；
- fallback 根据 lifecycle 状态直接调用 `/amcl/change_state`：unconfigured 执行 configure、
  inactive 执行 activate；真正 transition 失败后保持 safe=false，1 秒后重试，不会在
  AMCL 未 active 时发布续接位姿。

## 4. 导航安全联锁

`cmd_vel_to_twist_stamped` 保留原有人工控制权 `publish_enabled`，另增加独立的定位安全
门。AMCL 模式下必须持续收到 true 的 `/navigation/localization_safe`；bringup 使用
3 秒心跳超时。

定位 unsafe 或心跳超时时：

1. 立即向 `/motion_target/target_speed_chassis` 发布一帧零速度；
2. 强制导航输出状态变为 `disabled`；
3. 停止转发后续 Nav2 `/cmd_vel`；
4. 持续重试 `/navigate_to_pose/_action/cancel_goal` 和
   `/navigate_through_poses/_action/cancel_goal`；
   单次取消 future 超过 2 秒会被作废并立即重试，迟到的旧回调不能解除安全门；
5. Web 发起且仍 pending/running 的 Nav2 `Spin` 同样会被取消；即使没有收到显式 false，
   安全心跳超时也会触发取消并关闭导航输出；
6. 在两个导航取消接口确认前拒绝重新启用导航；
7. 恢复 healthy 后只解除安全门，不恢复原控制权，也不继续旧目标。

unsafe -> safe 边界还有一个独立取消代际：收到恢复心跳时，即使 unsafe
期间的 CancelGoal future 仍在飞行，也会要求一个更新的最终 cancel-all generation。
旧代际的迟到回调只能释放它自己的 in-flight 槽位，不能确认或清除恢复后的
新代际；两个取消接口都确认这个最终代际前，显式重新启用请求仍被拒绝。
这修复了“unsafe 期间旧请求的回调，在 safe 边界误解除取消门”的竞态；输出全程
保持 disabled，不会因 safe 回调自动重启。

操作端必须重新调用 `/cmd_vel_to_twist_stamped/set_nav_output_enabled`，再重新下发目标。
factory localization 模式会显式关闭 AMCL 健康门，保持原行为。

## 5. 对外导航接口

| 名称 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/navigation/pose` | `geometry_msgs/msg/PoseStamped` | BEST_EFFORT、VOLATILE、KEEP_LAST(1) | 10 Hz 当前 `map -> body`；仅 healthy 时发布 |
| `/navigation/status` | `std_msgs/msg/String` JSON v1 | RELIABLE、TRANSIENT_LOCAL、KEEP_LAST(1) | 2 Hz 综合导航状态 |
| `/navigation/amcl_health` | `std_msgs/msg/String` JSON v1 | RELIABLE、TRANSIENT_LOCAL、KEEP_LAST(1) | watchdog 状态、输入年龄、PID 和计数 |
| `/navigation/localization_safe` | `std_msgs/msg/Bool` | RELIABLE、TRANSIENT_LOCAL、KEEP_LAST(1) | 2 Hz 安全心跳 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 现有兼容接口 | unsafe 时由 filter 拒绝 |
| `/navigation/start_localization_spin` | `std_srvs/srv/Trigger` | service | 仅健康门、导航 lifecycle 和输出服务就绪后启动原地定位旋转 |
| `/navigation/cancel_localization_spin` | `std_srvs/srv/Trigger` | service | 取消 Web 发起的定位旋转 |
| `/cmd_vel_to_twist_stamped/set_nav_output_enabled` | `std_srvs/srv/SetBool` | service | 定位安全且旧目标取消完成后才允许 true |

`/navigation/status` 仍为 `version: 1`，旧字段没有删除。新增 `localization` 对象：

```json
{
  "localization": {
    "required": true,
    "ready": true,
    "tf_ready": true,
    "tf_age_sec": 0.03,
    "health_state": "healthy",
    "health_detail": "AMCL map->odom and inputs are current",
    "safe": true,
    "safe_heartbeat_fresh": true,
    "safe_heartbeat_age_sec": 0.2,
    "restart_count": 1,
    "sigkill_count": 1,
    "amcl_pid": 12345,
    "ages_sec": {"scan": 0.02, "odom": 0.01, "map_to_odom": 0.03}
  }
}
```

watchdog 主要状态：

| 状态 | 含义 |
|---|---|
| `waiting_initial_pose` | 从未建立有效定位，不自动杀 AMCL |
| `stabilizing` | TF 已出现，等待连续稳定样本 |
| `healthy` | 输入与直接 `map -> odom` 均正常，safe=true |
| `degraded` | TF 已过期，正在确认故障 |
| `terminating` / `killing` | 正在结束旧 AMCL |
| `recovering` / `reinitializing` | 等待 respawn、lifecycle 和初始位姿恢复 |
| `needs_initial_pose` | 自动续接不安全，需要人工定位 |
| `restart_refused` | PID 不唯一等原因拒绝发信号 |

## 6. 构建与测试

```bash
cd /home/nvidia/galaxea/galaxea_overlay
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash

colcon build --symlink-install \
  --packages-select r1_pro_localization r1_pro_navigation2

source install/setup.bash
colcon test --packages-select r1_pro_localization r1_pro_navigation2
colcon test-result --verbose
```

DDS 环境入口和 XML 另做静态校验：

```bash
bash -n \
  src/r1_pro_navigation2/config/source_fastdds_simple.sh \
  src/r1_pro_navigation2/config/source_fastdds_client.sh \
  src/r1_pro_navigation2/config/source_fastdds_super_client.sh

xmllint --noout \
  src/r1_pro_navigation2/config/fastdds_robot_udp.xml \
  /home/nvidia/galaxea/install/startup_config/share/startup_config/script/prestart/fastrtps_profiles.xml \
  /home/nvidia/galaxea_dev/external_web_192_168_3_5/fastrtps_profiles_p2p_web_3_5_to_3_4.xml
```

覆盖的单元测试包括：

- AMCL PID 精确匹配；
- 刚体变换复合与四元数校验；
- 有限协方差（含负交叉项）保持、非有限值清洗和对角下限；
- odom 位移/角度跳变检测；
- odom 长接收间隔换 epoch、异常直接 TF 作废恢复缓存；
- 挂起 CancelGoal future 的超时重试和迟到回调隔离；
- unsafe -> safe 边界必须产生新的最终 cancel generation，旧代回调不能
  确认新代，且恢复不会自动重启导航输出；
- 定位安全心跳未收到、false、刚好超时和 factory bypass；
- Web 状态门和目标过滤门。
- 点云包围盒边界、旋转/平移、NaN/Inf、FLOAT32/FLOAT64 兼容及非 XYZ 字段保持。

README 同样安装到 `share/r1_pro_navigation2/README.md`，源码与安装空间读取的是同一份
symlink-install 文档。

最终证据是 2026-08-04 15:07--15:08 生成的两包 xUnit/CTest；其时间晚于
unsafe -> safe generation 源码和测试的 14:57:39 修改时间。`colcon test-result`
汇总为 78 tests，0 errors、0 failures、0 skipped，因此该结果是在最后加固后生成，
不是沿用旧 xUnit。

## 7. 生产 SIMPLE 模式启动与重启

当前生产模式固定为 SIMPLE。以下命令使用安装路径；本工作空间采用 symlink-install，路径
会解析到源码中的同名文件。机器人必须静止，周围无人员和障碍物，网页手动控制也必须
停止。

日常完整冷启动已经封装为两个一键入口，必须按顺序从普通 SSH/本机终端执行：

```bash
bash ~/start_r1pro_base.sh
bash ~/start_r1pro_navigation.sh
```

第一条包含 EHI 停止、基础 tmux allowlist、零 SHM 打开文件门禁、CAN 配置和
`R1PROBody` 启动；成功时打印 `BASE_READY`，并有意让 EHI 保持停止。只有看到
`BASE_READY` 才能执行第二条。第二条拒绝重复 navigation/AMCL，等待 AMCL
active/healthy/safe 后恢复 EHI，成功时打印 `NAVIGATION_READY`；导航底盘输出仍保持
disabled。两个 home 入口分别转到安装空间的 `start_r1pro_base.sh` 和
`start_r1pro_navigation.sh`，长命令的可审计实现仍由本包维护。

当前一键基础启动有意将“躯干保护”默认关闭：`R1PROBody.d/mobiman.yaml` 使用
`start_mobiman_r1pro_joint_pid.sh slow enable_torso`，最终参数必须为
`disable_torso=false`。这表示 joint tracker 可以输出躯干控制，不是硬件保护关闭或绕过
急停。基础脚本会在停止现有系统前检查该配置与 torso key，并在启动前拒绝任何未退出的
VR/独立 torso speed controller；启动后还会确认唯一 joint tracker 的命令行明确包含
`disable_torso:=false`。厂包重装若覆盖该 YAML，脚本会 fail-stop，不能静默退回
`disable_torso=true`。

### 7.0 原启动命令是否还能用

原来的 `ros2 launch r1_pro_navigation2 body_attach.launch.py
map_yaml:=.../mx_map.yaml` 主体和地图参数仍然正确，但整套命令不能原样继续使用：

- 三行 profile export 应改为最后显式 source `source_fastdds_simple.sh`；后者还统一设置
  RMW/domain/interface，并清除遗留 CLIENT/SUPER_CLIENT 变量；
- 完整重启必须先停止 `ehi-gateway.service`，否则仍有 Fast DDS participant 持有 SHM；
- 禁止 `find /dev/shm ... -delete`，只能在全部 participant 退出且零打开文件门禁通过后
  执行 `fastdds shm clean`；
- 仅重启 navigation 时其它 participant 仍在运行，绝对不能清 SHM；
- 当前生产仍为 SIMPLE，远端 `192.168.3.5` 未同步前不能把业务节点切到 CLIENT。

完整基础重启使用 7.1；基础系统正常、只需更新导航时使用 7.2。

### 7.1 基础服务、导航和 EHI 完整重启

整段复制到机器人终端执行。命令放在子 shell 中，失败只退出该命令块，不会关闭当前
终端。它是 fail-stop 流程而不是自动回滚事务：一旦执行到停止阶段，后续任何 ERROR 都
可能留下 EHI stopped、基础 tmux 已停止或只启动了一部分。此时不要恢复人工控制或跳过
门禁；先查看错误和当前 unit/tmux/PID，再从本节开头完整重跑或按明确故障点恢复。

必须从普通 SSH 或本机终端执行，不能在机器人现有 tmux session 内运行；vendor 的
`robot_startup.sh kill` 会执行 `tmux kill-server`，在 tmux 内运行会连同当前脚本一起杀掉。

命令块有意使用 `set -Ee -o pipefail`，不要改成 `set -Eeuo pipefail`。ROS 2 Humble 的
setup 脚本会读取尚未定义的 `AMENT_TRACE_SETUP_FILES`；在 source 之前启用 `nounset`
会使流程立即以 `unbound variable` 退出，后续停机和启动步骤都不会执行。

EHI 配置了 `Restart=always`，必须使用 `systemctl stop`；直接 kill 会被自动拉起。

```bash
(
set -Ee -o pipefail

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
STARTUP_DIR=/home/nvidia/galaxea/install/startup_config/share/startup_config/script
MAP_YAML=/home/nvidia/galaxea_dev/maps/mx_map.yaml
RSP_LAUNCH=/home/nvidia/galaxea/install/mobiman/share/mobiman/launch/simpleExample/R1_PRO/r1_pro_eepose_launch.py
MOBIMAN_START="$STARTUP_DIR/boot/modules/mobiman/start_mobiman_r1pro_joint_pid.sh"
DISCOVERY_START="$STARTUP_DIR/boot/modules/system/start_discover_server.sh"

test -r "$DDS_CONFIG/source_fastdds_simple.sh"
test -r "$DDS_CONFIG/fastdds_robot_udp.xml"
test -r "$MAP_YAML"
test -x "$STARTUP_DIR/robot_startup.sh"
test -r "$RSP_LAUNCH"
test -x "$MOBIMAN_START"
test -x "$DISCOVERY_START"
for REQUIRED_COMMAND in fastdds ip lsof pgrep python3 rg ss sudo systemctl timeout tmux xmllint; do
  command -v "$REQUIRED_COMMAND" >/dev/null || {
    echo "ERROR: required command not found: $REQUIRED_COMMAND" >&2
    exit 1
  }
done

# 厂包安装产物门禁。任何一项缺失都先重新应用运维补丁，不能启动一个 watchdog 会杀死、
# 但 launch 不会 respawn 的 RSP，也不能同时启动第二个 RSP。
rg -q 'respawn\s*=\s*True' "$RSP_LAUNCH"
rg -q "['\"]publish_frequency['\"]\s*:\s*15(\.0)?" "$RSP_LAUNCH"
test "$(rg -c 'start_robot_state_publisher:=false' "$MOBIMAN_START")" -eq 2
rg -q 'disable_torso:=\$\{DISABLE_TORSO\}' "$MOBIMAN_START"

# Discovery Server 启动器也属于 vendor 安装产物；必须先清掉 CLIENT/SUPER_CLIENT 环境，
# 且只监听预定的 11811。厂包覆盖后此门禁会在停机前失败。
rg -q '^unset ROS_DISCOVERY_SERVER$' "$DISCOVERY_START"
rg -q '^unset ROS_SUPER_CLIENT$' "$DISCOVERY_START"
rg -q '^unset R1PRO_DISCOVERY_SERVER$' "$DISCOVERY_START"
rg -q '^exec fastdds discovery -i 0 -l 0\.0\.0\.0 -p 11811$' "$DISCOVERY_START"

# EHI 保留 systemd env 中的 legacy profile 路径，但其 XML 策略必须已同步为
# 30 s / 5 s、Data Sharing OFF、默认 SHM+UDP（无 LARGE_DATA）。
EHI_FASTRTPS=$(awk -F= '$1 == "FASTRTPS_DEFAULT_PROFILES_FILE" {print $2}' \
  /etc/ehi-gateway.env)
EHI_FASTDDS=$(awk -F= '$1 == "FASTDDS_DEFAULT_PROFILES_FILE" {print $2}' \
  /etc/ehi-gateway.env)
if [ -z "$EHI_FASTRTPS" ] || [ "$EHI_FASTRTPS" != "$EHI_FASTDDS" ] || \
   [ ! -r "$EHI_FASTRTPS" ]; then
  echo 'ERROR: EHI Fast DDS profile paths are missing, different, or unreadable' >&2
  exit 1
fi
xmllint --noout "$EHI_FASTRTPS"
EHI_LEASE=$(xmllint --xpath \
  'string(//*[local-name()="leaseDuration"]/*[local-name()="sec"])' "$EHI_FASTRTPS")
EHI_ANNOUNCEMENT=$(xmllint --xpath \
  'string(//*[local-name()="leaseAnnouncement"]/*[local-name()="sec"])' "$EHI_FASTRTPS")
EHI_DATA_OFF_COUNT=$(xmllint --xpath \
  'count(//*[local-name()="data_sharing"]/*[local-name()="kind" and text()="OFF"])' \
  "$EHI_FASTRTPS")
EHI_LARGE_DATA_COUNT=$(xmllint --xpath \
  'count(//*[local-name()="builtinTransports" and normalize-space(text())="LARGE_DATA"])' \
  "$EHI_FASTRTPS")
if [ "$EHI_LEASE" != 30 ] || [ "$EHI_ANNOUNCEMENT" != 5 ] || \
   [ "$EHI_DATA_OFF_COUNT" != 2 ] || [ "$EHI_LARGE_DATA_COUNT" != 0 ]; then
  echo 'ERROR: EHI legacy profile has not been synchronized to the production SIMPLE policy' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"

# 在停止任何服务前一次性验证 sudo，避免停机一半才发现无权限。
sudo -v

if [ -n "${TMUX:-}" ]; then
  echo 'ERROR: run this full restart from a normal SSH/local terminal, not inside tmux' >&2
  exit 1
fi

# robot_startup.sh kill 会 kill 默认 tmux socket 中的全部 session。先做 allowlist；
# 若有不属于本机基础/导航的 session，拒绝停机，避免误杀用户任务。
mapfile -t EXISTING_TMUX_SESSIONS < <(
  tmux list-sessions -F '#{session_name}' 2>/dev/null || true
)
for SESSION_NAME in "${EXISTING_TMUX_SESSIONS[@]}"; do
  case "$SESSION_NAME" in
    hdas|mobiman|navigation|ros_discovery|tools) ;;
    *)
      echo "ERROR: unexpected tmux session '$SESSION_NAME'; full restart refused" >&2
      exit 1
      ;;
  esac
done
printf 'WARNING: robot_startup.sh kill will terminate every session on the default tmux socket: %s\n' \
  "${EXISTING_TMUX_SESSIONS[*]:-(none)}" >&2

# 若接口仍可用，先撤销导航底盘输出；失败时后续仍按停机流程处理。
timeout 5 ros2 service call \
  /cmd_vel_to_twist_stamped/set_nav_output_enabled \
  std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true

# 不能用 kill/pkill 代替：该 service 的 Restart=always 会自动重启进程。
sudo systemctl stop ehi-gateway.service
if systemctl is-active --quiet ehi-gateway.service; then
  echo 'ERROR: ehi-gateway.service did not stop' >&2
  exit 1
fi

# Discovery Server 只保留 vendor robot_startup 管理的 ros_discovery tmux 实例。
# 旧 fastdds-discovery.service 引用了不存在的 XML profile，并会每 3 秒失败重启；
# 若修好它又不禁用 tmux，则两个启动器会争抢 11811。因此在完整停机窗口永久禁用旧 unit。
sudo systemctl disable --now fastdds-discovery.service
DISCOVERY_UNIT_FILE_STATE=$(systemctl show fastdds-discovery.service \
  -p UnitFileState --value)
DISCOVERY_ACTIVE_STATE=$(systemctl show fastdds-discovery.service \
  -p ActiveState --value)
DISCOVERY_SUB_STATE=$(systemctl show fastdds-discovery.service \
  -p SubState --value)
if [ "$DISCOVERY_UNIT_FILE_STATE" != disabled ] || \
   [ "$DISCOVERY_ACTIVE_STATE" != inactive ] || \
   [ "$DISCOVERY_SUB_STATE" != dead ]; then
  echo "ERROR: fastdds-discovery.service state is file=$DISCOVERY_UNIT_FILE_STATE active=$DISCOVERY_ACTIVE_STATE sub=$DISCOVERY_SUB_STATE" >&2
  exit 1
fi

cd "$STARTUP_DIR"
if [ "${#EXISTING_TMUX_SESSIONS[@]}" -gt 0 ]; then
  ./robot_startup.sh kill
else
  ros2 daemon stop >/dev/null 2>&1 || true
fi

# SHM 清理门禁：最多等待 15 秒。只要任一 Fast DDS 文件仍被打开就中止，
# 不删除 /dev/shm，也不继续启动，先人工处理输出中的残留 PID。
FASTDDS_OPEN=''
for _attempt in $(seq 1 15); do
  # lsof 在“没有任何打开文件”时返回 1；它是成功门禁条件，不能被 set -e 误判为错误。
  # 但 sudo/路径/权限错误也可能返回非零，因此只有 rc=1 且输出为空才视为无打开文件。
  if LSOF_OUTPUT=$(sudo lsof -nP +D /dev/shm 2>&1); then
    LSOF_RC=0
  else
    LSOF_RC=$?
  fi
  if [ "$LSOF_RC" -ne 0 ]; then
    if [ "$LSOF_RC" -eq 1 ] && [ -z "$LSOF_OUTPUT" ]; then
      FASTDDS_OPEN=''
      break
    fi
    echo "ERROR: cannot audit /dev/shm with lsof (rc=$LSOF_RC):" >&2
    printf '%s\n' "$LSOF_OUTPUT" >&2
    exit 1
  fi
  FASTDDS_OPEN=$(printf '%s\n' "$LSOF_OUTPUT" | \
    awk 'NR > 1 && $0 ~ /\/dev\/shm\/(fastrtps_|sem\.fastrtps_)/ {print}')
  [ -z "$FASTDDS_OPEN" ] && break
  sleep 1
done
if [ -n "$FASTDDS_OPEN" ]; then
  echo 'ERROR: Fast DDS participants still hold SHM files; cleanup refused:' >&2
  printf '%s\n' "$FASTDDS_OPEN" >&2
  exit 1
fi

# fastdds shm clean 只清理 zombie 文件；即使如此也必须先通过上面的零打开文件门禁。
fastdds shm clean

# 不调用含硬编码凭据且会吞掉 sudo/ip 错误的 ~/can.sh。set -e 会使任一步
# 失败立即中止，不在 CAN 未就绪时继续 boot。
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.875 \
  dbitrate 5000000 fd on dsample-point 0.875
sudo ip link set can0 up
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

# 相机、底盘、robot_state_publisher、odom 和 lidar 需要先形成稳定基础链路。
# navigation preflight 自身只有有限超时，不能在基础 tmux 刚创建后立即启动。
sleep 20

mapfile -t BASE_TMUX_SESSIONS < <(tmux list-sessions -F '#{session_name}')
for EXPECTED_SESSION in hdas mobiman ros_discovery tools; do
  tmux has-session -t "$EXPECTED_SESSION"
done
for SESSION_NAME in "${BASE_TMUX_SESSIONS[@]}"; do
  case "$SESSION_NAME" in
    hdas|mobiman|ros_discovery|tools) ;;
    *) echo "ERROR: unexpected post-boot tmux session '$SESSION_NAME'" >&2; exit 1 ;;
  esac
done

mapfile -t RSP_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher .*__node:=robot_state_publisher( |$)' || true)
test "${#RSP_PIDS[@]}" -eq 1
test "$(pgrep -fc '^/home/nvidia/galaxea/install/mobiman/lib/mobiman/chassis_control_node( |$)' || true)" -eq 1
test "$(pgrep -fc '^/home/nvidia/galaxea/install/HDAS/lib/HDAS/HDAS( |$)' || true)" -eq 1

DISCOVERY_COUNT=$(pgrep -fc \
  '^/opt/ros/humble/bin/fast-discovery-server .* -p 11811( |$)' || true)
DISCOVERY_SS=$(ss -Hlunp)
if [ "$DISCOVERY_COUNT" -ne 1 ] || \
   ! rg -q ':11811\b.*fast-discovery-' <<<"$DISCOVERY_SS"; then
  echo "ERROR: expected exactly one tmux Discovery Server on UDP 11811; found $DISCOVERY_COUNT" >&2
  exit 1
fi

if tmux has-session -t navigation 2>/dev/null; then
  echo 'ERROR: navigation tmux session already exists' >&2
  exit 1
fi

# 在 tmux 子 shell 内再次 source SIMPLE，避免父 shell、login shell 或旧 daemon 环境串入。
tmux new-session -d -s navigation "bash -lc '
export R1PRO_FASTDDS_DISCOVERY_MODE=simple
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/source_fastdds_simple.sh
exec ros2 launch r1_pro_navigation2 body_attach.launch.py \
  map_yaml:=/home/nvidia/galaxea_dev/maps/mx_map.yaml
'"

# 拒绝“有 tmux 但无 launch”或双 launch；最多等待 60 s。
for _attempt in $(seq 1 60); do
  mapfile -t NAV_LAUNCH_PIDS < <(pgrep -f \
    '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
  [ "${#NAV_LAUNCH_PIDS[@]}" -le 1 ] || {
    echo "ERROR: duplicate navigation launches: ${NAV_LAUNCH_PIDS[*]}" >&2
    exit 1
  }
  [ "${#NAV_LAUNCH_PIDS[@]}" -eq 1 ] && break
  sleep 1
done
test "${#NAV_LAUNCH_PIDS[@]}" -eq 1

for _attempt in $(seq 1 60); do
  mapfile -t AMCL_PIDS < <(pgrep -f \
    '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
  [ "${#AMCL_PIDS[@]}" -le 1 ] || {
    echo "ERROR: duplicate AMCL processes: ${AMCL_PIDS[*]}" >&2
    exit 1
  }
  [ "${#AMCL_PIDS[@]}" -eq 1 ] && break
  sleep 1
done
test "${#AMCL_PIDS[@]}" -eq 1

# 这两个进程属于 navigation launch，而不是 R1PROBody 基础 tmux；必须在 navigation
# 启动后检查。后面的 healthy 门还会进一步验证 map/scan/odom 数据确实可用。
test "$(pgrep -fc '/r1_pro_odom/lib/r1_pro_odom/localization_to_odom\.py .*__node:=localization_to_odom( |$)' || true)" -eq 1
test "$(pgrep -fc '^/home/nvidia/galaxea/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node( |$)' || true)" -eq 1

AMCL_STATE=''
for _attempt in $(seq 1 30); do
  if AMCL_STATE=$(timeout --signal=INT --kill-after=2 6 \
      ros2 lifecycle get /amcl --no-daemon 2>/dev/null) && \
     printf '%s\n' "$AMCL_STATE" | rg -q '^active \[3\]$'; then
    break
  fi
  sleep 1
done
printf '%s\n' "$AMCL_STATE" | rg -q '^active \[3\]$'

# 持续订阅直到收到 healthy + safe=true，再用 JSON parser 显式断言。
HEALTH_JSON=$(timeout --signal=INT --kill-after=2 90 ros2 topic echo \
  /navigation/amcl_health std_msgs/msg/String --no-daemon --once --field data \
  --full-length --no-lost-messages --qos-reliability reliable \
  --qos-durability transient_local \
  --filter "'\"state\":\"healthy\"' in m.data and '\"safe\":true' in m.data")
HEALTH_JSON=$(printf '%s\n' "$HEALTH_JSON" | sed '/^---$/d')
printf '%s\n' "$HEALTH_JSON" | python3 -c \
  'import json,sys; d=json.load(sys.stdin); assert d["state"] == "healthy" and d["safe"] is True and d["lifecycle_state"] == "active"'

sudo systemctl start ehi-gateway.service
systemctl is-active --quiet ehi-gateway.service
tmux list-sessions
)
```

不要在该流程中恢复手工 `find /dev/shm ... -delete`。若 SHM 门禁失败，说明停机不完整；
应先处理列出的 PID，而不是强删仍被进程映射的文件。

### 7.2 仅重启导航

基础 HDAS、mobiman、EHI 正常，仅需更新或恢复导航时使用下面命令。机器人仍须静止。
此场景其它 Fast DDS participant 正在运行，严禁执行 `fastdds shm clean`。

```bash
(
set -Ee -o pipefail

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"

timeout 5 ros2 service call \
  /cmd_vel_to_twist_stamped/set_nav_output_enabled \
  std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true

NAV_PID=''
NAV_PGID=''
if tmux has-session -t navigation 2>/dev/null; then
  mapfile -t NAV_PANE_PIDS < <(tmux list-panes -t navigation -F '#{pane_pid}')
  test "${#NAV_PANE_PIDS[@]}" -eq 1
  NAV_PID=${NAV_PANE_PIDS[0]}
  NAV_PGID=$(ps -o pgid= -p "$NAV_PID" | tr -d ' ')
  test -n "$NAV_PGID"
  tmux kill-session -t navigation
else
  # tmux 丢失但 launch/AMCL 仍存在时，不能盲目新建第二套。
  mapfile -t ORPHAN_NAV_PIDS < <(pgrep -f \
    '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
  mapfile -t ORPHAN_AMCL_PIDS < <(pgrep -f \
    '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
  if [ "${#ORPHAN_NAV_PIDS[@]}" -gt 0 ] || [ "${#ORPHAN_AMCL_PIDS[@]}" -gt 0 ]; then
    echo "ERROR: navigation tmux is absent but launch/AMCL is still alive; launch=${ORPHAN_NAV_PIDS[*]:-(none)} amcl=${ORPHAN_AMCL_PIDS[*]:-(none)}" >&2
    echo 'Use the full restart after identifying the orphan process owner.' >&2
    exit 1
  fi
fi

# 等待原 navigation 整个进程组退出；不自动 SIGKILL，也不与新实例并存。
if [ -n "$NAV_PGID" ]; then
  for _attempt in $(seq 1 15); do
    if ! ps -eo pgid= | awk -v pgid="$NAV_PGID" '$1 == pgid {found=1} END {exit !found}'; then
      break
    fi
    sleep 1
  done
  NAV_LEFTOVERS=$(ps -eo pid=,pgid=,args= | \
    awk -v pgid="$NAV_PGID" '$2 == pgid {print}')
  if [ -n "$NAV_LEFTOVERS" ]; then
    echo 'ERROR: old navigation process group is still alive:' >&2
    printf '%s\n' "$NAV_LEFTOVERS" >&2
    exit 1
  fi
fi

mapfile -t OLD_NAV_PIDS < <(pgrep -f \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
mapfile -t OLD_AMCL_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
if [ "${#OLD_NAV_PIDS[@]}" -gt 0 ] || [ "${#OLD_AMCL_PIDS[@]}" -gt 0 ]; then
  echo "ERROR: old navigation processes remain; launch=${OLD_NAV_PIDS[*]:-(none)} amcl=${OLD_AMCL_PIDS[*]:-(none)}" >&2
  exit 1
fi

tmux new-session -d -s navigation "bash -lc '
export R1PRO_FASTDDS_DISCOVERY_MODE=simple
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/source_fastdds_simple.sh
exec ros2 launch r1_pro_navigation2 body_attach.launch.py \
  map_yaml:=/home/nvidia/galaxea_dev/maps/mx_map.yaml
'"

for _attempt in $(seq 1 60); do
  mapfile -t NAV_LAUNCH_PIDS < <(pgrep -f \
    '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
  [ "${#NAV_LAUNCH_PIDS[@]}" -le 1 ] || {
    echo "ERROR: duplicate navigation launches: ${NAV_LAUNCH_PIDS[*]}" >&2
    exit 1
  }
  [ "${#NAV_LAUNCH_PIDS[@]}" -eq 1 ] && break
  sleep 1
done
test "${#NAV_LAUNCH_PIDS[@]}" -eq 1

for _attempt in $(seq 1 60); do
  mapfile -t AMCL_PIDS < <(pgrep -f \
    '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
  [ "${#AMCL_PIDS[@]}" -le 1 ] || {
    echo "ERROR: duplicate AMCL processes: ${AMCL_PIDS[*]}" >&2
    exit 1
  }
  [ "${#AMCL_PIDS[@]}" -eq 1 ] && break
  sleep 1
done
test "${#AMCL_PIDS[@]}" -eq 1

AMCL_STATE=''
for _attempt in $(seq 1 30); do
  if AMCL_STATE=$(timeout --signal=INT --kill-after=2 6 \
      ros2 lifecycle get /amcl --no-daemon 2>/dev/null) && \
     printf '%s\n' "$AMCL_STATE" | rg -q '^active \[3\]$'; then
    break
  fi
  sleep 1
done
printf '%s\n' "$AMCL_STATE" | rg -q '^active \[3\]$'

HEALTH_JSON=$(timeout --signal=INT --kill-after=2 90 ros2 topic echo \
  /navigation/amcl_health std_msgs/msg/String --no-daemon --once --field data \
  --full-length --no-lost-messages --qos-reliability reliable \
  --qos-durability transient_local \
  --filter "'\"state\":\"healthy\"' in m.data and '\"safe\":true' in m.data")
HEALTH_JSON=$(printf '%s\n' "$HEALTH_JSON" | sed '/^---$/d')
printf '%s\n' "$HEALTH_JSON" | python3 -c \
  'import json,sys; d=json.load(sys.stdin); assert d["state"] == "healthy" and d["safe"] is True and d["lifecycle_state"] == "active"'

tmux list-panes -t navigation \
  -F 'pid=#{pane_pid} command=#{pane_current_command} start=#{pane_start_command}'
)
```

## 8. 上线验收

### 8.1 节点、状态和 QoS

```bash
(
set -Eeuo pipefail

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
timeout 5 ros2 daemon status | rg -q 'not running'

systemctl is-active --quiet ehi-gateway.service
for EXPECTED_SESSION in hdas mobiman navigation ros_discovery tools; do
  tmux has-session -t "$EXPECTED_SESSION"
done
tmux list-sessions

# 唯一 Discovery Server 所有者应为 robot_startup 的 ros_discovery tmux；旧 systemd unit
# 必须精确为 disabled/inactive/dead，且 UDP 11811 只有一个 server 进程。
test "$(systemctl show fastdds-discovery.service -p UnitFileState --value)" = disabled
test "$(systemctl show fastdds-discovery.service -p ActiveState --value)" = inactive
test "$(systemctl show fastdds-discovery.service -p SubState --value)" = dead
test "$(pgrep -fc '^/opt/ros/humble/bin/fast-discovery-server .* -p 11811( |$)' || true)" -eq 1
DISCOVERY_SS=$(ss -Hlunp)
rg -q ':11811\b.*fast-discovery-' <<<"$DISCOVERY_SS"

# robot_tf_watchdog 的前提是唯一 RSP，且该 PID 必须由含 respawn 的 eepose launch 管理。
mapfile -t RSP_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher .*__node:=robot_state_publisher( |$)' || true)
test "${#RSP_PIDS[@]}" -eq 1
RSP_PID=${RSP_PIDS[0]}
ps -o pid=,ppid=,lstart=,args= -p "$RSP_PID"

mapfile -t NAV_LAUNCH_PIDS < <(pgrep -f \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
test "${#NAV_LAUNCH_PIDS[@]}" -eq 1
NAV_LAUNCH_PID=${NAV_LAUNCH_PIDS[0]}
NAV_ENV=$(tr '\0' '\n' < "/proc/$NAV_LAUNCH_PID/environ")
printf '%s\n' "$NAV_ENV" | rg -q '^R1PRO_FASTDDS_DISCOVERY_MODE=simple$'
printf '%s\n' "$NAV_ENV" | rg -q '^FASTDDS_DEFAULT_PROFILES_FILE=.*/fastdds_robot_udp\.xml$'
! printf '%s\n' "$NAV_ENV" | rg -q '^(ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT)='
printf '%s\n' "$NAV_ENV" | \
  rg '^(R1PRO_FASTDDS_DISCOVERY_MODE|FASTDDS_DEFAULT_PROFILES_FILE)=' | sort

mapfile -t AMCL_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
test "${#AMCL_PIDS[@]}" -eq 1

EHI_PID=$(systemctl show ehi-gateway.service -p MainPID --value)
test "$EHI_PID" -gt 0
tr '\0' '\n' < "/proc/$EHI_PID/environ" | \
  rg '^(FASTDDS_DEFAULT_PROFILES_FILE|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT)=' | sort

NODE_LIST=$(timeout --signal=INT --kill-after=2 15 ros2 node list --no-daemon)
printf '%s\n' "$NODE_LIST" | rg -qx '/amcl'
printf '%s\n' "$NODE_LIST" | rg -qx '/amcl_watchdog'
printf '%s\n' "$NODE_LIST" | rg -qx '/robot_tf_watchdog'
printf '%s\n' "$NODE_LIST" | rg -qx '/web_navigation_status'

AMCL_STATE=$(timeout --signal=INT --kill-after=2 15 \
  ros2 lifecycle get /amcl --no-daemon)
printf '%s\n' "$AMCL_STATE" | rg -q '^active \[3\]$'

HEALTH_JSON=$(timeout --signal=INT --kill-after=2 15 ros2 topic echo \
  /navigation/amcl_health std_msgs/msg/String --no-daemon --once --field data \
  --full-length --no-lost-messages --qos-reliability reliable \
  --qos-durability transient_local)
HEALTH_JSON=$(printf '%s\n' "$HEALTH_JSON" | sed '/^---$/d')
printf '%s\n' "$HEALTH_JSON" | python3 -c \
  'import json,sys; d=json.load(sys.stdin); assert d["state"] == "healthy" and d["safe"] is True and d["lifecycle_state"] == "active"'
printf '%s\n' "$HEALTH_JSON"

SAFE_VALUE=$(timeout --signal=INT --kill-after=2 15 ros2 topic echo \
  /navigation/localization_safe std_msgs/msg/Bool --no-daemon --once --field data \
  --no-lost-messages --qos-reliability reliable \
  --qos-durability transient_local | sed '/^---$/d')
test "$SAFE_VALUE" = True
printf 'localization_safe=%s\n' "$SAFE_VALUE"

timeout --signal=INT --kill-after=2 15 ros2 topic echo \
  /navigation/status std_msgs/msg/String --no-daemon --once --full-length \
  --no-lost-messages --qos-reliability reliable --qos-durability transient_local

timeout --signal=INT --kill-after=2 15 ros2 topic info -v \
  /navigation/pose --no-daemon
timeout --signal=INT --kill-after=2 12 ros2 topic hz /navigation/pose || \
  test "$?" -eq 124
)
```

期望：health 为 `healthy`、safe 为 true；在连续 healthy/safe 窗口内 pose 为 8--12 Hz，
并且 pose publisher 使用 BEST_EFFORT、VOLATILE、KEEP_LAST(1)；外部 subscriber 必须
选择兼容 QoS。`waiting_scan` 等 unsafe 状态会按设计停发 pose，不能把跨 unsafe 窗口的
整体平均频率误判为 10 Hz timer 配置错误。导航 launch 环境应显示 `simple` 和
`fastdds_robot_udp.xml`，不应出现 `ROS_DISCOVERY_SERVER` 或 `ROS_SUPER_CLIENT`。EHI 的
两个 profile 变量应相同；它们可以继续显示 startup_config 下的 legacy 文件名，但该 XML
内容必须已经同步为同一 30/5、Data Sharing OFF、默认 SHM+UDP 策略。

### 8.2 TF

```bash
(
set -Eeuo pipefail

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
timeout 5 ros2 daemon status | rg -q 'not running'

capture_tf() {
  local TF_OUTPUT TF_RC=0
  TF_OUTPUT=$(timeout --signal=INT --kill-after=2 20 \
    ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1) || TF_RC=$?
  if [ "$TF_RC" -ne 0 ] && [ "$TF_RC" -ne 124 ]; then
    printf '%s\n' "$TF_OUTPUT" >&2
    return "$TF_RC"
  fi
  printf '%s\n' "$TF_OUTPUT"
  printf '%s\n' "$TF_OUTPUT" | rg -q '^At time '
}

capture_tf map body
capture_tf base_link camera_head_left_link
)
```

两条变换都应持续为当前时间。`map -> body` 不应再长期停留在旧时间，
`camera_head_left_link` 用于回归验证 robot_state_publisher watchdog 修复没有退化。

### 8.3 UDP 队列

```bash
mapfile -t AMCL_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
test "${#AMCL_PIDS[@]}" -eq 1
AMCL_PID=${AMCL_PIDS[0]}
sudo ss -uapn | rg "pid=${AMCL_PID},"
```

记录 `Recv-Q`，每分钟采一次，至少观察 30 分钟。健康 participant 的 discovery 队列
允许短时波动，但不应持续单调增长到数十 MB。用户数据带宽应另用 `ip -s link show wlan0`
和按 topic 带宽工具检查，不能用 discovery Recv-Q 代替。

## 9. AMCL 故障注入

仅在机器人静止、导航场地封闭且已有有效定位时执行。先保存 AMCL、map server 和 Nav2
PID，以及当前 `/navigation/pose`。下面先严格确认唯一 AMCL，并安排一个 30 秒自动
`SIGCONT` 逃生任务；若 watchdog 本身没有完成恢复，旧进程不会永久冻结：

```bash
inject_amcl_stop() {
local -a AMCL_PIDS
mapfile -t AMCL_PIDS < <(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
if [ "${#AMCL_PIDS[@]}" -ne 1 ]; then
  echo "ERROR: expected exactly one AMCL; found ${#AMCL_PIDS[@]}" >&2
  printf '%s\n' "${AMCL_PIDS[@]}" >&2
  return 1
fi
local AMCL_PID AMCL_START_TICKS AMCL_CMDLINE AMCL_ESCAPE_JOB
AMCL_PID=${AMCL_PIDS[0]}
AMCL_START_TICKS=$(awk '{print $22}' "/proc/$AMCL_PID/stat")
AMCL_CMDLINE=$(tr '\0' ' ' < "/proc/$AMCL_PID/cmdline")
if [[ ! "$AMCL_CMDLINE" =~ ^/opt/ros/humble/lib/nav2_amcl/amcl[[:space:]].*__node:=amcl([[:space:]]|$) ]]; then
  echo "ERROR: PID $AMCL_PID no longer has the expected AMCL cmdline" >&2
  return 1
fi

(
  sleep 30
  if [ -r "/proc/$AMCL_PID/stat" ] && [ -r "/proc/$AMCL_PID/cmdline" ]; then
    CURRENT_START_TICKS=$(awk '{print $22}' "/proc/$AMCL_PID/stat")
    CURRENT_CMDLINE=$(tr '\0' ' ' < "/proc/$AMCL_PID/cmdline")
    if [ "$CURRENT_START_TICKS" = "$AMCL_START_TICKS" ] && \
       [[ "$CURRENT_CMDLINE" =~ ^/opt/ros/humble/lib/nav2_amcl/amcl[[:space:]].*__node:=amcl([[:space:]]|$) ]]; then
      echo "SAFETY: watchdog did not replace AMCL in 30 s; sending SIGCONT to PID $AMCL_PID" >&2
      kill -CONT "$AMCL_PID"
    fi
  fi
) &
AMCL_ESCAPE_JOB=$!
disown "$AMCL_ESCAPE_JOB" 2>/dev/null || true

kill -STOP "$AMCL_PID"
echo "Paused AMCL PID $AMCL_PID; automatic SIGCONT escape job=$AMCL_ESCAPE_JOB"
}

if ! inject_amcl_stop; then
  echo 'Fault injection was not performed.' >&2
fi
unset -f inject_amcl_stop
```

SIGSTOP 后旧进程无法处理 SIGTERM，因此预期 watchdog 在确认故障后最终使用 SIGKILL。
请在另一个已经显式 source SIMPLE 环境的终端持续观察：

```bash
ros2 topic echo /navigation/amcl_health \
  --qos-reliability reliable --qos-durability transient_local
```

验收条件：

- TF 过期约 3 秒后导航输出变为 disabled，并发出一帧停止；
- 旧目标被取消，安全恢复后不会自己续跑；
- 约 8 秒确认后发送 SIGTERM，5 秒无响应再 SIGKILL；
- launch 产生新 AMCL PID，lifecycle 回到 active；
- 25 秒内恢复 stable `map -> odom`；
- 静止位姿跳变小于 0.3 m 和 10°；
- map server 与 Nav2 PID 不变；
- `/proc` 中始终不超过一个 AMCL；
- 连续执行 3 轮结果一致。

停止 scan、停止 odom 和 lifecycle deactivate 的“不误杀”测试只应在受控维护窗口进行。
这些情况下 safe 应变为 false，但 AMCL PID 不应被 watchdog 杀掉。

### 9.1 机器人端实测（2026-07-31）

机器人静止、导航输出 disabled 时，在加载本次最终 watchdog 的同一个 navigation launch
内连续进行了三轮 `SIGSTOP`。时间均从发送 `SIGSTOP` 起算：

| 轮次 | unsafe | 显式零速 | 新 AMCL PID | healthy + active | 位姿跳变 |
|---|---:|---:|---:|---:|---:|
| 1 | 3.052 s | 3.116 s | 15.724 s | 23.359 s | 0.1082 m / 4.047 deg |
| 2 | 3.041 s | 3.139 s | 15.762 s | 22.445 s | 0.0670 m / 8.045 deg |
| 3 | 3.129 s | 3.273 s | 15.866 s | 23.267 s | 0.0834 m / 0.019 deg |

三轮中 map server 与 7 个 Nav2 server PID 均未变化，`/proc` 同时出现的 AMCL 最多为
1 个；每轮最终 `localization_safe=true`、lifecycle active，输出仍为 disabled。由于注入
使用 `SIGSTOP`，旧 AMCL 无法响应 SIGTERM，三轮均按预期在 5 秒 TERM 宽限后使用
SIGKILL；累计 `restart_count=3`、`sigkill_count=3`。

同一维护窗口的其它验收结果：

- 暂停唯一 `/scan` publisher：2.428 秒进入 `waiting_scan`、2.587 秒 unsafe；恢复输入后
  1.072 秒回到 healthy，AMCL PID 保持 `266624`；
- 暂停唯一 `/odom` publisher：2.095 秒进入 `waiting_odom`、2.131 秒 unsafe；恢复输入后
  0.958 秒回到 healthy，AMCL PID 保持 `266624`；
- 显式 deactivate AMCL 后 health 为 `inactive`、safe=false，观察 7 秒未重启；原 PID
  重新 activate 后 4.147 秒回到 healthy；
- `/navigation/pose` 在 15 秒内收到 151 个样本，10.002 Hz，最大到达间隔 0.140 秒；
- `map -> body` 与 `base_link -> camera_head_left_link` 均可查询且时间持续更新；相机平移
  约为 `[0.2923, 0.0603, 1.4586]`。

上述是功能与恢复时序验收。2026-08-04 13:42 启动的 30 分钟 AMCL
discovery socket Recv-Q soak 在运行 20 分 40 秒时因用户执行整套重启而中断，
不能记为“30 分钟通过”。中断前已取得截至 1201 秒的 121 个样本，期间 AMCL
PID 固定；端口 `32936` 的 Recv-Q 仅出现一次瞬时峰值 `293632` 字节，随后归零，
其它所属 socket 的 Recv-Q 最大值为 0。用户已明确取消重做长时测试，因此本轮
只保留这份中断记录，不声称完成 30 分钟验收。

### 9.2 整机重启后复验（2026-08-04）

#### 14:08 整机重启

2026-08-04 14:08 用户实际重启机器人并关闭 VR 遥操后，已成功将旧
`fastdds-discovery.service` 切到 `UnitFileState=disabled`、`ActiveState=inactive`、
`SubState=dead`，然后重新启动 `R1PROBody` 与 navigation。下列 14:08 快照数字由
同一次运行态验收取得：

- navigation launch PID `12430`、AMCL PID `12793`、RSP PID `8208`、Discovery
  Server PID `6675`、EHI PID `1154`，各自均唯一且在短时复验内稳定；其中
  navigation/AMCL 是 14:08 历史 PID，已被后续 navigation-only 实例替换；
- AMCL lifecycle `active`、health `healthy`、safe=true；15 秒观察内收到 9 个连续
  true 安全心跳，`restart_count=0`、`sigkill_count=0`；
- `/navigation/pose` 约 9.1 Hz；`map -> body` 与
  `base_link -> camera_head_left_link` 在短时观察内都持续推进；
- AMCL 所属 6 个 UDP socket 的最终 Recv-Q 全部为 0；这是短时快照，不替代
  已取消的 30 分钟长测；
- 导航输出保持 `disabled`；VR 进程、tmux、节点和控制 endpoint/publisher 均不存在；
- 最终两包回归汇总为 78 tests，0 errors、0 failures、0 skipped。

#### 约 14:46 边界补丁 navigation-only 部署

约 14:46 将最后一轮边界条件补丁按第 7.2 节仅重启 navigation 部署，未清理 SHM。
该轮历史运行快照为：

- navigation launch PID `29906`、AMCL PID `30262`，两者都唯一；它们已被约
  15:10 的最终 navigation-only 实例替换；
- RSP PID `8208`、Discovery Server PID `6675`、EHI PID `1154` 继续沿用 14:08
  整机重启后的实例；这次 navigation-only 部署没有重启 RSP、Discovery Server
  或 EHI，也没有重启 `R1PROBody` 基础 participant；
- 最终两包回归汇总为 78 tests，0 errors、0 failures、0 skipped；
- 13:42 的 Recv-Q soak 仍是 20 分 40 秒时被整套重启中断的那次记录；
  约 14:46 的部署后没有重做或完成 30 分钟 soak，本 README 不声称该长测通过。

#### 约 15:10 cancel generation 竞态修复 navigation-only 部署

约 15:10 将第 4 节的 unsafe -> safe cancel generation 竞态修复按第 7.2 节仅重启
navigation 部署，未清理 SHM。当前最终运行快照为：

- navigation launch PID `42305`、AMCL PID `42681`、cmd bridge PID `42415`、
  Web 状态节点 PID `42417`，各自均唯一；
- RSP PID `8208`、Discovery Server PID `6675`、EHI PID `1154` 未重启，仍是 14:08
  整机重启后的基础实例；本次也没有重启 `R1PROBody` 基础 participant；
- AMCL lifecycle `active`、health `healthy`、safe=true，`restart_count=0`、
  `sigkill_count=0`；导航输出保持 `disabled`；
- `/navigation/pose` 短时采样约 9.397 Hz，末帧 stamp age 约 0.021 s，说明
  `map -> body` TF 与轻量位姿数据链的时效性正常；
- 最终 xUnit/CTest 为 78 tests，0 errors、0 failures、0 skipped；
- 用户确认机器人当前物理位置不在 `mx_map` 覆盖范围内。因此本轮只验证
  TF/数据链的时效性，不把 AMCL 全局位姿数值或 Nav2 目标可达性作为定位
  精度验收；
- 本次只做短时验收，没有重做或完成 30 分钟 Recv-Q soak，不声称长测通过。

以下 310.001 秒采样是 14:08 整机重启之前完成的高负载优化证据，保留用于
前后对比，不代表 14:08、约 14:46 或约 15:10 运行实例的 PID 或最终快照。

14:08 之前用于高负载优化的那次完整重启，同样是在关闭 VR 遥操后
重新启动 `R1PROBody` 和 navigation。与 7 月 31 日不同，当时 EHI、HDAS、mobiman、
RSP 和导航 participant 都是在最终 DDS 文件下新建的：

- EHI 运行环境中的两个 profile 变量指向同一 legacy 文件；实读 XML 为 lease 30 秒、
  announcement 5 秒、两个 participant profile 均为 Data Sharing OFF；
- navigation、AMCL 和唯一 RSP 都是 domain 102 / `rmw_fastrtps_cpp` / SIMPLE，环境中没有
  `ROS_DISCOVERY_SERVER` 或 `ROS_SUPER_CLIENT`；
- 唯一 RSP 正常，`map -> body` 和 `base_link -> camera_head_left_link` 连续 10 秒分别得到
  58 和 104 个推进的时间戳，末帧年龄分别为 0.054 秒和 0.071 秒；
- AMCL active/healthy/safe=true，导航 `ready`，输出仍为 `disabled`；VR 进程、tmux、节点
  和 `/vr_controller` endpoint 均不存在，`/cmd_vel_teleop` 没有 publisher；
- 远端 `192.168.3.5` 尚未迁移，WLAN 仍持续发送约 20 MB/s 以上，不能把本次机器人端
  实时队列优化写成“深度图带宽已消除”。

高负载优化前后的原始证据和设置见第 2.1 节。最终版本构建通过，两包测试汇总为
78 tests，0 errors、0 failures、0 skipped；最后两次部署都只重启 navigation，
未清 SHM，基础进程 PID 未因导航部署改变。

最终生产版本做了 310.001 秒并发到达时间采样：

| Topic | 频率 | 最大到达间隔 | 消息年龄均值 / 最大值 |
|---|---:|---:|---:|
| raw cloud | 9.322 Hz | 0.405 s | 0.106 / 0.265 s |
| filtered cloud | 5.019 Hz | 1.331 s | 0.343 / 1.498 s |
| `/scan` | 4.321 Hz | 1.175 s | 0.584 / 2.135 s |
| `/navigation/pose` | 9.383 Hz | 2.200 s | 0.080 / 0.772 s |

采样开始后曾因直接 `map -> odom` age 超限出现一次 2.092 秒 degraded；当时 scan age 为
0.474 秒，不是 scan 断流。随后约 305 秒持续 healthy/safe，无任何状态切换；AMCL PID
不变，`restart_count=0`、`sigkill_count=0`。pose 的 2.200 秒最大间隔包含这段按设计停发；
连续 healthy 窗口的总体频率为约 9.4 Hz。与优化前相比，`/scan` 最大间隔由 3.557 秒降到
1.175 秒，没有再次触发 `waiting_scan`。

### 9.3 已完成的 Discovery Server 单实例收敛

2026-08-04 14:08 维护窗口已成功执行：

```bash
sudo systemctl disable --now fastdds-discovery.service
```

实际状态已确认为 `UnitFileState=disabled`、`ActiveState=inactive`、
`SubState=dead`。正常监听 `0.0.0.0:11811` 的是 `ros_discovery` tmux 管理的
唯一 server，`NRestarts=0`；EHI 和基础 ROS 也已在最终 DDS 配置下重启。这项不再是未完成的
管理员动作；今后完整启动和第 8.1 节仍会以三个显式 systemd 状态作为门禁。

## 10. Discovery Server 配置

机器人安装后提供三个入口：

```text
.../config/source_fastdds_simple.sh
.../config/source_fastdds_client.sh
.../config/source_fastdds_super_client.sh
```

当前 `.bashrc` 默认：

```bash
export R1PRO_FASTDDS_DISCOVERY_MODE=simple
```

### 10.1 Discovery Server 唯一启动器

现场原来同时存在两个启动入口：

- `robot_startup.sh` 根据 `ros2_discovery.yaml` 创建 `ros_discovery` tmux，并由
  `start_discover_server.sh` 正常监听 `0.0.0.0:11811`；
- enabled 的 `/etc/systemd/system/fastdds-discovery.service` 使用
  `-x discovery_server_profile@.../fastrtps_profiles.xml`，但该 profile 不存在，导致 unit
  每 3 秒失败重启；2026-07-31 审计时 `NRestarts` 已超过 2700。

生产选择 vendor 的 `robot_startup`/tmux 作为唯一所有者。2026-08-04 14:08
已在完整维护窗口执行：

```bash
sudo systemctl disable --now fastdds-discovery.service
```

该 unit 现为 disabled/inactive/dead。不要通过向 legacy XML 随手增加同名
profile 来“修活”旧 unit；只要 tmux 入口仍在，两个
正常 server 会争抢 UDP 11811。验收必须同时满足 unit 为 disabled/inactive、精确一个
`fast-discovery-server` 进程以及 `0.0.0.0:11811` 正常监听。

### 10.2 当前生产 SIMPLE

```bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/\
r1_pro_navigation2/config/source_fastdds_simple.sh
```

它会清除 `ROS_DISCOVERY_SERVER` 和 `ROS_SUPER_CLIENT`。当前远端 Web 尚未迁移，所以生产
必须保持此模式。

EHI 的 `/etc/ehi-gateway.env` 仍可指向 factory startup_config 下的 legacy XML，以避免
修改 root-owned systemd 配置；现机已把该 XML 的核心策略同步为 30 秒 lease、5 秒
announcement、Data Sharing OFF，并移除 LARGE_DATA。该文件是安装产物，厂包重装后必须
重新检查，不能仅根据路径判断策略仍然有效。

`ROS_DDS_INTERFACE_IP=192.168.3.4` 是保留给现有启动环境的兼容元数据，Fast DDS 不会因
这个变量自动绑定网卡。机器人 XML 有意保留 builtin SHM+UDP 与可用接口，因为
`192.168.2.150` 设备网还承载传感器/DDS 流量；不能只为减少 Wi-Fi 发现流量就盲目把整机
whitelist 到 `192.168.3.4`。外部 Web XML 则明确 whitelist `192.168.3.5`。

### 10.3 双端完成后的 CLIENT

普通机器人节点：

```bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/\
r1_pro_navigation2/config/source_fastdds_client.sh
```

得到：

```text
ROS_DISCOVERY_SERVER=127.0.0.1:11811
ROS_SUPER_CLIENT=<unset>
```

两个机器人 CLIENT 入口会显式把 `R1PRO_DISCOVERY_SERVER` 覆盖为 localhost，不继承调用
shell 中可能残留的远端地址；如需其它 server，必须在 source 之后有意同时改写
`R1PRO_DISCOVERY_SERVER` 和 `ROS_DISCOVERY_SERVER`，并按整图原子切换。

诊断 shell 才使用：

```bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/\
r1_pro_navigation2/config/source_fastdds_super_client.sh
ros2 daemon stop
ros2 node list --no-daemon
```

业务节点不能设 SUPER_CLIENT，否则会重新接收完整 discovery 元数据，抵消优化。

Discovery Server 启动脚本已经在执行 server 前显式清除 CLIENT/SUPER_CLIENT 环境，
防止 server 连接自己。Discovery Server 只减少发现元数据，不提供 topic ACL，也不改变
publisher 到 subscriber 的用户数据直传路径。

该 `start_discover_server.sh` 与 EHI legacy XML 一样位于 vendor 安装目录，不受 overlay
重建保护。第 7.1 节会在停机前校验三个 `unset` 和唯一 `exec ... 11811`；厂包升级后必须
重新应用或确认补丁，不能绕过门禁。

## 11. `192.168.3.5` 迁移

本机准备包：

```text
/home/nvidia/galaxea_dev/external_web_192_168_3_5/
```

其中 `source_ros_p2p_web.sh` 已设置：

```text
ROS_DISCOVERY_SERVER=192.168.3.4:11811
ROS_SUPER_CLIENT=<unset>
lease=30 s, announcement=5 s, Data Sharing=OFF
```

不能只切其中一端。原子切换顺序：

1. 停止机器人与 Web 的全部 ROS participant；
2. 启动并确认 `192.168.3.4:11811` Discovery Server；
3. 设置 `R1PRO_FASTDDS_DISCOVERY_MODE=client` 后启动整套机器人 ROS；
4. 在 `192.168.3.5` source 新脚本，再启动 Web ROS 进程；
5. EHI 若仍参与 ROS domain，在 `/etc/ehi-gateway.env` 同步加入
   `ROS_DISCOVERY_SERVER=127.0.0.1:11811` 并重启；
6. 使用独立 SUPER_CLIENT shell 检查完整 ROS 图。

远端源码预期位于：

```text
/home/motrix/galaxea/galaxea_overlay/src/r1pro_web_control/
  r1pro_web_control/server_node.py
  r1pro_web_control/vision_worker.py
```

远端后续代码迁移要求：

- 导航主节点改订 `/navigation/pose`；
- 删除导航 participant 的 `/tf`、`/tf_static`、map updates、全局/局部 costmap 和 path
  常驻订阅；
- 地图若 UI 确实需要，使用单独可选订阅和 KEEP_LAST(1)；
- 相机、raw depth、PointCloud2 只在对应页面/视觉任务活动时创建订阅；
- `vision_worker.py` 当前需要 `robot_frame <- camera_frame` 做深度反投影，不能直接删 TF；
  应先拆成独立可选 participant，长期把坐标变换搬到机器人端，只向外发 body/map 下的
  object pose。

在远端修改前，机器人侧新增 `/navigation/pose` 不会自动让旧 Web 停止订阅 `/tf`，也
不会消除两路 raw depth 的带宽。本次 README 不把这部分标记为已完成。

## 12. 回滚

### DDS 回滚到 SIMPLE

先为操作终端显式准备 SIMPLE 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/\
r1_pro_navigation2/config/source_fastdds_simple.sh
ros2 daemon stop
```

上述命令只改变当前 shell 和后续 CLI participant，不会转换任何已运行 participant。
回滚时仍必须先让机器人静止，并按第 7.1 节原子重启机器人全部 ROS 与 EHI；远端同时清除
`ROS_DISCOVERY_SERVER`/`ROS_SUPER_CLIENT` 后重启 Web。不要让一端 CLIENT、另一端 SIMPLE
长期混用，否则双方无法发现。

### 定位恢复异常

- 先通过 `/navigation/amcl_health` 读取具体拒绝原因，不要手工反复 kill；
- `needs_initial_pose` 时重新发布可信 `/initialpose` 或执行全局定位；
- PID 不唯一时停止重复的 navigation launch，只保留一个 AMCL；
- 输入 stale 时先修复 `/scan`、`/odom` 或地图，不应绕过 watchdog；
- 如需紧急恢复旧版本，停止导航后重新构建此前备份版本，再整套重启；不要只杀
  `amcl_watchdog`，因为安全心跳超时会按设计保持导航 disabled。

## 13. 已知边界

- watchdog 能恢复独立 AMCL participant 卡死，不能修复整个内核、全进程调度或所有
  Fast DDS participant 同时失效；
- Discovery Server 当前只有一台，已有匹配 endpoint 在 server 故障后通常仍能通信，
  但新节点和 respawn 节点不能重新发现；
- Discovery Server 不是安全边界或 topic 白名单；严格隔离需要独立 ROS domain 加桥、
  DDS Router/Zenoh，或只向外提供带 whitelist 的 WebSocket/HTTP gateway；
- RSP respawn/15 Hz、joint tracker 禁止重复 RSP、Discovery Server 环境清理和 EHI legacy
  XML 都是本机 vendor 安装产物补丁；当前工作空间没有 `mobiman` 源码，厂包升级后必须先
  通过第 7.1 节内容门禁再启动；
- self-filter 最终仍通过 TF Buffer 查询 `body <- livox_frame`。本次曾验证“首次查询后缓存
  并退订动态 `/tf`”方案，但冷启动时 filtered/scan 均不出流，因此已完整回退，最终源码和
  launch 不含该参数；不要在未做冷启动门禁的情况下重新引入此优化；
- `/home/nvidia/galaxea` 没有可用 Git 元数据，`install.bak` 只能作为本机厂包差异证据，
  不能替代可提交、可审计的源码历史；
- 本轮没有扩大 socket buffer，也没有删除用户原有地图、视觉或控制功能。
