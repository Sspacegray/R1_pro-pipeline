# 头部深度分拣 Workflow 标定与闭环 TODO

适用 workflow：`head_depth_sorting`

实现文件：

- `src/r1pro_web_control/r1pro_web_control/code_workflows/head_depth_sorting.py`
- `src/r1pro_web_control/r1pro_web_control/code_workflows/head_left_bottle_grasp.py`
- `src/r1pro_web_control/r1pro_web_control/code_workflows/dual_box_pick_and_place.py`

深度来源为头部双目 `Fast-FoundationStereo`，不是 FoundationPose 的物体 6D 位姿估计。

## 0. 安全前提

- [ ] 首次运行保持 `dry_run=true`。
- [ ] 实机运行前确认 token、控制模式和急停可用。
- [ ] 按单工件、双工件、三工件逐级验证。
- [ ] 标定未通过时，不执行双臂搬盆和 Nav2 回航。

## 1. 头部 RGB-D 与 TF

相关 topic：

```text
RGB:        /r1pro_web/camera_head/left/rectified/image_raw/compressed
Depth:      /r1pro_web/camera_head/left/rectified/aligned_depth/image_raw
CameraInfo: /r1pro_web/camera_head/left/rectified/camera_info
状态:        /r1pro_web/camera_head/depth_processor/status
相机 frame:  camera_head_left_rectified_optical
机器人:      base_link
```

- [ ] `head_depth_processor/status` 为 `ready`。
- [ ] 左右头部原始图像持续发布并能正确时间配对。
- [ ] `CameraInfo` 的 `K/D/width/height` 正常。
- [ ] `depth_scale_m` 和深度单位确认是米。
- [ ] `base_link <- camera_head_left_rectified_optical` TF 可查询。
- [ ] RGB/Depth 时间差不超过 `max_rgb_depth_delta_s`。
- [ ] 记录深度频率、延迟和无效深度比例。

验证命令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /r1pro_web/camera_head/depth_processor/status --once
ros2 topic hz /r1pro_web/camera_head/left/rectified/aligned_depth/image_raw
ros2 run tf2_ros tf2_echo base_link camera_head_left_rectified_optical
```

## 2. 深度到 `base_link` 的几何标定

- [ ] 在实际桌面布置 5×5 或 6×6 个已知三维点。
- [ ] 每个点记录像素 `(u,v)`、深度、真实 `base_link XYZ`。
- [ ] 对比程序计算的 `camera_xyz_m` 和 `robot_xyz_m`。
- [ ] 验证或重新拟合头部坐标补偿矩阵。
- [ ] 记录矩阵版本，例如 `checker_6x6_<date>_v1`。

建议验收：XY/Z 平均误差均不超过 10 mm，最大误差不超过 20 mm。

现有补偿矩阵位置：

`src/r1pro_web_control/r1pro_web_control/code_workflows/head_left_bottle_grasp.py`

## 3. YOLO 类别与目标选择

- [ ] 确认模型实际输出的工件类别名称。
- [ ] 设置 `prompt` 为实际类别，例如 `workpiece`、`part` 或 `component`。
- [ ] 确认料盆不会被工件 prompt 误识别。
- [ ] 确认 mask 覆盖工件主体而不是大面积桌面。
- [ ] 验证 `confidence_threshold`。
- [ ] 验证多目标时选择最高置信度且 `position_status=ok` 的目标。

## 4. 单工件右臂抓取标定

重点参数：

```text
target_offset_m
tool_tip_offset_m
pregrasp_backoff_m
grasp_rpy_deg / approach_axis_base
depth_window_px
allow_torso_ik
```

- [ ] 在桌面近/中/远、左/中/右位置分别放置工件。
- [ ] 记录 YOLO 像素、深度、`robot_xyz_m` 和实际夹爪中心。
- [ ] 调整 `target_offset_m` 使夹爪落在有效抓取区域。
- [ ] 调整 `tool_tip_offset_m` 使闭合中心与工件中心一致。
- [ ] 调整抓取姿态、水平接近方向和 `pregrasp_backoff_m`。
- [ ] 验证夹爪打开值、闭合值、渐变时间和稳定等待时间。
- [ ] 低速成功抓取至少 5 次。

## 5. 工件落入源料盆标定

落盆高度计算为：

```text
drop_z = tray_box_z_m + tray_box_size_z_m + object_drop_clearance_m
```

重点参数：

```text
object_drop_clearance_m
object_drop_pregrasp_z_offset_m
object_drop_retreat_z_offset_m
object_drop_rpy_deg
```

- [ ] 测量盆口、内壁高度和最大工件尺寸。
- [ ] 确认释放点在盆口有效区域中心。
- [ ] 确认下降和退出过程不碰盆壁。
- [ ] 分别测试最小、最大和不同姿态工件。
- [ ] 单个工件连续落盆成功至少 5 次。

## 6. 源料盆双臂抓取标定

需要测量：

```text
tray_box_x_m, tray_box_y_m, tray_box_z_m
tray_box_roll_rad, tray_box_pitch_rad, tray_box_yaw_rad
tray_box_size_x_m, tray_box_size_y_m, tray_box_size_z_m
basket_grasp_side_axis
basket_grasp_local_x_m, basket_grasp_local_y_m, basket_grasp_local_z_m
gripper_center_offset_z_m
```

- [ ] 测量源料盆中心、姿态和尺寸。
- [ ] 标定左、右夹爪相对料盆的抓取点。
- [ ] 执行双臂全路径 IK 预检查。
- [ ] 空盆抬升后确认不倾斜、不滑落。
- [ ] 装入 1、2、3 个工件后分别验证仍能稳定搬运。

## 7. 目标放置位置标定

重点参数：

```text
tray_place_x_m, tray_place_y_m, tray_place_z_m
tray_place_roll_rad, tray_place_pitch_rad, tray_place_yaw_rad
tray_place_pregrasp_z_offset_m
tray_place_retreat_z_offset_m
tray_place_clearance_m
tray_carry_z_m
```

- [ ] 测量目标支撑面中心、高度和姿态。
- [ ] 验证搬运安全高度和全路径 IK。
- [ ] 验证空盆、半满盆、满盆的重心变化不会导致失败。

注意：当前 `tray_place_*` 是 `base_link` 固定坐标，不是 `map` 坐标。如果目标在另一地图区域，需要增加“先 Nav2 到目标区域，再执行双臂放置”的流程。

## 8. Nav2 返回原点

当前默认目标：

```text
frame: map
x: 0.0
y: 0.0
yaw: 0.0
```

- [ ] 确认 map 原点就是人工补料点。
- [ ] 确认 AMCL 在补料点附近稳定。
- [ ] 确认 `map -> odom -> base_link` TF 正常。
- [ ] 连续回航至少 5 次并记录最终误差。
- [ ] 验收位置误差 <= 5 cm、角度误差 <= 5 deg、Nav2 状态为 `succeeded`。

## 9. 真正闭环所需的反馈

当前 workflow 主要依据 IK 和动作接口返回值判断成功，还需要增加至少一种实际反馈。

### 抓取成功

- [ ] 夹爪闭合宽度、力或电流反馈。
- [ ] 抓取后桌面工件从 YOLO 检测中消失。
- [ ] 抓取后工件出现在夹爪区域。

### 落盆成功

- [ ] 料盆 ROI 中检测到工件。
- [ ] 桌面目标数量减少、料盆目标数量增加。
- [ ] 释放后夹爪区域没有残留工件。

### 失败恢复

- [ ] 目标仍在桌面：重新检测并重试。
- [ ] 夹爪闭合但无工件：回观察姿态后重试或跳过。
- [ ] 工件未进入料盆：停止搬盆并报警。
- [ ] 双臂料盆抓取失败：禁止 Nav2。
- [ ] Nav2 回航失败：禁止进入下一批。

## 10. 端到端验收顺序

- [ ] `dry_run=true` 检查所有位姿和 IK 计划。
- [ ] 验证头部深度和 `position_status=ok`。
- [ ] 单独验证右臂抓取一个工件。
- [ ] 单独验证一个工件落盆。
- [ ] 单独验证空盆双臂抓取、搬运和放置。
- [ ] 单独验证 Nav2 回 `(0,0,0)`。
- [ ] 完成 1 个工件端到端流程。
- [ ] 完成 2 个工件端到端流程。
- [ ] 完成 3 个工件端到端流程。
- [ ] 验证连续无目标时不搬运空盆。
- [ ] 验证人工重新放置空盆和物料后可以开始下一批。

## 11. 建议保存的标定文件字段

后续建议保存为独立 YAML，而不是长期只在 Web 中手动输入：

```yaml
camera:
  depth_scale_m: 1.0
  camera_frame: camera_head_left_rectified_optical
  base_frame: base_link
  compensation_matrix: []
object:
  prompt: workpiece
  target_offset_m: [0.0, 0.0, 0.0]
  tool_tip_offset_m: 0.04
  pregrasp_backoff_m: 0.06
  grasp_rpy_deg: []
  drop_rpy_deg: []
source_tray:
  pose_xyz_m: [0.280968, 0.00000035, 0.94227]
  pose_rpy_rad: [0.0, 0.0, 1.5708]
  size_m: [0.60, 0.40, 0.12]
  grasp_local_m: []
destination_tray:
  pose_xyz_m: [0.8, -0.2, 0.5]
  pose_rpy_rad: [0.0, 0.0, 1.5708]
navigation:
  return_map_pose: [0.0, 0.0, 0.0]
verification:
  max_objects_per_batch: 3
  no_detection_confirmations: 2
  grasp_retry_count: 2
```


## 12. 钩子搬运专用标定与反馈（新增）

钩子 workflow：

    dual_hook_pick_and_place

实现文件：

src/r1pro_web_control/r1pro_web_control/code_workflows/dual_hook_pick_and_place.py

当前实现假设钩子是固定安装在左右末端的被动工具，不发送夹爪开合动作。

### 12.1 钩子 TCP 标定

左右钩子必须分别标定，不要直接复用夹爪 TCP。

需要得到：

    hook_left_tip_offset_xyz_m
    hook_right_tip_offset_xyz_m
    hook_left_rpy_deg
    hook_right_rpy_deg

标定方法：

1. 将钩子刚性安装到左右末端。
2. 在钩尖贴一个明显标记点。
3. 让机器人从至少 4 个不同姿态接近同一个固定标记点。
4. 用示教器或视觉记录每次末端位姿。
5. 求解末端控制点到钩尖的固定变换。
6. 将左、右结果分别写入 workflow 参数。
7. 用未参与拟合的第 5 个点验证钩尖误差。

验收：

    钩尖位置误差 <= 5 mm
    钩尖姿态误差 <= 2 deg

### 12.2 箱体挂入点标定

需要在箱体坐标系中测量：

    hook_source_left_local_xyz_m
    hook_source_right_local_xyz_m
    hook_place_left_local_xyz_m
    hook_place_right_local_xyz_m

测量方法：

1. 定义箱体中心为 box frame 原点。
2. 测量左右把手、孔、箱沿或可挂接边缘的中心点。
3. 测量挂钩方向、有效插入长度和允许侧向间隙。
4. 确认钩子不会碰到箱体内壁或加强筋。
5. 分别填写源位置和目标放置位置的挂入点。
6. 用空箱验证左右钩子能够同时进入。

验收：

    左右钩尖均进入有效挂接区域
    钩子与箱体不发生硬碰撞
    左右受力方向基本对称

### 12.3 预挂钩、插入、试抬升轨迹

需要标定：

    hook_pregrasp_offset_xyz_m
    hook_test_lift_offset_xyz_m
    hook_lift_offset_xyz_m
    hook_engage_waypoint_count
    hook_test_waypoint_count
    hook_lift_waypoint_count

推荐步骤：

    预挂钩点
    -> 低速插入
    -> 到达挂接点
    -> 试抬升 10~20 mm
    -> 确认挂住
    -> 抬升到搬运高度

标定方法：

1. 先不挂箱体，执行预挂钩到插入轨迹。
2. 调整插入偏移，确保钩子不碰箱体。
3. 挂空箱执行试抬升。
4. 逐步增加试抬升高度，直到箱体稳定跟随。
5. 记录不滑脱的最小抬升高度和安全抬升高度。
6. 再用半载和满载箱体重复验证。

### 12.4 钩子成功反馈

workflow 支持三种模式：

    effort_delta  推荐
    operator      仅用于人工标定或没有 effort 反馈时
    motion_only   不推荐，只能作为受控测试

#### effort_delta

当前实现会：

1. 挂入后记录左右臂 effort。
2. 执行小距离试抬升。
3. 再次读取左右臂 effort。
4. 计算最大 effort 差值。
5. 差值小于 hook_min_effort_delta 时停止，不继续搬运。

需要标定：

    hook_min_effort_delta
    hook_feedback_timeout_s

标定方法：

1. 记录未挂箱、挂空箱、挂半载、挂满载四组 effort。
2. 每组至少重复 5 次。
3. 计算“未挂箱”和“已挂箱”的差值分布。
4. 阈值设置在未挂箱最大值和已挂箱最小值之间，并保留安全余量。
5. 确认左右臂 effort 数组都有效且时间戳新鲜。

注意：effort 差值是负载证据，不是绝对的挂钩证明；仍应结合箱体位移或视觉验证。

#### operator

只有在人工已经确认钩子挂住时才允许：

    hook_feedback_mode=operator
    hook_engagement_confirmed=true

该模式不是真正的自动闭环，只适合没有力/电流反馈的初期标定。

#### motion_only

必须同时设置：

    allow_unverified_hook=true

该模式只确认机械臂运动到位，不能证明箱体已挂住。禁止用于满载正式运行。

### 12.5 放置、释放和脱钩

需要标定：

    hook_release_offset_xyz_m
    hook_disengage_offset_left_xyz_m
    hook_disengage_offset_right_xyz_m
    hook_retreat_offset_xyz_m
    hook_release_waypoint_count
    hook_disengage_waypoint_count
    hook_retreat_waypoint_count

推荐步骤：

    搬运到目标上方
    -> 下降到箱体由目标平面承重
    -> 沿释放偏移降低/放松
    -> 左右钩子沿各自脱钩方向退出
    -> 上升退出

验收：

- [ ] 箱体由目标平面承重后，钩子才开始退出。
- [ ] 脱钩时箱体不被拖动。
- [ ] 左右钩子都完全离开箱体。
- [ ] 退出轨迹无碰撞。
- [ ] 释放后箱体位置误差不超过 20 mm。

### 12.6 钩子负载和安全数据

必须记录：

    空箱质量
    半载质量
    满载质量
    箱体重心
    钩子额定载荷
    钩子与箱沿的最小安全厚度
    最大搬运加速度
    最大允许底盘速度

建议初始安全策略：

- [ ] 先空箱。
- [ ] 再半载。
- [ ] 最后满载。
- [ ] 搬运过程中禁用急转弯和急停式速度变化。
- [ ] 任何 effort 异常、箱体滑移或钩子脱离立即停止。

### 12.7 钩子 workflow 实机验收顺序

- [ ] Web 重新加载后能看到“**双臂钩子固定料盆抓取并放置**”。
- [ ] dry_run=true 检查预挂钩、试抬升、搬运、释放、脱钩和 IK。
- [ ] hook_calibration_confirmed=false 时真实运行应被拒绝。
- [ ] 无箱体执行预挂钩和脱钩轨迹。
- [ ] 空箱执行挂入和 10~20 mm 试抬升。
- [ ] 验证 effort 或人工反馈能正确区分挂住/未挂住。
- [ ] 空箱完成一次搬运和放置。
- [ ] 半载重复 3 次。
- [ ] 满载重复 3 次。
- [ ] 失败反馈出现时不得进入搬运阶段。

### 12.8 交互式 effort 标定程序

已新增只读标定程序：

    calibration/hook_effort_calibration.py

实际 ROS 2 模块位于：

    src/r1pro_web_control/r1pro_web_control/calibration/hook_effort_calibration.py

它只订阅左右臂 `sensor_msgs/msg/JointState` 的 `effort`，不会发送机械臂、夹爪、底盘或导航
指令。程序会依次引导以下四种工况：

    no_load     只挂钩子，不挂料盆
    empty_tray  挂空料盆
    half_load   挂半载料盆
    full_load   挂满载料盆

每种工况默认重复 5 次。每次流程为：

1. 操作员把机器人恢复到统一的预抬升姿态。
2. 程序自动采集抬升前窗口内的左右臂 effort 中值。
3. 操作员手动低速抬升 10～20 mm 并保持，按 Enter。
4. 程序自动采集抬升后窗口内的 effort 中值。
5. 程序计算左右臂以及双臂最大 effort 差值。
6. 全部工况结束后，使用未挂载差值的 P95 和挂载差值的 P05 计算推荐阈值。

在容器中运行：

    source /opt/ros/humble/setup.bash
    source /home/pc10/r1pro_web_colcon/install/setup.bash
    python3 calibration/hook_effort_calibration.py

或者使用 ROS 2 安装入口：

    ros2 run r1pro_web_control r1pro_hook_effort_calibrate

默认输出：

    runtime_data/r1pro_web_control/hook_effort_calibration.yaml

常用参数：

    ros2 run r1pro_web_control r1pro_hook_effort_calibrate \
      --repeats 5 --window-s 2.0 \
      --output runtime_data/r1pro_web_control/hook_effort_calibration.yaml

程序输出的 `workflow_params` 可直接填入钩子 workflow：

    hook_feedback_mode: effort_delta
    hook_min_effort_delta: <程序推荐值>
    hook_feedback_timeout_s: <程序推荐值>

如果程序报告“未挂载与挂载 effort 差值没有形成安全分离”，不得强行使用该阈值。应检查
钩子是否真正承重、试抬升高度是否一致、左右臂是否处于同一姿态，并重新采样。

### 12.9 挂载料盆后的底盘导航

钩子挂住并试抬升确认后，workflow 可以保持双臂承重，执行 Web 中保存的两个
`navigate_to_pose` 原子动作：

    hook_nav_to_place_action_name  目标放置点
    hook_nav_to_home_action_name   返回初始点

流程为：挂钩/试抬升 -> 目标点导航 -> 底盘稳定 -> 双臂放置/脱钩 -> 返回初始点导航。
导航动作名必须来自 Web 的保存动作，不能只填写坐标或依赖默认 `(0,0,0)`。
标定向导会列出动作库中的导航动作，并把动作名写入完整报告；报告中这两个字段为空时，
`calibration_complete` 必须保持为 false，live runner 拒绝执行。

### 12.10 标定结果自动载入与 Web 保存

完整报告默认路径为：

    runtime_data/r1pro_web_control/hook_transport_calibration.yaml

`dual_hook_pick_and_place` 和 `head_depth_sorting(tray_tool_mode=hook)` 会自动载入
该报告的 `workflow_params`。报告中的几何、TCP、轨迹、effort 阈值、固定躯干和两条
导航动作名具有优先级，Web 表单里未重新标定的默认值不会覆盖它们；`dry_run` 和
`motion_confirmed` 仍由本次运行控制。报告不完整时只允许 dry-run。

直接从 Web 运行时还必须勾选“已确认现场安全并允许真实运动”；它是默认关闭的
后端安全门，不属于标定数据。分拣 workflow 在连续无目标返回时会主动关闭 YOLO，
避免下一次运行继承旧 worker。

如果报告不在默认位置，在 Web 中填写“完整钩子标定报告路径”（分拣 workflow 对应
“钩子完整标定报告路径”），再点击“另存为原子动作”。保存后的 `code_workflow`
动作会携带这些参数，加入总 workflow 后仍然读取同一份报告，不需要再手工编辑
Python 或粘贴大段 JSON。


## 13. 分拣 workflow 切换到钩子

head_depth_sorting 新增：

    tray_tool_mode=gripper 或 hook
    tray_hook_params_json={...}

说明：

- 默认仍为 gripper，保证原有流程兼容。
- 设置 tray_tool_mode=hook 后，料盆搬运阶段调用 dual_hook_pick_and_place。
- 只需填写 `tray_hook_calibration_report_path` 指向完整标定报告；报告会自动提供
  左右钩子 TCP、挂入点、反馈模式、waypoint 和两条导航动作名。
- `tray_hook_params_json` 仅作为临时覆盖入口，通常保持 `{}`，不再需要粘贴整份标定参数。
- Web 前端会根据 params_schema 显示报告路径，不需要新增专用 HTML 控件。

钩子模式示例：

    tray_tool_mode=hook
    tray_hook_params_json={
      "hook_calibration_confirmed": false,
      "hook_feedback_mode": "effort_delta",
      "hook_source_left_local_xyz_m": [0.30, 0.0, 0.06],
      "hook_source_right_local_xyz_m": [-0.30, 0.0, 0.06],
      "hook_place_left_local_xyz_m": [0.30, 0.0, 0.06],
      "hook_place_right_local_xyz_m": [-0.30, 0.0, 0.06]
    }

未完成 TCP 和挂入点标定前，保持 hook_calibration_confirmed=false。

## 14. 导航重复测试记录

导航测试必须先满足：

- [ ] navigation_ready=true。
- [ ] localized=true。
- [ ] map_ready=true。
- [ ] nav_output_state=armed_idle。
- [ ] Nav2 action server、/navigation/status、/amcl_pose 和底盘反馈均在线。
- [ ] 钩子搬运已通过空载挂入和试抬升测试。

每次测试记录：

    日期时间
    起点 map pose
    目标 map pose
    Nav2 action 状态
    最终 x/y/yaw 误差
    planner/controller 状态
    底盘最终速度
    是否有钩子滑移或脱钩

重复测试顺序：

- [ ] 只读检查 3 次导航状态，确认状态稳定。
- [ ] 无载荷导航到回航点 3 次。
- [ ] 空箱挂钩后导航回航 3 次。
- [ ] 半载料盆导航回航 3 次。
- [ ] 满载料盆只在前述测试全部通过后执行 3 次。
- [ ] 任一次导航失败、定位丢失、钩子 effort 异常或箱体滑移，立即停止后续测试。

当前已知运行时状态（本次检查）：

    navigation_ready=false
    localized=false
    map_ready=true
    nav_output_state=armed_idle
    service_available=true
    /navigation/status=SUCCEEDED（历史目标）
    head_depth processor=stopped

注意：`/navigation/status` 当前只报告历史 `SUCCEEDED`、定位状态和控制器状态，没有提供 Web 所需的
`navigation_ready=true` / `localization_ready=true` 字段。因此 Web 的安全门仍保持关闭；历史
`SUCCEEDED` 不能作为新的导航目标成功证明。

### 14.1 2026-09-28 实测记录

- [x] 已将源码构建到 `/home/pc10/r1pro_web_colcon/install`，运行 Web 使用该 overlay。
- [x] 已调用 `POST /api/code_workflows/reload`，返回 13 个 workflow，包含
  `dual_hook_pick_and_place`，参数 schema 数量为 29；前端采用 metadata 自动生成参数表单，
  不需要新增专用 HTML/JS 控件。
- [x] 钩子 workflow 本地 dry-run 重复 5 次：计划生成成功，`passive_hook`，全路径共 78 个
  预检查 waypoint；没有夹爪开合动作。
- [x] 通过 Web API 以 `dry_run=true`、`validate_ik=false` 重复 3 次：均返回 workflow complete，
  仅计算计划，不发送机械臂或底盘动作。
- [x] 通过 Web API 以默认 `validate_ik=true` 运行 1 次：安全拒绝，原因是
  `left waypoint 1 joint jump 136.9 deg exceeds 15.0 deg`。这说明当前默认钩子 TCP、挂入点或
  姿态尚未按实物标定，不能进入真实运动；完成第 12 节标定后必须重新通过 IK 预检查。
- [x] 连续采样 Web 导航状态 3 次，结果一致：
  `navigation_ready=false`、`localized=false`、`map_ready=true`、`nav_output_state=armed_idle`、
  `service_available=true`，历史 `/navigation/status=SUCCEEDED`。
- [x] 连续向 `POST /api/navigation/goal` 发送 3 次 `(0,0,0)` 测试请求；三次均返回
  `navigation is not ready according to /navigation/status`，没有发布新的导航目标。
- [ ] 尚未完成真实钩子搬运和导航重复测试。完成条件是：补齐钩子 TCP/挂入点/姿态/脱钩偏移，
  使 `navigation_ready=true` 且 `localized=true`，先空载再空箱、半载、满载按第 14 节顺序执行。

因此当前不能宣称已经完成真实钩子搬运导航重复测试；已完成的是 workflow、Web 元数据、计划 dry-run、
IK 安全阻断和导航安全门重复验证。
