# R1Pro ArUco Fine Approach

这个包用于导航停到定点后的底盘微调。节点订阅头部左相机识别出的 ArUco id=1 在
`base_link` 下的位姿，只用 `x/y` 误差做是否到位判断，`z` 不参与闭环。

## Topics

- Subscribe: `/r1pro_aruco_box_pose/marker_pose_base_link_1`
  (`geometry_msgs/msg/PoseStamped`)
- Subscribe: `/r1pro_aruco_fine_approach/enable` (`std_msgs/msg/Bool`)
- Publish: `/motion_target/target_speed_chassis`
  (`geometry_msgs/msg/TwistStamped`)
- Publish: `/r1pro_aruco_fine_approach/in_tolerance` (`std_msgs/msg/Bool`)
- Publish: `/r1pro_aruco_fine_approach/status` (`std_msgs/msg/String`)

## Run

```bash
cd /home/motrix/galaxea/galaxea_overlay
colcon build --packages-select r1pro_aruco_fine_approach
source install/setup.bash
ros2 launch r1pro_aruco_fine_approach aruco_fine_approach.launch.py
```

导航停稳后打开微调：

```bash
ros2 topic pub --once /r1pro_aruco_fine_approach/enable std_msgs/msg/Bool "{data: true}"
```

手动停止：

```bash
ros2 topic pub --once /r1pro_aruco_fine_approach/enable std_msgs/msg/Bool "{data: false}"
```

## Tuning

默认目标是 `x=0.616m, y=0.007m`，对应当前日志里 `base_link` 下 id=1 的可见稳定位置。
`z` 是高度，不参与微调。默认只开 `x/y` 平移微调，yaw 控制保持关闭，
避免现场符号未确认时把 ArUco 转出视野。现场可以改
`config/aruco_fine_approach.yaml` 里的 `x_tolerance_m/y_tolerance_m`、
`kp_x/kp_y/kp_yaw`、速度限幅和 `x_command_sign/y_command_sign/yaw_command_sign`。
`max_start_error_x_m/max_start_error_y_m` 是启动保险，首帧误差超过这个范围时节点只发零速，
并输出 `initial_error_too_large`，用于防止目标坐标填错后继续盲动。
如果手动确认角速度方向正确，再把 `enable_yaw_control` 改成 `true`。若要改用箱体位姿反馈，把 `pose_topic` 改成：

```yaml
pose_topic: /r1pro_aruco_box_pose/box_pose_base_link_1
```
