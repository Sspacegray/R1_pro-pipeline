### indoor_location说明：

1 cloud_process：揽沃雷达mid_360以及rs16数据处理
发布的话题：/deskew_cloud，数据类型 sensor_msgs::msg::PointCloud2
2 nano_flann：用来存储全局pcd地图的kdtree树数据结构，提高匹配效率
3 ndt：三维点云匹配库
4 p2pl_icp：三维点面匹配算法
5 utility：定义基础函数类型
6 location_livox_p2pl_icp.cpp：揽沃mid_360定位总程序,使用点面匹配算法p2pl_icp，location_livox_ndt.cpp 揽沃mid_360定位总程序,使用点面匹配算法ndt

2 ###功能说明

1 支持手动重定位功能，rviz2中点2D Pose Estimate就可以给定重定位位姿

2 支持雷达角度距离数据裁剪、以及机械雷达rs16去畸变(慎重使用，特别是大场景点云多的话容易影响定位效率)

2 发布/agv_pose位姿话题以及map-baselink的tf变换

3 支持 BBS 分支定界全局重定位，重定位成功后继续使用现有 3D ICP/NDT 前端做精配准

4 支持断电前自动记录稳定定位位姿，重启时优先恢复，并在恢复失败时自动回退到全局重定位

   位姿默认保存到 `~/.local/state/indoor_location/`，持久化实现使用“临时文件 + fsync + 原子 rename”，比 `/tmp` 更适合整机断电后的恢复

   落盘条件支持“最小时间周期 + 位姿变化阈值”双条件，并带有静止时的最长强制落盘周期，避免静止机器人长期不刷新恢复位姿

5 支持使用轮式里程计 `odom->base_link` TF 为正常跟踪提供匹配初值先验

6 发布 `/localization_status` 实时定位状态，便于上层监控 `GLOBAL_RELOCALIZING / TRACKING_OK / TRACKING_WEAK / LOST`

3 需要更高精度的话可以调整参数体素地图、点云范围、点云体素、匹配迭代次数

2 ####运行说明：
ros2 launch indoor_location run_rs16.launch.py  #速腾雷达rs16，搭载wit的imu；
ros2 launch indoor_location run_livox_mid360.launch.py #ea100搭载livox_mid360雷达；
ros2 launch indoor_location run_qr100_livox_mid360.launch.py #qr100搭载livox_mid360雷达
对于大场景使用ndt做前端，只需要在CMakeLists.txt修改location_ndt.cpp，location_p2pl_icp.cpp程序，启动不同雷达的yaml文件；

总结：对于30米左右的距离雷达，小范围使用的话可以使用p2picp；对于大环境得使用距离更远80米以上雷达，使用ndt;当然后续也可以使用ndt做粗匹配，p2picp做细匹配来增加鲁棒性，但是有带来一个问题eskf位姿出来得的耗时会更大，为了更好的兼容性会持续更新

#### 位姿保存恢复参数说明与调参建议

- `enable_pose_restore`
  含义：节点启动时是否优先读取上次保存位姿做局部验证。
  建议：需要断电恢复时保持 `true`；只有在你明确不想复用历史位姿时再关掉。

- `pose_save_only_when_stable`
  含义：只在稳定定位状态下保存，避免把漂移中的位姿写进恢复文件。
  建议：默认保持 `true`。只有调试阶段想观察所有状态都落盘时才考虑关闭。

- `pose_save_interval_sec`
  含义：最小保存时间间隔。到这个时间后，还要再满足“位姿变化超过阈值”才会落盘。
  调参：
  `0.5s`：更积极，断电时丢失更少，但写盘更频繁。
  `1.0s`：推荐默认值，兼顾恢复效果和写盘次数。
  `2.0s` 以上：更省写盘，但断电时恢复位姿可能比真实位置落后更多。

- `pose_save_max_interval_sec`
  含义：最长强制保存时间。机器人长时间静止时，即使位姿几乎不变，也会按这个周期刷新一次恢复位姿。
  调参：
  机器人经常原地待机：建议 `3~10s`。
  如果希望静止时也更快刷新最新时间戳：调小到 `3~5s`。
  如果更在意减少写盘：可以放宽到 `10~30s`。

- `pose_save_min_translation_m`
  含义：相对上次保存位姿的最小平移变化阈值。
  调参：
  轨迹细、速度慢、希望更频繁记录：`0.1~0.2m`。
  一般室内轮式机器人：`0.2~0.5m`。
  如果发现文件更新过于频繁：适当调大。
  如果发现断电后恢复位置经常落后太多：适当调小。

- `pose_save_min_yaw_deg`
  含义：相对上次保存位姿的最小偏航变化阈值。
  调参：
  原地转向频繁：`3~8deg` 更合适。
  一般移动平台：`5~15deg`。
  如果机器人经常只是转头不平移，而你希望也保存这些姿态变化，就把它调小。

- `pose_persistence_path`
  含义：恢复位姿文件存放路径。
  建议：使用持久目录，例如 `~/.local/state/indoor_location/...`，不要放在 `/tmp`，否则整机重启后可能丢失。

#### 保存策略怎么理解

当前保存判定不是“纯定时器”，而是下面这套组合：

```text
满足稳定定位
-> 到达最小时间间隔 pose_save_interval_sec
-> 并且平移变化 >= pose_save_min_translation_m 或 偏航变化 >= pose_save_min_yaw_deg
-> 才真正落盘

如果机器人长时间静止
-> 到达 pose_save_max_interval_sec
-> 即使没动也会强制落盘一次
```

#### 现场调参建议

- 想更像“断电恢复”：
  把 `pose_save_interval_sec` 调小到 `0.5`，把 `pose_save_max_interval_sec` 调到 `3~5`。

- 想减少写盘次数：
  把 `pose_save_interval_sec` 调大到 `1~2`，把 `pose_save_min_translation_m` 调大到 `0.3~0.5`。

- 机器人经常原地转向，恢复朝向不准：
  把 `pose_save_min_yaw_deg` 调小到 `3~5`。

- 文件几乎不更新：
  先检查是否一直没进入 `TRACKING_OK`；若状态正常，再适当减小 `pose_save_min_translation_m` 和 `pose_save_min_yaw_deg`。

- 文件更新太频繁：
  增大 `pose_save_interval_sec`，或增大 `pose_save_min_translation_m / pose_save_min_yaw_deg`。

#### BBS 全局重定位参数说明与调参建议

- `bbs_min_tx / bbs_max_tx / bbs_min_ty / bbs_max_ty`
  含义：BBS 全局搜索的平移范围，单位米。
  调参：
  范围必须覆盖地图有效区域，否则机器人在边缘区域会直接搜不到。
  范围过大时，搜索会更慢，也更容易出现错误候选。
  做法：先按地图包围盒给范围，再额外留 `3~10m` 裕量。

- `bbs_min_theta / bbs_max_theta`
  含义：BBS 全局搜索的偏航范围，单位弧度。
  调参：
  冷启动通常直接给 `[-pi, pi]`。
  如果你明确知道启动朝向不会差太多，可以收窄范围来提速。

- `bbs_map_min_z / bbs_map_max_z`
  含义：从 3D 地图中切出 2.5D BBS 匹配层的高度范围。
  调参：
  目标是保留最稳定、区分度最高的结构层，通常是墙面中下部、门框、柜体侧面。
  地面点太多：提高 `bbs_map_min_z`。
  天花板/横梁干扰大：降低 `bbs_map_max_z`。
  如果切片后点太少、BBS 经常找不到：适当放宽上下界。

- `bbs_scan_min_z / bbs_scan_max_z`
  含义：实时扫描用于 BBS 匹配的高度范围。
  调参：
  应尽量和 map 切片保持同一层语义，否则地图和实时扫描看到的结构层不一致。
  如果地图切的是“墙面中下部”，扫描也尽量切到相近高度。

- `bbs_map_resolution`
  含义：BBS 栅格分辨率，单位米。
  调参：
  分辨率小：更精细，但更耗时，对稀疏雷达更敏感。
  分辨率大：更稳更快，但粗糙，容易损失细结构。
  一般经验：
  稠密雷达如 MID360：`0.3~0.6m`
  稀疏机械雷达如 RS16：`0.6~1.0m`

- `bbs_map_width / bbs_map_height`
  含义：BBS 内部栅格尺寸，和 `bbs_map_resolution` 一起决定物理覆盖范围。
  调参：
  物理覆盖宽度约等于 `width * resolution`。
  必须保证大于你的搜索范围，否则会在边界截断。

- `bbs_map_pyramid_level`
  含义：BBS 多分辨率金字塔层数。
  调参：
  `5~7` 通常够用。
  地图大、搜索慢时可以适当增大。
  内存紧张或预处理太慢时可以适当减小。

- `bbs_max_points_per_cell`
  含义：每个栅格最多保留多少点。
  调参：
  太大时局部超密区域会主导评分。
  太小时又会丢掉局部结构。
  一般 `3~8` 是合理范围。

- `bbs_bounded_translation_window`
  含义：当已有恢复位姿或人工初值时，局部 BBS 搜索窗口的平移半宽。
  调参：
  先验可信时调小到 `3~8m`，搜索更快。
  里程计可能有较大漂移时调大到 `8~15m`。
  如果恢复位姿经常“就在附近却搜不到”，先把它调大。

- `bbs_bounded_yaw_window_deg`
  含义：当已有恢复位姿或人工初值时，局部 BBS 搜索窗口的偏航半宽，单位度。
  调参：
  轮式机器人一般 `20~60deg`。
  如果朝向先验比较靠谱，调小能显著提速。
  如果恢复时经常方向差很多，调大一些更稳。

#### BBS 参数的现场调法

- 冷启动时经常完全找不到：
  先检查 `bbs_min_tx/max_tx`、`bbs_min_ty/max_ty` 是否覆盖地图；再检查 `bbs_map_min_z/max_z` 和 `bbs_scan_min_z/max_z` 是否切得太窄。

- 能找到候选，但经常落在错误走廊/相似区域：
  先优化高度切片，让地图和扫描都只保留更有区分度的结构层；必要时适当减小 `bbs_map_resolution`。

- 搜索太慢：
  优先减小全局搜索范围，或者在有先验时减小 `bbs_bounded_translation_window / bbs_bounded_yaw_window_deg`；其次再考虑增大 `bbs_map_resolution`。

- 稀疏雷达上 BBS 很不稳定：
  适当增大 `bbs_map_resolution`，并检查切片是否保留了足够多的墙体点。

- 地图是大场景，局部恢复位姿基本可信：
  打开局部窗口搜索，优先调 `bbs_bounded_translation_window`，不要一上来就全图搜索。

## 版本优化更新 2025-10-16

### 时间戳同步优化，全部使用雷达时间戳进行处理数据

**问题描述：**
在rviz2中观察到点云和定位位姿存在滞后现象，主要原因是数据处理链中时间戳不一致。
