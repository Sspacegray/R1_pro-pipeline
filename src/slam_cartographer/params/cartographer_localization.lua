
include "cartographer_mapping.lua"

-- 启用纯定位模式，冻结地图，不再新建子地图
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,  -- 保留的子地图数量
}

-- 禁用新子地图的创建，强制使用先验地图
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 999999  -- 设置极大值，防止创建新子地图

-- 定位模式专用参数调整，减少抖动
-- 注意：这些参数在options表中定义，在文件末尾修改

-- 增强Motion Filter，减少微小抖动
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0  -- 减少时间阈值，更频繁过滤
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- 减小距离阈值，过滤微小移动
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)  -- 减小角度阈值，过滤微小旋转

-- 增强Ceres扫描匹配器稳定性
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 50.0  -- 增加平移权重，提高稳定性
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100.0  -- 增加旋转权重，减少角度抖动
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- 减少迭代次数，提高实时性

-- 修改options表中的发布频率参数
options.pose_publish_period_sec = 0.02  -- 50Hz定位频率，提供最高精度
options.trajectory_publish_period_sec = 0.1  -- 10Hz轨迹发布频率

return options
