#pragma once
// 防止头文件被多次包含

#include "commons.h"      
// 公共定义（Config、PointType、CloudType、Vec 等）

#include "ieskf.h"        
// IESKF 滤波器，用于状态估计

#include "ikd_Tree.h"     
// 增量式 KD-Tree（ikd-Tree，用于点云最近邻搜索）

#include <pcl/filters/voxel_grid.h>    
// PCL 体素滤波器（点云下采样）

#include <pcl/common/transforms.h>     
// PCL 点云坐标变换函数


struct LocalMap
{
    bool initialed = false;             // 局部地图是否已经初始化
    BoxPointType local_map_corner;      // 局部地图的包围盒角点（立方体范围）
    Vec<BoxPointType> cub_to_rm;        // 待移除的立方体（局部地图裁剪时用）
};

class LidarProcessor
{
public:
    // 构造函数：需要系统配置 config 和 IESKF 状态估计器
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();   // 裁剪局部地图（防止地图无限膨胀）

    void incrCloudMap();   // 增量式更新局部地图（加入新的扫描数据）

    void initCloudMap(PointVec &point_vec);  
    // 初始化局部地图（通常在系统开始时，把第一帧点云加入地图）

    void process(SyncPackage &package); 
    // 处理一帧同步包（IMU+Lidar）：包括点云处理、状态估计、地图更新

    void updateLossFunc(State &state, SharedState &share_data);  
    // 更新损失函数：基于点到平面的残差，填充 EKF 的观测 Jacobian H、残差 b

    // 静态方法：将输入点云应用旋转 r、平移 t，得到变换后的点云
    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);

    // 当前 LiDAR 在世界坐标系下的旋转矩阵：R_wi * R_il
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }

    // 当前 LiDAR 在世界坐标系下的平移向量：t_wi + R_wi * t_il
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

private:
    Config m_config;                          // 系统配置参数
    LocalMap m_local_map;                     // 局部地图管理
    std::shared_ptr<IESKF> m_kf;              // IESKF 状态估计器
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree; // 局部地图 KD-Tree（用于最近邻搜索）

    CloudType::Ptr m_cloud_lidar;             // 当前帧原始 LiDAR 点云
    CloudType::Ptr m_cloud_down_lidar;        // 下采样后的 LiDAR 点云（雷达坐标系）
    CloudType::Ptr m_cloud_down_world;        // 下采样后的 LiDAR 点云（世界坐标系）

    std::vector<bool> m_point_selected_flag;  // 点是否被选中用于匹配的标志位

    CloudType::Ptr m_norm_vec;                // 法向量集合（估计出的平面法向）
    CloudType::Ptr m_effect_cloud_lidar;      // 有效点（参与优化的点）
    CloudType::Ptr m_effect_norm_vec;         // 有效点对应的法向量

    std::vector<PointVec> m_nearest_points;   // 每个点的最近邻点集合（用于平面拟合）

    pcl::VoxelGrid<PointType> m_scan_filter;  // PCL 体素滤波器（下采样用）
};
