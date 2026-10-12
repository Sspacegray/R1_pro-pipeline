#ifndef _P2PL_ICP_H_
#define _P2PL_ICP_H_

#include "utility.h"
#include "nanoflann.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "pclomp/ndt_omp.h"

//特征点数据结构
struct pickup_feature{
  std::vector<PointType> features; //特征点
  std::vector<PointType> toplane_coffi; //特征点附近的点云构成平面的平面方程
  uint32_t pickup_num; //特征点数量
};

//2维的点云与地图匹配
class scan2map2d{
public:
  float plane_threshold_;
  float correspondences_threshold_normal_;
  float correspondences_threshold_reloc_;
  float use_correspondences_threshold_;
  int max_iteration_;

  float fitness_;

  int thread_num_;

  int outlier_point_num_;

  float locked_z_;

  std::shared_ptr<nanoflann::KdTreeFLANN<PointType>> map_kdtree_;
  pcl::PointCloud<PointType>::Ptr map_cloud_;

  pcl::PointCloud<PointType>::Ptr cloud_world; 

  scan2map2d(pcl::PointCloud<PointType>::Ptr map_cloud, std::string yaml_path);

  pickup_feature features_; //提取出的特征

  int iter_count_;
  double during_time_;

  void setRelocMode(void);
  void setNormalMode(void);
  void setMaxIteration(int iter);
  void setCorrespondencesThreshold(float value);


  void surfOptimization(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
  bool LM_optimization(int iter_count, pose_type &act_pose);
  
  pose_type location(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
};

//3维的点云与地图匹配
class scan2map3d{
public:
  enum class RegistrationMethod {
    kPointToPlaneICP,
    kNDT,
    kNDTPlusGicp
  };

  float plane_threshold_;
  float correspondences_threshold_normal_;
  float correspondences_threshold_reloc_;
  float use_correspondences_threshold_;
  int max_iteration_;

  float fitness_;
  float raw_fitness_;

  int thread_num_;

  std::shared_ptr<nanoflann::KdTreeFLANN<PointType>> map_kdtree_;
  pcl::PointCloud<PointType>::Ptr map_cloud_;

  pcl::PointCloud<PointType>::Ptr cloud_world;
  pcl::PointCloud<PointType>::Ptr ndt_source_cloud_;

  RegistrationMethod registration_method_;
  std::string registration_method_name_;
  float ndt_voxel_size_;
  int ndt_max_iteration_;
  int ndt_min_effective_pts_;
  int ndt_min_pts_in_voxel_;
  float ndt_eps_;
  float ndt_res_outlier_th_;
  bool ndt_remove_centroid_;
  int ndt_nearby_type_;
  int ndt_thread_num_;
  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> ndt_;

  // ndt_gicp 模式: small_gicp 终精配准
  float gicp_voxel_size_;
  float gicp_max_corr_dist_;
  int gicp_max_iterations_;
  float gicp_min_inlier_ratio_;
  double gicp_during_time_ms_ = 0.0;
  pcl::PointCloud<pcl::PointCovariance>::Ptr gicp_target_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> gicp_target_tree_;

  scan2map3d(pcl::PointCloud<PointType>::Ptr map_cloud, std::string yaml_path);

  pickup_feature features_; //提取出的特征

  int iter_count_;
  double during_time_;

  void setRelocMode(void);
  void setNormalMode(void);
  void setMaxIteration(int iter);
  void setCorrespondencesThreshold(float value);
  const std::string &registrationMethodName() const;


  void surfOptimization(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
  bool LM_optimization(int iter_count, pose_type &act_pose);
  pose_type locationNDT(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
  pose_type locationGicpRefine(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
  float computeInlierRatio(const pcl::PointCloud<PointType>::Ptr &aligned_cloud) const;
  float computeInlierRatioTight(const pcl::PointCloud<PointType>::Ptr &aligned_cloud, float threshold) const;

  // 用指定位姿把源云变换到地图系后, 以严格阈值评估贴合度(不动 fitness_)
  float probeTightFitness(const pose_type &pose, const pcl::PointCloud<PointType>::Ptr &cloud, float threshold_m) const;
  pclomp::NeighborSearchMethod resolveNdtSearchMethod() const;
  
  pose_type location(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud);
};





#endif
