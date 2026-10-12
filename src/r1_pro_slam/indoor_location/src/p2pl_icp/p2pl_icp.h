#ifndef _P2PL_ICP_H_
#define _P2PL_ICP_H_

#include "utility.h"
#include "nanoflann.hpp"

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
  float plane_threshold_;
  float correspondences_threshold_normal_;
  float correspondences_threshold_reloc_;
  float use_correspondences_threshold_;
  int max_iteration_;

  float fitness_;

  int thread_num_;

  std::shared_ptr<nanoflann::KdTreeFLANN<PointType>> map_kdtree_;
  pcl::PointCloud<PointType>::Ptr map_cloud_;

  pcl::PointCloud<PointType>::Ptr cloud_world; 

  scan2map3d(pcl::PointCloud<PointType>::Ptr map_cloud, std::string yaml_path);

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





#endif