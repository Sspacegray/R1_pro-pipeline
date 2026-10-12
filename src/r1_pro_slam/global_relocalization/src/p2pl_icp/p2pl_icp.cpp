#include "p2pl_icp.h"

#include <algorithm>
#include <cctype>

#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/util/normal_estimation_omp.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

// small_gicp 类型别名 (与 icp_relocalization/gicp_filter 一致的用法)
using GicpPointCloud = pcl::PointCloud<pcl::PointCovariance>;
using GicpRegister =
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;

/*=============================================================================================================================

icp 2 plane: 3 dimension

=============================================================================================================================*/
scan2map3d::scan2map3d(pcl::PointCloud<PointType>::Ptr map_cloud, std::string yaml_path){
  map_kdtree_.reset(new nanoflann::KdTreeFLANN<PointType>());
  map_kdtree_->setInputCloud(map_cloud);

  map_cloud_.reset(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*map_cloud, *map_cloud_);

  YAML::Node config;
  //读取yaml数据
  try{
    config = YAML::LoadFile(yaml_path);
  } catch(YAML::BadFile &e) {
    std::cout<<" Loc :  in process scan2map2d -- read config file error!"<<std::endl;
    return;
  }

  plane_threshold_ = config["plane_threshold"].as<float>();
  correspondences_threshold_normal_ = config["opt_correspondences_threshold_normal"].as<float>();
  correspondences_threshold_reloc_ = config["opt_correspondences_threshold_relocation"].as<float>();
  use_correspondences_threshold_ = correspondences_threshold_reloc_;

  max_iteration_ = config["opt_max_iteration"].as<int>();
  thread_num_ = config["thread_num"].as<int>();
  registration_method_name_ = config["registration_method"] ? config["registration_method"].as<std::string>() : "p2pl_icp";
  std::transform(
      registration_method_name_.begin(),
      registration_method_name_.end(),
      registration_method_name_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (registration_method_name_ == "ndt") {
    registration_method_ = RegistrationMethod::kNDT;
  } else if (registration_method_name_ == "ndt_gicp") {
    // NDT 粗精配准 + small_gicp 终精配准两段串联
    registration_method_ = RegistrationMethod::kNDTPlusGicp;
  } else {
    registration_method_ = RegistrationMethod::kPointToPlaneICP;
    registration_method_name_ = "p2pl_icp";
  }

  gicp_voxel_size_ = config["gicp_voxel_size"] ? config["gicp_voxel_size"].as<float>() : 0.2f;
  gicp_max_corr_dist_ = config["gicp_max_correspondence_distance"] ? config["gicp_max_correspondence_distance"].as<float>() : 1.0f;
  gicp_max_iterations_ = config["gicp_max_iterations"] ? config["gicp_max_iterations"].as<int>() : 64;
  gicp_min_inlier_ratio_ = config["gicp_min_inlier_ratio"] ? config["gicp_min_inlier_ratio"].as<float>() : 0.3f;

  ndt_voxel_size_ = config["ndt_voxel_size"] ? config["ndt_voxel_size"].as<float>() : 0.5f;
  ndt_max_iteration_ = config["ndt_max_iteration"] ? config["ndt_max_iteration"].as<int>() : max_iteration_;
  ndt_min_effective_pts_ = config["ndt_min_effective_pts"] ? config["ndt_min_effective_pts"].as<int>() : 10;
  ndt_min_pts_in_voxel_ = config["ndt_min_pts_in_voxel"] ? config["ndt_min_pts_in_voxel"].as<int>() : 6;
  ndt_eps_ = config["ndt_eps"] ? config["ndt_eps"].as<float>() : 0.01f;
  ndt_res_outlier_th_ = config["ndt_res_outlier_th"] ? config["ndt_res_outlier_th"].as<float>() : use_correspondences_threshold_;
  ndt_remove_centroid_ = config["ndt_remove_centroid"] ? config["ndt_remove_centroid"].as<bool>() : false;
  ndt_nearby_type_ = config["ndt_nearby_type"] ? config["ndt_nearby_type"].as<int>() : 0;
  ndt_thread_num_ = config["ndt_thread_num"] ? config["ndt_thread_num"].as<int>() : thread_num_;

  cloud_world.reset(new pcl::PointCloud<PointType>());
  ndt_source_cloud_.reset(new pcl::PointCloud<PointType>());
  raw_fitness_ = 0.0f;

  if (registration_method_ == RegistrationMethod::kNDT ||
      registration_method_ == RegistrationMethod::kNDTPlusGicp) {
    ndt_.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>());
    ndt_->setInputTarget(map_cloud_);
    ndt_->setResolution(std::max(ndt_voxel_size_, 0.1f));
    ndt_->setMaximumIterations(ndt_max_iteration_);
    ndt_->setTransformationEpsilon(ndt_eps_);
    ndt_->setNumThreads(std::max(1, ndt_thread_num_));
    ndt_->setNeighborhoodSearchMethod(resolveNdtSearchMethod());
    ndt_->setMinPointPerVoxel(std::max(3, ndt_min_pts_in_voxel_));
  }

  // GICP 目标(地图): 体素采样 + 协方差 + KD-tree, 一次构建终身复用
  if (registration_method_ == RegistrationMethod::kNDTPlusGicp) {
    gicp_target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointType>, GicpPointCloud>(
        *map_cloud_, gicp_voxel_size_);
    small_gicp::estimate_covariances_omp(*gicp_target_, 20, 8);
    gicp_target_tree_ = std::make_shared<small_gicp::KdTree<GicpPointCloud>>(
        gicp_target_, small_gicp::KdTreeBuilderOMP(8));
    std::cout << "[GICP精] 目标地图就绪: " << gicp_target_->size() << " 点 (voxel=" << gicp_voxel_size_ << ")" << std::endl;
  }
}

void scan2map3d::setRelocMode(void){
  use_correspondences_threshold_ = correspondences_threshold_reloc_;
}

void scan2map3d::setNormalMode(void){
  use_correspondences_threshold_ = correspondences_threshold_normal_;
}

void scan2map3d::setMaxIteration(int iter){
  max_iteration_ = iter;
  ndt_max_iteration_ = iter;
  if (ndt_) {
    ndt_->setMaximumIterations(ndt_max_iteration_);
  }
}

void scan2map3d::setCorrespondencesThreshold(float value){
  use_correspondences_threshold_ = value;
}

const std::string &scan2map3d::registrationMethodName() const {
  return registration_method_name_;
}

pclomp::NeighborSearchMethod scan2map3d::resolveNdtSearchMethod() const {
  switch (ndt_nearby_type_) {
    case 1:
      return pclomp::DIRECT26;
    case 2:
      return pclomp::DIRECT7;
    case 3:
      return pclomp::DIRECT1;
    case 0:
    default:
      return pclomp::KDTREE;
  }
}

float scan2map3d::computeInlierRatio(const pcl::PointCloud<PointType>::Ptr &aligned_cloud) const {
  if (!aligned_cloud || aligned_cloud->empty()) {
    return 0.0f;
  }

  const float threshold_sq = std::max(
      1e-4f,
      ndt_res_outlier_th_ > 0.0f ? ndt_res_outlier_th_ : use_correspondences_threshold_);

  int inlier_count = 0;
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  pointSearchInd.reserve(1);
  pointSearchSqDis.reserve(1);
  for (const auto &point : aligned_cloud->points) {
    pointSearchInd.clear();
    pointSearchSqDis.clear();
    map_kdtree_->nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);
    if (!pointSearchSqDis.empty() && pointSearchSqDis.front() <= threshold_sq) {
      ++inlier_count;
    }
  }

  return static_cast<float>(inlier_count) / static_cast<float>(aligned_cloud->size());
}

float scan2map3d::computeInlierRatioTight(const pcl::PointCloud<PointType>::Ptr &aligned_cloud, float threshold) const {
  if (!aligned_cloud || aligned_cloud->empty() || !map_kdtree_) {
    return 0.0f;
  }
  const float threshold_sq = threshold * threshold;
  int inlier_count = 0;
  std::vector<int> idx;
  std::vector<float> sq;
  idx.reserve(1);
  sq.reserve(1);
  for (const auto &point : aligned_cloud->points) {
    idx.clear();
    sq.clear();
    map_kdtree_->nearestKSearch(point, 1, idx, sq);
    if (!sq.empty() && sq.front() <= threshold_sq) {
      ++inlier_count;
    }
  }
  return static_cast<float>(inlier_count) / static_cast<float>(aligned_cloud->size());
}

float scan2map3d::probeTightFitness(const pose_type &pose, const pcl::PointCloud<PointType>::Ptr &cloud, float threshold_m) const {
  if (!cloud || cloud->empty()) return 0.0f;
  pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*cloud, *aligned, Pose2Matrix(pose));
  return computeInlierRatioTight(aligned, threshold_m);
}

pose_type scan2map3d::locationGicpRefine(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud)
{
  // small_gicp 终精配准: 以 NDT/ICP 结果为初值, GICP 面对面配准
  pose_type res_pose = init_pose;
  if (!gicp_target_ || gicp_target_->empty() || !gicp_target_tree_ ||
      !lidar_cloud || lidar_cloud->points.empty()) {
    return res_pose;
  }

  // 源点云: 体素降采样到 PointCovariance 云 + 协方差估计 (一次性, 每帧重建)
  GicpPointCloud::Ptr source(new GicpPointCloud());
  if (gicp_voxel_size_ > 0.0f) {
    source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointType>, GicpPointCloud>(
        *lidar_cloud, gicp_voxel_size_);
  } else {
    source->points.reserve(lidar_cloud->points.size());
    for (const auto &pt : lidar_cloud->points) {
      pcl::PointCovariance q;
      q.x = pt.x; q.y = pt.y; q.z = pt.z;
      source->points.push_back(q);
    }
    source->width = static_cast<uint32_t>(source->points.size());
    source->height = 1;
  }
  if (source->empty()) return res_pose;
  small_gicp::estimate_covariances_omp(*source, 20, 8);

  GicpRegister reg;
  reg.reduction.num_threads = 8;
  reg.rejector.max_dist_sq = gicp_max_corr_dist_ * gicp_max_corr_dist_;
  reg.optimizer.max_iterations = gicp_max_iterations_;

  Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();
  T0.matrix() = Pose2Matrix(init_pose).cast<double>();

  auto start = std::chrono::system_clock::now();
  const auto result = reg.align(*gicp_target_, *source, *gicp_target_tree_, T0);
  auto end = std::chrono::system_clock::now();
  gicp_during_time_ms_ =
      std::chrono::duration<double, std::milli>(end - start).count();

  if (!result.converged) {
    std::cout << "[GICP精] 未收敛, 保留上段位姿 (err=" << result.error
              << ", inliers=" << result.num_inliers << ")" << std::endl;
    return res_pose;
  }

  const double inlier_ratio = source->size() > 0
      ? static_cast<double>(result.num_inliers) / static_cast<double>(source->size())
      : 0.0;
  if (inlier_ratio < gicp_min_inlier_ratio_) {
    std::cout << "[GICP精] 内点率过低 " << inlier_ratio << " < " << gicp_min_inlier_ratio_
              << ", 保留上段位姿" << std::endl;
    return res_pose;
  }

  pose_type refined = Matrix2Pose(result.T_target_source.matrix().cast<float>());
  res_pose.pos = refined.pos;
  res_pose.orient = refined.orient;
  update_q(res_pose);

  // fitness 重算: 用对齐后源云在地图 KD-tree 的内点率, 语义与 NDT 路径一致
  pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*lidar_cloud, *aligned, Pose2Matrix(res_pose));
  fitness_ = computeInlierRatio(aligned);
  std::cout << "[GICP精] 收敛: " << gicp_during_time_ms_ << "ms inliers=" << result.num_inliers
            << "/" << source->size() << " (" << inlier_ratio << ") fitness=" << fitness_ << std::endl;
  return res_pose;
}

pose_type scan2map3d::locationNDT(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud) {
  pose_type res_pose = init_pose;
  raw_fitness_ = 0.0f;
  fitness_ = 0.0f;
  iter_count_ = 0;

  if (!ndt_ || !lidar_cloud || lidar_cloud->points.empty()) {
    return res_pose;
  }

  ndt_source_cloud_->clear();
  if (ndt_voxel_size_ > 0.0f) {
    pcl::VoxelGrid<PointType> source_filter;
    source_filter.setLeafSize(ndt_voxel_size_, ndt_voxel_size_, ndt_voxel_size_);
    source_filter.setInputCloud(lidar_cloud);
    source_filter.filter(*ndt_source_cloud_);
  } else {
    pcl::copyPointCloud(*lidar_cloud, *ndt_source_cloud_);
  }

  if (static_cast<int>(ndt_source_cloud_->size()) < ndt_min_effective_pts_) {
    return res_pose;
  }

  ndt_->setInputSource(ndt_source_cloud_);
  pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
  const Eigen::Matrix4f init_guess = Pose2Matrix(init_pose);

  auto start = std::chrono::system_clock::now();
  ndt_->align(*aligned_cloud, init_guess);
  auto end = std::chrono::system_clock::now();
  during_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  iter_count_ = ndt_->getFinalNumIteration();
  raw_fitness_ = static_cast<float>(ndt_->getFitnessScore());

  if (!ndt_->hasConverged()) {
    return res_pose;
  }

  pose_type optimized_pose = Matrix2Pose(ndt_->getFinalTransformation());
  res_pose.pos = optimized_pose.pos;
  res_pose.orient = optimized_pose.orient;
  update_q(res_pose);
  fitness_ = computeInlierRatio(aligned_cloud);
  return res_pose;
}

pose_type scan2map3d::location(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud)
{
    if (registration_method_ == RegistrationMethod::kNDT ||
        registration_method_ == RegistrationMethod::kNDTPlusGicp) {
        pose_type ndt_pose = locationNDT(init_pose, lidar_cloud);
        if (registration_method_ == RegistrationMethod::kNDTPlusGicp) {
            return locationGicpRefine(ndt_pose, lidar_cloud);
        }
        return ndt_pose;
    }

    pose_type res_pose = init_pose;
    raw_fitness_ = 0.0f;
    if (!lidar_cloud || lidar_cloud->points.empty()) {
        fitness_ = 0.0f;
        return res_pose;
    }
    //RCLCPP_INFO(rclcpp::get_logger("location"), "666666666666666666666666666666666666666666");
    //double dt_seq1, dt_seq2;
    iter_count_=0;
//    RCLCPP_INFO(rclcpp::get_logger("location"), "max_iteration_: %d", max_iteration_);
    auto start = std::chrono::system_clock::now();
    while(iter_count_ < max_iteration_){
        surfOptimization(res_pose, lidar_cloud);
        //auto end_seq1 = std::chrono::system_clock::now();
        fitness_ = static_cast<float>(features_.pickup_num) / static_cast<float>(lidar_cloud->points.size());
        if (LM_optimization(iter_count_, res_pose) == true){
            if(iter_count_>1) break;
        }
        iter_count_++;
    }
    auto end = std::chrono::system_clock::now();
    during_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return res_pose;
}

void scan2map3d::surfOptimization(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud){
    if (!lidar_cloud || lidar_cloud->points.empty()) {
        features_.features.clear();
        features_.toplane_coffi.clear();
        features_.pickup_num = 0;
        return;
    }

    Eigen::Matrix4f affine_mat = Pose2Matrix(init_pose);

    uint32_t pl_size = lidar_cloud->points.size();
    // 预分配，避免 push_back 频繁重分配
    cloud_world->points.resize(pl_size);
    PointType pt_world;

    for(uint32_t i=0; i<pl_size; i++){
        const PointType &pt = lidar_cloud->points[i];
        pt_world.x = affine_mat(0,0) * pt.x + affine_mat(0,1) * pt.y + affine_mat(0,2) * pt.z + affine_mat(0,3);
        pt_world.y = affine_mat(1,0) * pt.x + affine_mat(1,1) * pt.y + affine_mat(1,2) * pt.z + affine_mat(1,3);
        pt_world.z = affine_mat(2,0) * pt.x + affine_mat(2,1) * pt.y + affine_mat(2,2) * pt.z + affine_mat(2,3);
        pt_world.intensity = pt.intensity;
        cloud_world->points[i] = pt_world;
    }

    std::vector<PointType> features(pl_size);
    std::vector<PointType> toplane_coffi(pl_size);
    std::vector<bool> picked(pl_size, false);

#pragma omp parallel for num_threads(thread_num_)
    for(uint32_t idx=0; idx<pl_size; idx++){
        const PointType &pointOri = lidar_cloud->points[idx];
        const PointType &pointSel = cloud_world->points[idx];
        PointType coeff;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // 如果 nanoflann 的 KdTreeFLANN::nearestKSearch 不是线程安全的，保留 critical
        #pragma omp critical(kdtree_search)
        map_kdtree_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if(pointSearchInd.size() < 5) { continue; }
        if(pointSearchSqDis[4] > use_correspondences_threshold_) { continue; }

        Eigen::Matrix<float, 5, 3> matA0;
        matA0.setZero();
        for (int j = 0; j < 5; j++) {
            const auto &mpt = map_cloud_->points[pointSearchInd[j]];
            matA0(j,0) = mpt.x; matA0(j,1) = mpt.y; matA0(j,2) = mpt.z;
        }

        Eigen::Matrix<float,3,3> M = Eigen::Matrix<float,3,3>::Zero();
        Eigen::Matrix<float,3,1> N = Eigen::Matrix<float,3,1>::Zero();
        for(int k=0;k<5;k++){
            M(0,0) += matA0(k,0) * matA0(k,0);
            M(0,1) += matA0(k,0) * matA0(k,1);
            M(0,2) += matA0(k,0) * matA0(k,2);
            M(1,1) += matA0(k,1) * matA0(k,1);
            M(1,2) += matA0(k,1) * matA0(k,2);
            M(2,2) += matA0(k,2) * matA0(k,2);
            N(0) -= matA0(k,0);
            N(1) -= matA0(k,1);
            N(2) -= matA0(k,2);
        }
        M(1,0)=M(0,1); M(2,0)=M(0,2); M(2,1)=M(1,2);

        Eigen::Vector3f matX0 = M.llt().solve(N);
        if(std::isnan(matX0(0))||std::isnan(matX0(1))||std::isnan(matX0(2))) continue;

        float pa = matX0(0), pb = matX0(1), pc = matX0(2), pd = 1.0f;
        float ps = std::sqrt(pa*pa + pb*pb + pc*pc);
        pa/=ps; pb/=ps; pc/=ps; pd/=ps;

        bool is_plane = true;
        for(int j=0;j<5;j++){
            const auto &mpt = map_cloud_->points[pointSearchInd[j]];
            if (fabs(pa*mpt.x + pb*mpt.y + pc*mpt.z + pd) > plane_threshold_){ is_plane=false; break; }
        }
        if(!is_plane) continue;

        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
        float s = 1 - 0.9f * fabs(pd2) / sqrt(sqrt(pointOri.x*pointOri.x + pointOri.y*pointOri.y + pointOri.z*pointOri.z));
        if(s < 0.1f) continue;

        coeff.x = s*pa; coeff.y = s*pb; coeff.z = s*pc; coeff.intensity = s * pd2;

        features[idx] = pointOri;
        toplane_coffi[idx] = coeff;
        picked[idx] = true;
    }

    features_.features.clear();
    features_.toplane_coffi.clear();
    features_.pickup_num = 0;
    for(uint32_t i=0;i<pl_size;i++){
        if(picked[i]){
            features_.features.push_back(features[i]);
            features_.toplane_coffi.push_back(toplane_coffi[i]);
            features_.pickup_num++;
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("surfOptimization"), "Plan Matched feature points: %u", features_.pickup_num);
}



bool scan2map3d::LM_optimization(int iter_count, pose_type &act_pose){

  uint32_t pl_size = features_.pickup_num;
  if (pl_size == 0) return true;

  float srx = sin(act_pose.orient[0]);
  float crx = cos(act_pose.orient[0]);
  float sry = sin(act_pose.orient[1]);
  float cry = cos(act_pose.orient[1]);
  float srz = sin(act_pose.orient[2]);
  float crz = cos(act_pose.orient[2]);

  Eigen::Matrix<double, 6, 6> matAtA = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> matAtB = Eigen::Matrix<double, 6, 1>::Zero();

#pragma omp parallel num_threads(thread_num_)
  {
    Eigen::Matrix<double, 6, 6> localAtA = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> localAtB = Eigen::Matrix<double, 6, 1>::Zero();

    #pragma omp for nowait
    for(uint32_t i=0; i<pl_size; i++){
      const PointType &pointOri = features_.features[i];
      const PointType &coeff = features_.toplane_coffi[i];

      float arx = ((crz * sry * crx + srz * srx)* pointOri.y - (crz * sry * srx - srz * crx) * pointOri.z) * coeff.x
                  + ((srz * sry * crx - crz * srx) * pointOri.y - (srz * sry *srx + crz * crx) * pointOri.z) * coeff.y
                  + (cry * crx * pointOri.y - cry * srx * pointOri.z) * coeff.z;

      float ary = (- crz * sry * pointOri.x + crz * cry * srx * pointOri.y  + crz * cry *crx * pointOri.z) * coeff.x
                  + (-srz * sry * pointOri.x + srz * cry * srx * pointOri.y + srz * cry * crx * pointOri.z) * coeff.y 
                  + (-cry * pointOri.x - srx * sry * pointOri.y - crx * sry * pointOri.z) * coeff.z;

      float arz = (-cry * srz * pointOri.x - (srx * sry * srz + crz * crx) * pointOri.y - (crx * sry * srz - crz * srx) * pointOri.z) * coeff.x
                  + (cry * crz * pointOri.x + (srx * sry * crz - srz * crx) * pointOri.y + (crx * sry * crz + srz * srx) * pointOri.z) * coeff.y;

      Eigen::Matrix<double, 1, 6> J;
      J(0,0) = static_cast<double>(coeff.x);
      J(0,1) = static_cast<double>(coeff.y);
      J(0,2) = static_cast<double>(coeff.z);
      J(0,3) = static_cast<double>(arx);
      J(0,4) = static_cast<double>(ary);
      J(0,5) = static_cast<double>(arz);

      double error = -static_cast<double>(coeff.intensity);
      localAtA += J.transpose() * J;
      localAtB += J.transpose() * error;
    }

    #pragma omp critical
    {
      matAtA += localAtA;
      matAtB += localAtB;
    }
  }

  Eigen::Matrix<double, 6,1> matX = matAtA.colPivHouseholderQr().solve(matAtB);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(matAtA);

  bool isDegenerate = false;
  Eigen::Matrix<double, 6, 6> matP;
  if (iter_count == 0){
    Eigen::Matrix<double, 6, 1> matE = eigen_solver.eigenvalues();
    Eigen::Matrix<double, 6, 6> matV = eigen_solver.eigenvectors();
    Eigen::Matrix<double, 6, 6> matV2 = matV;
    double eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE(i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) matV2(i, j) = 0;
        isDegenerate = true;
      } else break;
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate){
    Eigen::Matrix<double, 6, 1> matX2 = matX;
    matX = matP * matX2;
  }

  act_pose.pos(0)+=matX(0);
  act_pose.pos(1)+=matX(1);
  act_pose.pos(2)+=matX(2);
  act_pose.orient(0)+=matX(3);
  act_pose.orient(1)+=matX(4);
  act_pose.orient(2)+=matX(5);

  double deltaR = sqrt(pow(matX(3)/Deg2Rad, 2) + pow(matX(4)/Deg2Rad, 2) + pow(matX(5)/Deg2Rad, 2));
  double deltaT = sqrt(pow(matX(0), 2) + pow(matX(1), 2) + pow(matX(2), 2));

  return (deltaR < 0.1 && deltaT < 0.03);
}
