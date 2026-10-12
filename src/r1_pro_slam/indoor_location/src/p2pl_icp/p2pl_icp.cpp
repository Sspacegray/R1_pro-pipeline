#include "p2pl_icp.h"
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


  cloud_world.reset(new pcl::PointCloud<PointType>());
}

void scan2map3d::setRelocMode(void){
  use_correspondences_threshold_ = correspondences_threshold_reloc_;
}

void scan2map3d::setNormalMode(void){
  use_correspondences_threshold_ = correspondences_threshold_normal_;
}

void scan2map3d::setMaxIteration(int iter){
  max_iteration_ = iter;
}

void scan2map3d::setCorrespondencesThreshold(float value){
  use_correspondences_threshold_ = value;
}

pose_type scan2map3d::location(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud)
{
    pose_type res_pose = init_pose;
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




