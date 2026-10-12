#include "eskf.h"

ErrorStateKalmanFilter::ErrorStateKalmanFilter(std::string yaml_file_path){
  YAML::Node config;

  std::cout << "yaml of eskf: " << yaml_file_path <<std::endl;
  //读取yaml数据
  try{
    config = YAML::LoadFile(yaml_file_path);
  } catch(YAML::BadFile &e) {
    std::cout<<" Loc :  in process eskf -- read config file error!"<<std::endl;
    return;
  }

  options_.gyro_var_ = config["gyro_var"].as<double>();
  options_.acce_var_ = config["acc_var"].as<double>();
  options_.bias_gyro_var_ = config["bias_gyro_var"].as<double>();
  options_.bias_acce_var_ = config["bias_acc_var"].as<double>();
  options_.obv_pos_noise_ = config["obv_pos_noise"].as<double>();
  options_.obv_ang_noise_ = config["obv_ang_noise"].as<double>() * Deg2Rad;
  options_.update_bias_gyro_ = config["update_bias_gyro"].as<bool>();
  options_.update_bias_acce_ = config["update_bias_acc"].as<bool>();

  options_.g_(0) = 0.0;
  options_.g_(1) = 0.0;
  options_.g_(2) = -9.8;


  BuildNoise();

  cov_ = Eigen::Matrix<double, 18, 18>::Identity() * 0.0001;
  dx_ = Eigen::Matrix<double, 18, 1>::Zero();

  v_ = Eigen::Vector3d::Zero();

  bg_ = Eigen::Vector3d::Zero();
  ba_ = Eigen::Vector3d::Zero();
  g_ = options_.g_;

  Eigen::Vector3d gyro_last_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_last_ = -options_.g_;
}


void ErrorStateKalmanFilter::reset(void){
  cov_ = Eigen::Matrix<double, 18, 18>::Identity() * 0.0001;
  dx_ = Eigen::Matrix<double, 18, 1>::Zero();

  v_ = Eigen::Vector3d::Zero();

  bg_ = Eigen::Vector3d::Zero();
  ba_ = Eigen::Vector3d::Zero();
  g_ = options_.g_;

  Eigen::Vector3d gyro_last_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_last_ = -options_.g_;
}


void ErrorStateKalmanFilter::setMean(Eigen::Vector3d pos, Eigen::Vector3d rpy, Eigen::Vector3d vel){
  p_ = pos;
  v_ = vel;
  R_ = ExpF64(rpy);
}

void ErrorStateKalmanFilter::setCov(Eigen::Matrix<double, 18, 18> s){
  cov_ = s;
}

void ErrorStateKalmanFilter::getPose(pose_type &pose){
  if(first_obv_) return;

  pose.pos = p_.cast<float>();
  pose.orient = RotMtoEuler(R_.cast<float>());
  pose.vel = v_.cast<float>();
  update_q(pose);
}

Eigen::Matrix<double, 6, 1> ErrorStateKalmanFilter::getVel(void){ 
  Eigen::Matrix<double, 6, 1> res;
  res(0) = v_(0);
  res(1) = v_(1);
  res(2) = v_(2);
  res(3) = gyro_last_(0);
  res(4) = gyro_last_(1);
  res(5) = gyro_last_(2);
  return res;
}

/* void ErrorStateKalmanFilter::predict(Eigen::VectorXd& control, double timestamp){
  Eigen::Vector3d acc_raw = control.middleRows(0, 3);
  Eigen::Vector3d gyro_raw = control.middleRows(3, 3);


  if(first_obv_){
    last_timestamp = timestamp;
    acc_last_ = acc_raw - ba_;
    gyro_last_ = gyro_raw - bg_;
    return;
  }

  double dt = timestamp - last_timestamp;
  if(dt>0.03) dt = 0.01;

  Eigen::Vector3d acc = acc_raw - ba_;
  Eigen::Vector3d gyro = gyro_raw - bg_;

  Eigen::Vector3d acc_mean = 0.5 * (acc + acc_last_);
  Eigen::Vector3d gyro_mean = 0.5 * (gyro + gyro_last_);

  bool data_ok = true;
  if(std::abs(acc_mean(0)) > 10000.0) data_ok = false;
  if(std::abs(acc_mean(1)) > 10000.0) data_ok = false;
  if(std::abs(acc_mean(2)) > 10000.0) data_ok = false;
  if(std::abs(gyro_mean(0)) > 10000.0) data_ok = false;
  if(std::abs(gyro_mean(1)) > 10000.0) data_ok = false;
  if(std::abs(gyro_mean(2)) > 10000.0) data_ok = false;

  if(!data_ok){
    acc_mean = -options_.g_;
    gyro_mean = Eigen::Vector3d::Zero();
    std::cout << "imu data failed" <<std::endl;
  }

  // nominal state 递推
  Eigen::Vector3d new_p = p_ + v_ * dt + 0.5 * (R_ * acc_mean) * dt * dt + 0.5 * g_ * dt * dt;
  Eigen::Vector3d new_v = v_ + R_ * acc_mean * dt + g_ * dt;
  Eigen::Matrix3d new_R = R_ * ExpF64(gyro_mean * dt); 

  
  R_ = new_R;
  v_ = new_v;
  p_ = new_p;

  // 其余状态维度不变
  Eigen::Matrix3d hat_acc;
  hat_acc << 0.0, -acc_mean(2), acc_mean(1), acc_mean(2), 0.0, -acc_mean(0), -acc_mean(1), acc_mean(0), 0.0;

  // error state 递推
  // 计算运动过程雅可比矩阵 F，见(3.47)
  // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
  Eigen::Matrix<double, 18, 18> F = Eigen::Matrix<double, 18, 18>::Identity();   // 主对角线
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;                        // p 对 v
  F.block<3, 3>(3, 6) = -R_ * hat_acc * dt;                                      // v对theta
  F.block<3, 3>(3, 12) = -R_ * dt;                                               // v 对 ba
  F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;                       // v 对 g
  F.block<3, 3>(6, 6) = ExpF64(-gyro_mean * dt);                                 // theta 对 theta
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;                       // theta 对 bg

  // mean and cov prediction
  //dx_ = F * dx_;  // 这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
  cov_ = F * cov_.eval() * F.transpose() + Q_;


  last_timestamp = timestamp;
  acc_last_ = acc;
  gyro_last_ = gyro;
} */

/*使用 真实 dt，不再截断；对加速度和角速度做 梯形法(midpoint)积分；对旋转积分用 midpoint + ExpF64，提高快速旋转精度；
 * 异常 IMU 数据处理更稳妥，不直接用 -g，用前一帧或丢弃；支持 dt 较大时 分步积分，避免积分误差累积*/
void ErrorStateKalmanFilter::predict(Eigen::VectorXd& control, double timestamp)
{
    Eigen::Vector3d acc_raw = control.middleRows(0, 3);
    Eigen::Vector3d gyro_raw = control.middleRows(3, 3);

    if(first_obv_){
        last_timestamp = timestamp;
        acc_last_ = acc_raw - ba_;
        gyro_last_ = gyro_raw - bg_;
        first_obv_ = false;
        return;
    }

    double dt_full = timestamp - last_timestamp;
    if(dt_full <= 0) return;

    // 如果 dt 太大，拆成多步积分
    const double max_dt = 0.01;
    int steps = std::ceil(dt_full / max_dt);
    double dt = dt_full / steps;

    Eigen::Vector3d acc_prev = acc_last_;
    Eigen::Vector3d gyro_prev = gyro_last_;

    for(int i = 0; i < steps; ++i)
    {
        // 当前时刻 IMU
        Eigen::Vector3d acc_curr = acc_raw - ba_;
        Eigen::Vector3d gyro_curr = gyro_raw - bg_;

        // 梯形法积分
        Eigen::Vector3d acc_mean = 0.5 * (acc_prev + acc_curr);
        Eigen::Vector3d gyro_mean = 0.5 * (gyro_prev + gyro_curr);

        // 数据异常判断
        bool data_ok = true;
        for(int j=0;j<3;j++){
            if(std::abs(acc_mean(j))>10000.0 || std::abs(gyro_mean(j))>10000.0){
                data_ok = false;
                break;
            }
        }
        if(!data_ok){
            acc_mean = Eigen::Vector3d::Zero();
            gyro_mean = Eigen::Vector3d::Zero();
            std::cout << "[ESKF] IMU data abnormal, skipped integration\n";
        }

        // 旋转积分 (midpoint)
        Eigen::Matrix3d R_mid = R_ * ExpF64(gyro_mean * dt);

        // 世界坐标下加速度
        Eigen::Vector3d a_world = R_ * acc_mean + g_;

        // 位置、速度积分
        Eigen::Vector3d v_new = v_ + a_world * dt;
        Eigen::Vector3d p_new = p_ + v_ * dt + 0.5 * a_world * dt * dt;

        // 更新 nominal state
        R_ = R_mid;
        v_ = v_new;
        p_ = p_new;

        // 更新雅可比 F 预测协方差
        Eigen::Matrix3d hat_acc;
        hat_acc << 0.0, -acc_mean(2), acc_mean(1),
                acc_mean(2), 0.0, -acc_mean(0),
                -acc_mean(1), acc_mean(0), 0.0;

        Eigen::Matrix<double, 18, 18> F = Eigen::Matrix<double, 18, 18>::Identity();
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3,3>(3,6) = -R_ * hat_acc * dt;
        F.block<3,3>(3,12) = -R_ * dt;
        F.block<3,3>(3,15) = Eigen::Matrix3d::Identity() * dt;
        F.block<3,3>(6,6) = ExpF64(-gyro_mean * dt);
        F.block<3,3>(6,9) = -Eigen::Matrix3d::Identity() * dt;

        cov_ = F * cov_ * F.transpose() + Q_;

        // 下一步积分
        acc_prev = acc_curr;
        gyro_prev = gyro_curr;
    }

    last_timestamp = timestamp;
    acc_last_ = acc_raw - ba_;
    gyro_last_ = gyro_raw - bg_;
}



void ErrorStateKalmanFilter::correct(pose_type &pose){
  Eigen::Vector3d obv_pos = pose.pos.cast<double>();
  Eigen::Vector3d obv_rpy = pose.orient.cast<double>();

  //std::cout << "eskf corr: here 1.0" << std::endl;
  if (first_obv_){
    R_ = ExpF64(obv_rpy);
    p_ = obv_pos;
    first_obv_ = false;
    Eigen::Vector3d gyro_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_last_ = -options_.g_;

    return;
  }

  // 既有旋转，也有平移
  // 观测状态变量中的p, R，H为6x18，其余为零
  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // P部分
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // R部分（3.66）

  // 卡尔曼增益和更新过程
  Eigen::Matrix<double,6,1> noise_vec;
  noise_vec << options_.obv_pos_noise_, options_.obv_pos_noise_, options_.obv_pos_noise_, options_.obv_ang_noise_, options_.obv_ang_noise_, options_.obv_ang_noise_;

  Eigen::Matrix<double,6,6> V = noise_vec.asDiagonal();
  Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

  // 更新x和cov
  Eigen::Matrix<double,6,1> innov = Eigen::Matrix<double,6,1>::Zero();
  innov.head<3>() = (obv_pos - p_);                          // 平移部分
  innov.tail<3>() = SO3_LOGF64(R_.inverse() * ExpF64(obv_rpy));  // 旋转部分(3.67)

  dx_ = K * innov;
  cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_;

  /// 更新名义状态变量，重置error state
  p_ += dx_.block<3, 1>(0, 0);
  v_ += dx_.block<3, 1>(3, 0);
  R_ = R_ * ExpF64(dx_.block<3, 1>(6, 0));

  if (options_.update_bias_gyro_) {
    bg_ += dx_.block<3, 1>(9, 0);
  }

  if (options_.update_bias_acce_) {
    ba_ += dx_.block<3, 1>(12, 0);
  }

  g_ += dx_.block<3, 1>(15, 0);


  // 对P阵进行投影，参考式(3.63)
  Eigen::Matrix<double, 18, 18> J = Eigen::Matrix<double, 18, 18>::Identity();
  J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * SkewSymMatrixF64(dx_.block<3, 1>(6,0));
  cov_ = J * cov_ * J.transpose();

  dx_.setZero();

  pose.pos = p_.cast<float>();
  pose.orient = SO3_LOGF64(R_).cast<float>();

  return;

}



void ErrorStateKalmanFilter::BuildNoise(void) {
  double ev = options_.acce_var_;
  double et = options_.gyro_var_;
  double eg = options_.bias_gyro_var_;
  double ea = options_.bias_acce_var_;

  double ev2 = ev;  // * ev;
  double et2 = et;  // * et;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // 设置过程噪声
  Q_ = Eigen::Matrix<double, 18, 18>::Zero();
  Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

  // 设置GNSS状态
  double p2 = options_.obv_pos_noise_ * options_.obv_pos_noise_;
  double a2 = options_.obv_ang_noise_ * options_.obv_ang_noise_;
  obv_noise_.diagonal() << p2, p2, p2, a2, a2, a2;
}



