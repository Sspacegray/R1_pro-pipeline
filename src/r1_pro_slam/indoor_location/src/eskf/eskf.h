#ifndef _ESKF_H_
#define _ESKF_H_

#include "utility.h"

struct eskf_options {
  eskf_options() = default;

  // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
  double gyro_var_ = 1e-5;       // 陀螺测量标准差
  double acce_var_ = 1e-2;       // 加计测量标准差
  double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
  double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

  /// 观测参数
  double obv_pos_noise_ = 0.1;            // 位置噪声
  double obv_ang_noise_ = 1.0 * Deg2Rad;  // 旋转噪声

  /// 其他配置
  bool update_bias_gyro_ = true;  // 是否更新陀螺bias
  bool update_bias_acce_ = true;  // 是否更新加计bias

  Eigen::Vector3d g_;

};

class ErrorStateKalmanFilter{
public:
  eskf_options options_;

  Eigen::Matrix<double, 6, 6> obv_noise_ = Eigen::Matrix<double, 6, 6>::Zero();
  ErrorStateKalmanFilter(std::string yaml_file_path);

  void reset(void);

  /*			setter			*/
  void setMean(Eigen::Vector3d pos, Eigen::Vector3d rpy, Eigen::Vector3d vel);
  void setCov(Eigen::Matrix<double, 18, 18> s);

  /*			getter			*/
  void getPose(pose_type &pose);
  Eigen::Matrix<double, 6, 1> getVel(void);

  void predict(Eigen::VectorXd& control, double timestamp);

  void correct(pose_type &pose);



private:
  // 名义状态
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d g_;

  // 误差状态
  Eigen::Matrix<double, 18, 1> dx_;

  double last_timestamp{0.0};  // 上一时刻时间戳

  // 协方差阵
  Eigen::Matrix<double, 18, 18> cov_;

  // 噪声阵
  Eigen::Matrix<double, 18, 18> Q_;

  // 标志位
  bool first_obv_{true};  // 是否为第一个观测数据

  //上一时刻imu的值
  Eigen::Vector3d gyro_last_;
  Eigen::Vector3d acc_last_;

  void BuildNoise(void);

};







#endif