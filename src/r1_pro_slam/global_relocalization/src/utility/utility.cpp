#include "utility.h"

/*rpy加位移转4x4矩阵*/
Eigen::Matrix4f Pose2Matrix(pose_type pose) {
    Eigen::Matrix4f res;

    // 从姿态角 orient 中提取欧拉角（roll, pitch, yaw）并计算对应的正余弦值
    float s_r = sinf(pose.orient(0));  // roll 的正弦值
    float c_r = cosf(pose.orient(0));  // roll 的余弦值
    float s_p = sinf(pose.orient(1));  // pitch 的正弦值
    float c_p = cosf(pose.orient(1));  // pitch 的余弦值
    float s_y = sinf(pose.orient(2));  // yaw 的正弦值
    float c_y = cosf(pose.orient(2));  // yaw 的余弦值

    // 绕 X 轴的旋转矩阵（Roll）
    Eigen::Matrix3f R_x;
    R_x <<  1,  0,    0,
            0,  c_r, -s_r,
            0,  s_r,  c_r;

    // 绕 Y 轴的旋转矩阵（Pitch）
    Eigen::Matrix3f R_y;
    R_y <<  c_p, 0,  s_p,
            0,   1,  0,
            -s_p, 0,  c_p;

    // 绕 Z 轴的旋转矩阵（Yaw）
    Eigen::Matrix3f R_z;
    R_z <<  c_y, -s_y, 0,
            s_y,  c_y, 0,
            0,    0,   1;

    // 最终的旋转矩阵，按 ZYX 顺序组合（先绕 X，再 Y，最后 Z）
    Eigen::Matrix3f R = R_z * R_y * R_x;

    // 将旋转矩阵填入仿射变换矩阵的左上角（3x3）
    res.block<3,3>(0,0) = R;

    // 将平移向量填入仿射变换矩阵的最后一列（前三个元素）
    res(0,3) = pose.pos(0);  // x 坐标
    res(1,3) = pose.pos(1);  // y 坐标
    res(2,3) = pose.pos(2);  // z 坐标

    // 最后一行是仿射矩阵的标准形式：[0 0 0 1]
    res(3,0) = 0.0;
    res(3,1) = 0.0;
    res(3,2) = 0.0;
    res(3,3) = 1.0;

    // 返回最终的 4x4 仿射变换矩阵
    return res;
}


pose_type Matrix2Pose(Eigen::Matrix4f mat) {

    pose_type res;
    res.pos(0) = mat(0,3);
    res.pos(1) = mat(1,3);
    res.pos(2) = mat(2,3);

    Eigen::Matrix3f rotSO3;

    rotSO3 << mat(0,0), mat(0,1), mat(0,2),
            mat(1,0), mat(1,1), mat(1,2),
            mat(2,0), mat(2,1), mat(2,2);

    Eigen::Quaternionf q(rotSO3);

    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    res.orient(0) = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        res.orient(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        res.orient(1) = std::asin(sinp);
 
    // yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    res.orient(2) = std::atan2(siny_cosp, cosy_cosp);

    res.q[0] = qw;
    res.q[1] = qx;
    res.q[2] = qy;
    res.q[3] = qz;
 
    return res;
}

void update_q(pose_type &pose){
    float s_r = sinf(pose.orient(0)/2.0);
    float c_r = cosf(pose.orient(0)/2.0);
    float s_p = sinf(pose.orient(1)/2.0);
    float c_p = cosf(pose.orient(1)/2.0);
    float s_y = sinf(pose.orient(2)/2.0);
    float c_y = cosf(pose.orient(2)/2.0);


    pose.q[0] = c_r * c_p * c_y + s_r * s_p * s_y; //qw
    pose.q[1] = s_r * c_p * c_y - c_r * s_p * s_y; //qx
    pose.q[2] = c_r * s_p * c_y + s_r * c_p * s_y; //qy
    pose.q[3] = c_r * c_p * s_y - s_r * s_p * c_y; //qz
}

Eigen::Quaternion<double> rpy2q(double roll, double pitch, double yaw){
    double s_r = sin(roll/2.0);
    double c_r = cos(roll/2.0);
    double s_p = sin(pitch/2.0);
    double c_p = cos(pitch/2.0);
    double s_y = sin(yaw/2.0);
    double c_y = cos(yaw/2.0);

    double qx = s_r * c_p * c_y - c_r * s_p * s_y;
    double qy = c_r * s_p * c_y + s_r * c_p * s_y;
    double qz = c_r * c_p * s_y - s_r * s_p * c_y;
    double qw = c_r * c_p * c_y + s_r * s_p * s_y;

    Eigen::Quaternion<double> res(qw, qx, qy, qz);

    return res;
}


Eigen::Vector3d q2rpy(double qx, double qy, double qz, double qw){
    Eigen::Vector3d res;

    // roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    res(0) = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        res(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        res(1) = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    res(2) = atan2(siny_cosp, cosy_cosp);

    return res;
}


Eigen::Matrix3f vec_to_hat(Eigen::Vector3f omega)
{
    Eigen::Matrix3f res_mat_33;
    res_mat_33.setZero();
    res_mat_33(0, 1) = -omega(2);
    res_mat_33(1, 0) = omega(2);
    res_mat_33(0, 2) = omega(1);
    res_mat_33(2, 0) = -omega(1);
    res_mat_33(1, 2) = -omega(0);
    res_mat_33(2, 1) = omega(0);
    return res_mat_33;
}


pcl::PointCloud<PointType>::Ptr cloud_transform(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Matrix4f mat)
{
    pcl::PointCloud<PointType>::Ptr res(new pcl::PointCloud<PointType>);

    res->width = cloud->points.size();
    res->height = 1;
    res->points.resize(cloud->points.size());

    for(uint32_t i=0; i<cloud->points.size(); i++){
        PointType pt = cloud->points[i];

        res->points[i].x = mat(0,0) * pt.x + mat(0,1) * pt.y + mat(0,2) * pt.z + mat(0,3);
        res->points[i].y = mat(1,0) * pt.x + mat(1,1) * pt.y + mat(1,2) * pt.z + mat(1,3);
        res->points[i].z = mat(2,0) * pt.x + mat(2,1) * pt.y + mat(2,2) * pt.z + mat(2,3);
        res->points[i].intensity = cloud->points[i].intensity;
    }
    return res;
}

double normalize_angle(double angle)
{
    double tmp_angle=angle;

    while (tmp_angle>MY_PI){
        tmp_angle-=2.0*MY_PI;
    }

    while (tmp_angle<-MY_PI){
        tmp_angle+=2.0*MY_PI;
    }

    return tmp_angle;
}

float dist(PointType p)
{
    return sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
}

Eigen::Matrix3f SkewSymMatrix(Eigen::Vector3f vec)
{
    Eigen::Matrix3f SkewMat;

    SkewMat << 0.0,     -vec[2],  vec[1],
               vec[2],  0.0,     -vec[0],
               -vec[1], vec[0],   0.0;

    return SkewMat;
}

Eigen::Matrix3d SkewSymMatrixF64(Eigen::Vector3d vec)
{
    Eigen::Matrix3d SkewMat;

    SkewMat << 0.0,     -vec[2],  vec[1],
               vec[2],  0.0,     -vec[0],
               -vec[1], vec[0],   0.0;

    return SkewMat;
}


Eigen::Matrix3f Exp(Eigen::Vector3f ang)
{
    float ang_norm = ang.norm();
    Eigen::Matrix3f Eye3 = Eigen::Matrix<float, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Vector3f r_axis = ang / ang_norm;
        Eigen::Matrix3f K;
        K << SkewSymMatrix(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}


// RPY (roll, pitch, yaw) 到旋转矩阵
Eigen::Matrix3f RPY2Mat(const Eigen::Vector3f &rpy) {
    float roll  = rpy.x();
    float pitch = rpy.y();
    float yaw   = rpy.z();

    Eigen::AngleAxisf rx(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz(yaw,   Eigen::Vector3f::UnitZ());

    return (rz * ry * rx).toRotationMatrix();   // ZYX 顺序
}


Eigen::Matrix3d ExpF64(Eigen::Vector3d ang){
    double ang_norm = ang.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Vector3d r_axis = ang / ang_norm;
        Eigen::Matrix3d K;
        K << 0.0,    -r_axis[2],  r_axis[1],
             r_axis[2],  0.0,     -r_axis[0],
            -r_axis[1],  r_axis[0],  0.0;

        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}


/* Logrithm of a Rotation Matrix */
Eigen::Vector3f SO3_LOG(Eigen::Matrix3f R){
    float theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Vector3f K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Vector3d SO3_LOGF64(Eigen::Matrix3d R)
{
    double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Vector3d K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}



Eigen::Vector3f RotMtoEuler(Eigen::Matrix3f rot)
{
    float sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    float x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Vector3f ang(x, y, z);
    return ang;
}



//旋转矩阵雅阁比右乘
Eigen::Matrix3f RightJacobianRotionMatrix(Eigen::Vector3f omega)
{
    Eigen::Matrix3f res_mat_33;

    float theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix3f::Identity();
    Eigen::Vector3f a = omega/ theta;
    Eigen::Matrix3f hat_a = vec_to_hat(a);
    res_mat_33 = sin(theta)/theta * Eigen::Matrix3f::Identity()
                    + (1 - (sin(theta)/theta))*a*a.transpose() 
                    + ((1 - cos(theta))/theta)*hat_a;

    return res_mat_33;
}

//旋转矩阵雅阁比右乘的逆
Eigen::Matrix3f InverseRightJacobianRotionMatrix(Eigen::Vector3f omega)
{

    Eigen::Matrix3f res_mat_33;

    float theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix3f::Identity();
    Eigen::Vector3f a = omega/ theta;
    Eigen::Matrix3f hat_a = vec_to_hat(a);
    res_mat_33 = (theta / 2) * (1.0/tan(theta / 2)) * Eigen::Matrix3f::Identity() 
                + (1 - (theta / 2) * (1.0/tan(theta / 2))) * a * a.transpose() 
                + (theta / 2) * hat_a;

    return res_mat_33;
}



void print_pose(pose_type pose)
{
    std::cout << "========================" << std::endl;
    std::cout << "pos : " << pose.pos << std::endl;
    std::cout << "rpy : " << pose.orient << std::endl;
    std::cout << "========================" << std::endl;
}


// 从ROS2消息的Header中提取时间戳（单位：秒）
double getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

// 将double类型的秒数转换为ROS2的builtin_interfaces::msg::Time格式
builtin_interfaces::msg::Time getTime(const double& sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
