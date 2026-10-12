#pragma once                              // 头文件保护，防止被多重包含
#include <Eigen/Eigen>                    // 引入 Eigen 库（线性代数矩阵/向量等）
#include <sophus/so3.hpp>                 // 引入 Sophus 的 SO3（旋转）支持
#include "commons.h"                      // 引入自定义公共头（可能定义了 M3D,V3D,PointType 等类型）
#include <rclcpp/rclcpp.hpp>
// 类型别名，简化矩阵/向量类型书写（基于 double 精度）
using M12D = Eigen::Matrix<double, 12, 12>;   // 12x12 矩阵类型别名
using M21D = Eigen::Matrix<double, 21, 21>;   // 21x21 矩阵类型别名

using V12D = Eigen::Matrix<double, 12, 1>;    // 12x1 向量类型别名
using V21D = Eigen::Matrix<double, 21, 1>;    // 21x1 向量类型别名
using M21X12D = Eigen::Matrix<double, 21, 12>; // 21x12 矩阵类型别名

M3D Jr(const V3D &inp);                    // 声明：返回左雅可比的转置（或称 Jr），inp 为 so(3) 向量
M3D JrInv(const V3D &inp);                 // 声明：返回左雅可比逆的转置（或称 JrInv），inp 为 so(3) 向量

// 共享数据结构：用于在 update 迭代中传递观测残差 / 雅可比 等信息
struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW      // Eigen 对齐宏，确保 SSE/AVX 对齐安全
    M12D H;                              // 观测雅可比组成的信息矩阵（12x12）
    V12D b;                              // 观测信息向量（12x1）
    double res = 1e10;                   // 残差值（初始化为一个很大的数）
    bool valid = false;                  // 标记当前 shared 数据是否有效
    size_t iter_num = 0;                 // 记录迭代次数（或用于其它计数）
};    

// IMU 输入结构体，包含加速度与角速度
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW      // Eigen 对齐宏（以防内部使用 Eigen 类型）
    V3D acc;                             // 加速度观测（body frame）
    V3D gyro;                            // 角速度观测（body frame）
    Input() = default;                   // 默认构造函数
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}  
                                         // 通过向量引用构造（注意：参数不是 const 引用，按原代码保持）
    Input(double a1, double a2, double a3, double g1, double g2, double g3) 
        : acc(a1, a2, a3), gyro(g1, g2, g3) {}  
                                         // 通过标量直接构造 acc 和 gyro 的构造函数
};

// 状态结构体：包含 IMU-LiDAR 的位姿、速度、偏置以及重力方向
struct State
{
    static double gravity;               // 静态重力常量（用于统一重力大小）
    M3D r_wi = M3D::Identity();          // 世界到IMU的旋转矩阵（3x3，默认单位阵）
    V3D t_wi = V3D::Zero();              // 世界到IMU的平移（位置）
    M3D r_il = M3D::Identity();          // LiDAR 到 IMU 的旋转矩阵（外参旋转）
    V3D t_il = V3D::Zero();              // LiDAR 到 IMU 的平移（外参平移）
    V3D v = V3D::Zero();                 // IMU 在世界系中的速度
    V3D bg = V3D::Zero();                // 陀螺仪偏置
    V3D ba = V3D::Zero();                // 加速度计偏置
    V3D g = V3D(0.0, 0.0, -9.81);        // 重力矢量（默认朝 -Z）

    void initGravityDir(const V3D &gravity_dir)
    {
        g = gravity_dir.normalized() * State::gravity;  
        // 根据给定方向初始化重力向量（只设置方向，大小使用静态 gravity）
        RCLCPP_INFO(rclcpp::get_logger("IMU Init"), "g = %f %f %f", g.x(), g.y(), g.z());
        // 输出初始化后的重力向量（便于调试）
    }

    void operator+=(const V21D &delta); // 声明：状态加法运算（在 cpp 中实现）

    V21D operator-(const State &other) const; // 声明：状态差运算（在 cpp 中实现）

    friend std::ostream &operator<<(std::ostream &os, const State &state); 
    // 声明：友元输出操作符（用于打印状态，实现在 cpp）
};

// 函数类型别名：损失函数与停止条件函数
using loss_func = std::function<void(State &, SharedState &)>;  
// loss_func：接受当前状态与 SharedState 引用，填充观测相关的 H, b, res, valid 等
using stop_func = std::function<bool(const V21D &)>;           
// stop_func：接受当前增量（delta），返回是否应停止迭代（true 表示停止）

// IESKF 类：迭代式扩展卡尔曼滤波器（或类似结构）
class IESKF
{
public:
    IESKF() = default;                    // 默认构造函数

    void setMaxIter(size_t iter) { m_max_iter = iter; }  
    // 设置最大迭代次数（用于 update 内部迭代）

    void setLossFunction(loss_func func) { m_loss_func = func; }  
    // 设置外部提供的损失/观测函数（用于构造 H, b）

    void setStopFunction(stop_func func) { m_stop_func = func; }  
    // 设置停止条件函数（判断增量收敛）

    void predict(const Input &inp, double dt, const M12D &Q);  
    // 预测步骤：基于 IMU 输入、时间间隔和过程噪声 Q 更新状态和协方差

    void update();                        // 更新步骤：基于观测（通过 m_loss_func 提供）做迭代优化

    State &x() { return m_x; }            // 访问当前状态（可读写）

    M21D &P() { return m_P; }             // 访问/修改当前协方差矩阵

private:
    size_t m_max_iter = 10;               // 最大迭代次数，默认 10
    State m_x;                            // 当前状态（21 维）
    M21D m_P;                             // 当前状态协方差矩阵（21x21）
    loss_func m_loss_func;                // 损失函数回调（由外部设置）
    stop_func m_stop_func;                // 停止条件回调（由外部设置）
    M21D m_F;                             // 状态转移矩阵（21x21）
    M21X12D m_G;                          // 过程噪声矩阵（21x12）
};
