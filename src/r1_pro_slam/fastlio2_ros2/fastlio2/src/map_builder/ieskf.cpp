#include "ieskf.h"  
// 引入头文件，定义了 State 和 IESKF 类的接口、别名（M3D,V3D,V21D 等）

double State::gravity = 9.81;  
// 静态成员变量，重力加速度常量，单位 m/s^2

// ---------------- 辅助函数 ----------------

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
// 计算李代数 SO(3) 的左雅可比矩阵的转置，用于旋转增量扰动传播

M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}
// 计算左雅可比逆矩阵的转置，用于旋转误差反映到状态差分

// ---------------- State 的运算符重载 ----------------

void State::operator+=(const V21D &delta)
{
    // delta 向量共有 21 维，依次更新各个子状态

    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    // 前3维是旋转扰动 (SO(3))，更新世界到IMU的旋转矩阵

    t_wi += delta.segment<3>(3);
    // 接下来的3维是平移扰动，更新位置

    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    // 接下来的3维是 LiDAR相对IMU的旋转扰动

    t_il += delta.segment<3>(9);
    // 接下来的3维是 LiDAR相对IMU的平移扰动

    v += delta.segment<3>(12);
    // 接下来的3维是速度扰动

    bg += delta.segment<3>(15);
    // 接下来的3维是陀螺仪偏置扰动

    ba += delta.segment<3>(18);
    // 接下来的3维是加速度计偏置扰动
}

V21D State::operator-(const State &other) const
{
    // 定义两个 State 之间的差异运算，返回 21维增量
    V21D delta = V21D::Zero();

    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    // 旋转误差 = log(R_other^T * R_current)，映射到so(3)

    delta.segment<3>(3) = t_wi - other.t_wi;
    // 位置差

    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    // LiDAR-IMU 旋转差

    delta.segment<3>(9) = t_il - other.t_il;
    // LiDAR-IMU 平移差

    delta.segment<3>(12) = v - other.v;
    // 速度差

    delta.segment<3>(15) = bg - other.bg;
    // 陀螺仪偏置差

    delta.segment<3>(18) = ba - other.ba;
    // 加速度计偏置差

    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    // 重载输出运算符，打印 State 的所有成员
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    // 世界到IMU旋转（欧拉角ZYX顺序）

    os << "t_il: " << state.t_il.transpose() << std::endl;
    // IMU到LiDAR平移（外参平移）

    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    // IMU到LiDAR旋转（外参旋转）

    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    // 世界到IMU平移（IMU在世界系中的位置）

    os << "v: " << state.v.transpose() << std::endl;
    // 速度

    os << "bg: " << state.bg.transpose() << std::endl;
    // 陀螺仪偏置

    os << "ba: " << state.ba.transpose() << std::endl;
    // 加速度计偏置

    os << "g: " << state.g.transpose() << std::endl;
    // 重力向量

    os << "===============END================" << std::endl;

    return os;
}

// ---------------- IESKF 的预测步骤 ----------------

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    // inp：IMU输入（陀螺仪gyro，加速度acc,dt：时间间隔,Q：过程噪声协方差矩阵 (12x12)
    
    // 初始化状态增量为0
    V21D delta = V21D::Zero();

    // 姿态增量 = 去偏置后的角速度 * dt
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt; 

    // 位置增量 = 当前速度 * dt
    delta.segment<3>(3) = m_x.v * dt;

    // 速度增量 = (旋转后的加速度 + 重力) * dt
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    // 状态转移矩阵 F
    m_F.setIdentity();

    // 姿态状态转移部分
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();

    // 姿态对陀螺仪偏置的偏导
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;

    // 位置对速度的偏导
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;

    // 速度对姿态的偏导
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;

    // 速度对加速度计偏置的偏导
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    // 控制输入矩阵 G
    m_G.setZero();
    // 姿态噪声
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;

    // 加速度噪声
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;

    // 陀螺仪偏置随机游走
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;

    // 加速度偏置随机游走
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    // 更新状态
    m_x += delta;

    // 更新协方差 P
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

// ---------------- IESKF 的更新步骤 ----------------

void IESKF::update()
{
    State predict_x = m_x;
    // 保存预测状态

    SharedState shared_data;
    // 共享结构，存放残差、雅可比、是否有效

    shared_data.iter_num = 0;
    shared_data.res = 1e10;

    V21D delta = V21D::Zero();
    M21D H = M21D::Identity();
    // 信息矩阵（初始单位）

    V21D b;
    // 信息向量

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        // 调用损失函数计算观测残差与雅可比

        if (!shared_data.valid)
            break;

        H.setZero();
        b.setZero();

        delta = m_x - predict_x;
        // 当前估计与预测值的差

        M21D J = M21D::Identity();
        // 雅可比修正矩阵

        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        // 姿态扰动的雅可比逆

        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        // LiDAR-IMU 姿态扰动的雅可比逆

        H += J.transpose() * m_P.inverse() * J;
        b += J.transpose() * m_P.inverse() * delta;
        // 高斯牛顿迭代：增量方程

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;
        // 加入观测信息矩阵和向量

        delta = -H.inverse() * b;
        // 解算增量

        m_x += delta;
        // 状态更新

        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
        // 如果满足停止条件（收敛），退出迭代
    }

    M21D L = M21D::Identity();
    // 最终线性化修正矩阵

    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    // 这里选择用 Jr来保持数值稳定性

    m_P = L * H.inverse() * L.transpose();
    // 最终更新状态协方差
}
