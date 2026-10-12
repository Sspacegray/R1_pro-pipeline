#include "commons.h"  
// 引入自定义的头文件 commons.h（里面可能定义了 PointType, PointVec, V3D, V4D 等类型）

bool esti_plane(PointVec &points, const double &thresh, V4D &out)  
// 函数 esti_plane：用于估计点集所在的平面方程
// 输入：points 点集合，thresh 阈值
// 输出：out = [a, b, c, d] 平面参数，返回是否估计成功
{
    Eigen::MatrixXd A(points.size(), 3);  
    // 创建矩阵 A，维度为 N×3，存储点的 x, y, z
    Eigen::MatrixXd b(points.size(), 1);  
    // 创建列向量 b，维度为 N×1，用于存储常数项

    A.setZero();  
    // 初始化矩阵 A 为 0
    b.setOnes();  
    // 初始化向量 b 为全 1
    b *= -1.0;  
    // 将 b 设置为全 -1（对应方程 ax+by+cz = -1）

    for (size_t i = 0; i < points.size(); i++)  
    // 遍历所有点
    {
        A(i, 0) = points[i].x;  
        // A 的第 1 列存放点的 x 坐标
        A(i, 1) = points[i].y;  
        // A 的第 2 列存放点的 y 坐标
        A(i, 2) = points[i].z;  
        // A 的第 3 列存放点的 z 坐标
    }

    V3D normvec = A.colPivHouseholderQr().solve(b);  
    // 使用最小二乘解 A*[a,b,c]^T = b，得到平面法向量 [a,b,c]
    // Eigen 采用列主元 Householder QR 分解求解

    double norm = normvec.norm();  
    // 计算法向量的模长（归一化用）

    out[0] = normvec(0) / norm;  
    // 平面法向量 a 分量（单位化）
    out[1] = normvec(1) / norm;  
    // 平面法向量 b 分量
    out[2] = normvec(2) / norm;  
    // 平面法向量 c 分量
    out[3] = 1.0 / norm;  
    // 平面常数项 d，这里等价于平面方程 ax+by+cz+d=0 中的 d

    for (size_t j = 0; j < points.size(); j++)  
    // 遍历所有点，检查拟合结果是否合理
    {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)  
        // 将点代入平面方程 |ax+by+cz+d|，判断点到平面的距离是否小于阈值
        {
            return false;  
            // 如果有点距离过大，说明拟合效果不好，返回失败
        }
    }

    return true;  
    // 所有点都在阈值范围内，平面拟合成功
}

float sq_dist(const PointType &p1, const PointType &p2)  
// 计算两点间的欧式距离平方
{
    return (p1.x - p2.x) * (p1.x - p2.x)  
         + (p1.y - p2.y) * (p1.y - p2.y)  
         + (p1.z - p2.z) * (p1.z - p2.z);  
    // 返回 (Δx^2 + Δy^2 + Δz^2)
}
