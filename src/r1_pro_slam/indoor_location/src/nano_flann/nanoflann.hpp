/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * 允许在符合条件的情况下自由使用、修改和分发该软件，
 * 不提供任何担保，使用者自负风险。
 *************************************************************************/

#ifndef NANOFLANN_HPP_
#define NANOFLANN_HPP_

#include <memory>                       // 使用 std::shared_ptr 管理内存
#include <thread>                       // 使用 thread_local 优化线程安全
#include <pcl/point_cloud.h>           // PCL 点云数据结构
#include <pcl/point_types.h>           // PCL 点类型定义
#include <pcl/kdtree/kdtree_flann.h>   // 为了兼容性引用（不直接使用）
#include "nanoflann_impl.hpp"          // nanoflann 实际实现（包含 KDTree adaptor 等）

namespace nanoflann
{

// 模板类：使用 PCL 点云构建 nanoflann KDTree 接口
    template <typename PointT>
    class KdTreeFLANN
    {
    public:
        // 类型别名定义
        typedef typename pcl::PointCloud<PointT> PointCloud;
        typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
        typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

        typedef std::shared_ptr<KdTreeFLANN<PointT>> Ptr;
        typedef std::shared_ptr<const KdTreeFLANN<PointT>> ConstPtr;
        typedef std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef std::shared_ptr<const std::vector<int>> IndicesConstPtr;

        // 构造函数：可设置是否排序结果
        KdTreeFLANN(bool sorted = false);

        // 拷贝构造函数
        KdTreeFLANN(const KdTreeFLANN<PointT>& k);

        // 设置搜索时的误差 epsilon（默认为 0，表示精确）
        void setEpsilon(float eps);

        // 设置搜索结果是否排序（默认关闭）
        void setSortedResults(bool sorted);

        // 返回当前对象的 shared_ptr 版本
        inline Ptr makeShared() { return Ptr(new KdTreeFLANN<PointT>(*this)); }

        // 设置输入点云以及可选的索引（对部分点进行建树）
        void setInputCloud(const PointCloudConstPtr& cloud,
                           const IndicesConstPtr& indices = IndicesConstPtr());

        // 获取当前设置的点云
        inline PointCloudConstPtr getInputCloud() const { return _adaptor.pcl; }

        // 最近邻搜索：查找与点 point 距离最近的 k 个点
        int nearestKSearch(const PointT& point, int k,
                           std::vector<int>& k_indices,
                           std::vector<float>& k_sqr_distances) const;

        // 半径搜索：查找距离 point 不超过 radius 的所有点
        int radiusSearch(const PointT& point, double radius,
                         std::vector<int>& k_indices,
                         std::vector<float>& k_sqr_distances) const;

    protected:
        // nanoflann 的搜索参数结构体（包括 eps、是否排序等）
        nanoflann::SearchParams _params;

        // 适配器：将 PCL 点云数据转换为 nanoflann 所需格式
        struct PointCloud_Adaptor
        {
            inline size_t kdtree_get_point_count() const; // 获取点的总数
            inline float kdtree_get_pt(const size_t idx, int dim) const; // 获取 idx 点的第 dim 个坐标值
            template <class BBOX>
            bool kdtree_get_bbox(BBOX&) const { return false; } // 不启用边界框加速
            PointCloudConstPtr pcl;     // 点云指针
            IndicesConstPtr indices;    // 可选的索引指针（若仅对部分点建树）
        };

        // 定义 nanoflann 的 KDTree 类型，使用 SO3 适配器表示 3D 空间
        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::SO3_Adaptor<float, PointCloud_Adaptor>,
                PointCloud_Adaptor,
                3, // 空间维度
                int> KDTreeFlann_PCL_SO3;

        PointCloud_Adaptor _adaptor;   // 点云访问适配器
        KDTreeFlann_PCL_SO3 _kdtree;   // nanoflann 的 KD 树结构
    };

// ================= 实现区域 ======================

// 构造函数：初始化 KDTree，设置维度 3、缓冲大小 100
    template<typename PointT> inline
    KdTreeFLANN<PointT>::KdTreeFLANN(bool sorted)
            : _kdtree(3, _adaptor, KDTreeSingleIndexAdaptorParams(100))
    {
        _params.sorted = sorted;
    }

// 设置 epsilon 搜索精度参数
    template<typename PointT> inline
    void KdTreeFLANN<PointT>::setEpsilon(float eps)
    {
        _params.eps = eps;
    }

// 设置是否返回排序后的搜索结果
    template<typename PointT> inline
    void KdTreeFLANN<PointT>::setSortedResults(bool sorted)
    {
        _params.sorted = sorted;
    }

// 设置输入点云及索引，并构建 KDTree
    template<typename PointT> inline
    void KdTreeFLANN<PointT>::setInputCloud(const PointCloudConstPtr& cloud,
                                            const IndicesConstPtr& indices)
    {
        _adaptor.pcl = cloud;
        _adaptor.indices = indices;
        _kdtree.buildIndex();  // 构建 KD 树索引结构
    }

// 最近邻搜索实现：查找距离 point 最近的 num_closest 个点
    template<typename PointT> inline
    int KdTreeFLANN<PointT>::nearestKSearch(const PointT& point, int num_closest,
                                            std::vector<int>& k_indices,
                                            std::vector<float>& k_sqr_distances) const
    {
        k_indices.resize(num_closest);
        k_sqr_distances.resize(num_closest);

        nanoflann::KNNResultSet<float, int> resultSet(num_closest);
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        _kdtree.findNeighbors(resultSet, point.data, nanoflann::SearchParams());
        return static_cast<int>(resultSet.size());  // 返回找到的点数
    }

// 半径搜索实现：查找距离 point 不超过 radius 的所有点
    template<typename PointT> inline
    int KdTreeFLANN<PointT>::radiusSearch(const PointT& point, double radius,
                                          std::vector<int>& k_indices,
                                          std::vector<float>& k_sqr_distances) const
    {
        // 使用线程局部存储，避免多线程数据竞争
        thread_local std::vector<std::pair<int, float>> indices_dist;
        indices_dist.clear();  // 清空上次的结果
        if (indices_dist.capacity() < 128) {
            indices_dist.reserve(128);  // 预留空间以减少内存分配
        }

        RadiusResultSet<float, int> resultSet(radius, indices_dist);
        const size_t nFound = _kdtree.findNeighbors(resultSet, point.data, _params);

        // 如果需要排序，则按照距离排序
        if (_params.sorted)
            std::sort(indices_dist.begin(), indices_dist.end(), IndexDist_Sorter());

        k_indices.resize(nFound);
        k_sqr_distances.resize(nFound);
        for (size_t i = 0; i < nFound; ++i) {
            k_indices[i]       = indices_dist[i].first;
            k_sqr_distances[i] = indices_dist[i].second;
        }
        return static_cast<int>(nFound);  // 返回找到的邻居数量
    }

// 返回点数量：若有索引则返回索引长度，否则返回点云数量
    template<typename PointT> inline
    size_t KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_point_count() const {
        if (indices) return indices->size();
        if (pcl)     return pcl->points.size();
        return 0;
    }

// 获取点的第 dim 个维度的值（0=x, 1=y, 2=z）
    template<typename PointT> inline
    float KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_pt(const size_t idx, int dim) const {
        const PointT& p = (indices) ? pcl->points[(*indices)[idx]] : pcl->points[idx];
        if (dim == 0) return p.x;
        if (dim == 1) return p.y;
        if (dim == 2) return p.z;
        return 0.0f;  // 超出维度时默认返回 0
    }

} // namespace nanoflann

#endif // NANOFLANN_HPP_
