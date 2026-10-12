#pragma once
#include "commons.h"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <random>
#include <string>
#include <vector>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;

    int global_candidates = 64;
    int global_min_inliers = 20;
    double global_voxel_resolution = 0.30;
    double global_max_correspondence_distance = 1.5;
};

struct GlobalAlignResult
{
    bool success = false;
    M4F pose = M4F::Identity();
    double score = 0.0;
    std::string method = "none";
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path);
    void setInput(const CloudType::Ptr &cloud);
    bool align(M4F &guess, double *score = nullptr);
    GlobalAlignResult globalAlign(int candidates, int64_t seed);

    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }

private:
    GlobalAlignResult smallGicpFrom(const M4F &initial);
    GlobalAlignResult refinePatch(const CloudType::Ptr &patch, const M4F &initial);
    GlobalAlignResult randomSearch(int candidates, int64_t seed);

    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    pcl::IterativeClosestPoint<PointType, PointType> m_refine_icp;
    pcl::IterativeClosestPoint<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
    V3D m_map_min = V3D::Zero();
    V3D m_map_max = V3D::Zero();
    std::vector<CloudType::Ptr> m_patch_clouds;
    std::vector<M4F> m_patch_poses;
};