#include "icp_localizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
}

bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    if (reader.read(path, *cloud) != 0 || cloud->empty())
        return false;

    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else pcl::copyPointCloud(*cloud, *m_refine_tgt);

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else pcl::copyPointCloud(*cloud, *m_rough_tgt);

    PointType min_pt, max_pt;
    pcl::getMinMax3D(*m_rough_tgt, min_pt, max_pt);
    m_map_min = V3D(min_pt.x, min_pt.y, min_pt.z);
    m_map_max = V3D(max_pt.x, max_pt.y, max_pt.z);
    m_pcd_path = path;
    m_patch_clouds.clear();
    m_patch_poses.clear();
    const auto root = std::filesystem::path(path).parent_path();
    const auto poses_path = root / "poses.txt";
    const auto patches_path = root / "patches";
    if (std::filesystem::exists(poses_path) && std::filesystem::is_directory(patches_path))
    {
        std::ifstream poses(poses_path);
        std::string line;
        while (std::getline(poses, line))
        {
            std::stringstream ss(line);
            std::string name;
            float x, y, z, qw, qx, qy, qz;
            if (!(ss >> name >> x >> y >> z >> qw >> qx >> qy >> qz)) continue;
            auto patch = patches_path / name;
            if (!std::filesystem::exists(patch)) continue;
            CloudType::Ptr patch_cloud(new CloudType);
            if (pcl::io::loadPCDFile<PointType>(patch.string(), *patch_cloud) != 0 || patch_cloud->empty()) continue;
            M4F pose = M4F::Identity();
            pose.block<3, 3>(0, 0) = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
            pose.block<3, 1>(0, 3) = V3F(x, y, z);
            m_patch_clouds.push_back(patch_cloud);
            m_patch_poses.push_back(pose);
        }
    }
    return !m_refine_tgt->empty() && !m_rough_tgt->empty();
}

void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return;
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else pcl::copyPointCloud(*cloud, *m_refine_inp);

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else pcl::copyPointCloud(*cloud, *m_rough_inp);
}

bool ICPLocalizer::align(M4F &guess, double *score)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->empty() || m_rough_tgt->empty() || m_refine_inp->empty() || m_rough_inp->empty()) return false;
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setMaxCorrespondenceDistance(m_config.global_max_correspondence_distance);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    const double rough_score = m_rough_icp.getFitnessScore();
    if (!m_rough_icp.hasConverged() || rough_score > m_config.rough_score_thresh) return false;

    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setMaxCorrespondenceDistance(m_config.global_max_correspondence_distance);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    const double refine_score = m_refine_icp.getFitnessScore();
    if (score) *score = refine_score;
    if (!m_refine_icp.hasConverged() || refine_score > m_config.refine_score_thresh) return false;
    guess = m_refine_icp.getFinalTransformation();
    return true;
}

GlobalAlignResult ICPLocalizer::smallGicpFrom(const M4F &initial)
{
    GlobalAlignResult out;
    if (m_refine_inp->empty() || m_refine_tgt->empty()) return out;
    CloudType::Ptr transformed(new CloudType), aligned(new CloudType);
    pcl::transformPointCloud(*m_refine_inp, *transformed, initial);

    small_gicp::RegistrationPCL<PointType, PointType> reg;
    reg.setNumThreads(4);
    reg.setCorrespondenceRandomness(20);
    reg.setMaxCorrespondenceDistance(m_config.global_max_correspondence_distance);
    reg.setVoxelResolution(m_config.global_voxel_resolution);
    reg.setRegistrationType("VGICP");
    reg.setInputTarget(m_refine_tgt);
    reg.setInputSource(transformed);
    reg.align(*aligned);
    const auto &result = reg.getRegistrationResult();
    if (transformed->empty()) return out;
    const double overlap = static_cast<double>(result.num_inliers) / transformed->size();
    if (!result.converged || overlap < 0.50) return out;

    out.pose = reg.getFinalTransformation().cast<float>() * initial;
    pcl::transformPointCloud(*m_refine_inp, *aligned, out.pose);
    pcl::IterativeClosestPoint<PointType, PointType> verify;
    verify.setMaximumIterations(m_config.refine_max_iteration);
    verify.setMaxCorrespondenceDistance(m_config.global_max_correspondence_distance);
    verify.setInputSource(m_refine_inp);
    verify.setInputTarget(m_refine_tgt);
    verify.align(*transformed, out.pose);
    out.score = verify.getFitnessScore();
    out.pose = verify.getFinalTransformation();
    out.success = verify.hasConverged() && out.score <= m_config.refine_score_thresh;
    out.method = "kiss_matcher+small_gicp";
    return out;
}

GlobalAlignResult ICPLocalizer::refinePatch(const CloudType::Ptr &patch, const M4F &initial)
{
    GlobalAlignResult out;
    if (!patch || patch->empty() || m_refine_inp->empty()) return out;
    CloudType::Ptr target = patch;
    CloudType::Ptr aligned(new CloudType);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaximumIterations(std::max(30, m_config.refine_max_iteration * 3));
    icp.setMaxCorrespondenceDistance(m_config.global_max_correspondence_distance);
    icp.setInputSource(m_refine_inp);
    icp.setInputTarget(target);
    icp.align(*aligned, initial);
    const double score = icp.getFitnessScore();
    if (!icp.hasConverged() || score > m_config.refine_score_thresh) return out;
    out.success = true;
    out.pose = icp.getFinalTransformation();
    out.score = score;
    out.method = "kiss_matcher+small_gicp+patch_icp";
    return out;
}

GlobalAlignResult ICPLocalizer::randomSearch(int candidates, int64_t seed)
{
    GlobalAlignResult best;
    if (candidates <= 0) candidates = m_config.global_candidates;
    std::mt19937_64 rng(seed < 0 ? 42 : static_cast<uint64_t>(seed));
    std::uniform_real_distribution<float> dx(static_cast<float>(m_map_min.x()), static_cast<float>(m_map_max.x()));
    std::uniform_real_distribution<float> dy(static_cast<float>(m_map_min.y()), static_cast<float>(m_map_max.y()));
    std::uniform_real_distribution<float> dz(static_cast<float>(m_map_min.z()), static_cast<float>(m_map_max.z()));
    std::uniform_real_distribution<float> yaw(-static_cast<float>(M_PI), static_cast<float>(M_PI));

    double best_score = std::numeric_limits<double>::infinity();
    for (int i = 0; i < candidates; ++i)
    {
        M4F guess = M4F::Identity();
        guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw(rng), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        guess.block<3, 1>(0, 3) = V3F(dx(rng), dy(rng), dz(rng));
        double score = 0.0;
        if (align(guess, &score) && score < best_score)
        {
            best.success = true;
            best.pose = guess;
            best.score = score;
            best.method = "small_gicp_multistart_fallback";
            best_score = score;
        }
    }
    return best;
}

GlobalAlignResult ICPLocalizer::globalAlign(int candidates, int64_t seed)
{
    GlobalAlignResult out;
    if (m_refine_inp->empty() || m_refine_tgt->empty()) return out;

    std::vector<Eigen::Vector3f> src, tgt;
    src.reserve(m_rough_inp->size());
    tgt.reserve(m_rough_tgt->size());
    for (const auto &p : *m_rough_inp) src.emplace_back(p.x, p.y, p.z);
    for (const auto &p : *m_rough_tgt) tgt.emplace_back(p.x, p.y, p.z);

    const float voxel = static_cast<float>(m_config.global_voxel_resolution);
    kiss_matcher::KISSMatcherConfig config(voxel, false);
    struct Candidate { size_t inliers; M4F pose; size_t patch_index; };
    std::vector<Candidate> patch_candidates;
    size_t best_inliers = 0;
    M4F best_coarse = M4F::Identity();

    // Prefer the saved PGO submaps when available. This turns global search into
    // scan-to-keyframe retrieval instead of matching a small scan to an entire map.
    for (size_t i = 0; i < m_patch_clouds.size(); ++i)
    {
        std::vector<Eigen::Vector3f> patch;
        patch.reserve(m_patch_clouds[i]->size());
        for (const auto &p : *m_patch_clouds[i]) patch.emplace_back(p.x, p.y, p.z);
        if (patch.size() < 100) continue;
        kiss_matcher::KISSMatcher matcher(config);
        const auto solution = matcher.estimate(src, patch);
        const size_t inliers = matcher.getNumFinalInliers();
        if (solution.valid && inliers >= static_cast<size_t>(m_config.global_min_inliers))
        {
            M4F relative = M4F::Identity();
            relative.block<3, 3>(0, 0) = solution.rotation.cast<float>();
            relative.block<3, 1>(0, 3) = solution.translation.cast<float>();
            patch_candidates.push_back({inliers, m_patch_poses[i] * relative, i});
        }
    }

    // A full-map descriptor match is the fallback when no patch metadata exists.
    // Search overlapping spatial tiles before falling back to seeded ICP starts.
    if (m_patch_clouds.empty())
    for (double cx = m_map_min.x(); cx <= m_map_max.x(); cx += 8.0)
    {
        for (double cy = m_map_min.y(); cy <= m_map_max.y(); cy += 8.0)
        {
            std::vector<Eigen::Vector3f> tile;
            tile.reserve(m_rough_tgt->size() / 4);
            for (const auto &p : *m_rough_tgt)
            {
                if (std::hypot(static_cast<double>(p.x) - cx, static_cast<double>(p.y) - cy) < 14.0)
                    tile.emplace_back(p.x, p.y, p.z);
            }
            if (tile.size() < 100) continue;
            kiss_matcher::KISSMatcher matcher(config);
            const auto solution = matcher.estimate(src, tile);
            const size_t inliers = matcher.getNumFinalInliers();
            if (solution.valid && inliers >= best_inliers)
            {
                best_inliers = inliers;
                best_coarse.setIdentity();
                best_coarse.block<3, 3>(0, 0) = solution.rotation.cast<float>();
                best_coarse.block<3, 1>(0, 3) = solution.translation.cast<float>();
            }
        }
    }
    std::sort(patch_candidates.begin(), patch_candidates.end(), [](const Candidate &a, const Candidate &b) {
        return a.inliers > b.inliers;
    });
    if (!patch_candidates.empty())
    {
        best_inliers = patch_candidates.front().inliers;
        best_coarse = patch_candidates.front().pose;
        const size_t limit = std::min<size_t>(patch_candidates.size(), 16);
        GlobalAlignResult best_verified;
        for (size_t k = 0; k < limit; ++k)
        {
            CloudType::Ptr patch_global(new CloudType);
            pcl::transformPointCloud(*m_patch_clouds[patch_candidates[k].patch_index], *patch_global,
                                     m_patch_poses[patch_candidates[k].patch_index]);
            const auto verified = refinePatch(patch_global, patch_candidates[k].pose);
            if (verified.success && (!best_verified.success || verified.score < best_verified.score))
                best_verified = verified;
        }
        if (best_verified.success) return best_verified;
        out = smallGicpFrom(best_coarse);
        if (out.success) return out;
        double score = 0.0;
        M4F refined = best_coarse;
        if (align(refined, &score))
        {
            out.success = true;
            out.pose = refined;
            out.score = score;
            out.method = "kiss_matcher+pcl_icp";
            return out;
        }
        auto fallback = randomSearch(candidates, seed);
        if (!fallback.success)
            fallback.method = "kiss_matcher_refine_failed";
        return fallback;
    }
    if (best_inliers >= static_cast<size_t>(m_config.global_min_inliers))
    {
        out = smallGicpFrom(best_coarse);
        if (out.success) return out;
        double score = 0.0;
        M4F refined = best_coarse;
        if (align(refined, &score))
        {
            out.success = true;
            out.pose = refined;
            out.score = score;
            out.method = "kiss_matcher+pcl_icp";
            return out;
        }
        auto fallback = randomSearch(candidates, seed);
        if (!fallback.success)
            fallback.method = "kiss_matcher_refine_failed";
        return fallback;
    }
    auto fallback = randomSearch(candidates, seed);
    if (!fallback.success)
        fallback.method = "kiss_matcher_no_candidate";
    return fallback;
}
