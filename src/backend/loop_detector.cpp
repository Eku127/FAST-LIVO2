/*
 * GICP-based loop closure detection implementation using small_gicp
 *
 * Replaces PCL ICP with small_gicp for better stability and performance.
 * small_gicp provides:
 * - Optimized GICP/VGICP algorithms (up to 2x faster than fast_gicp)
 * - Better numerical stability
 * - Parallel processing support (OpenMP/TBB)
 *
 * Point cloud allocation: use pcl::make_shared<>() instead of new so allocation
 * and deallocation use PCL's aligned allocator (Eigen-compatible).
 *
 * Updated to use Options pattern and SE3 poses from keyframe_types.h
 */

#include "backend/loop_detector.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>

#include <Eigen/SVD>
#include <pcl/memory.h>

namespace {
/// Orthogonalize a 3x3 matrix to a valid rotation (SO3) via SVD. GICP can return
/// slightly non-orthogonal matrices; Sophus::SE3 requires a proper rotation.
Eigen::Matrix3d orthogonalizeRotation(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d out = svd.matrixU() * svd.matrixV().transpose();
    if (out.determinant() < 0) out.col(2) *= -1;
    return out;
}
}  // namespace

// small_gicp includes
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling.hpp>

namespace livo2_offline {

LoopDetector::LoopDetector() : LoopDetector(Options{}) {}

LoopDetector::LoopDetector(const Options& options) : options_(options) {
    // No ICP member initialization needed - small_gicp creates registration
    // objects on-the-fly in detect() for better flexibility
}

PointCloudXYZI::Ptr LoopDetector::buildSubmap(
    const KeyFrameVector& keyframes,
    int center_idx, int half_range, double resolution) {
    const int n = static_cast<int>(keyframes.size());
    int min_idx = std::max(0, center_idx - half_range);
    int max_idx = std::min(n - 1, center_idx + half_range);

    PointCloudXYZI::Ptr ret = pcl::make_shared<PointCloudXYZI>();
    for (int i = min_idx; i <= max_idx; ++i) {
        const KeyFrame& kf = keyframes[i];
        if (!kf.body_cloud || kf.body_cloud->points.empty()) {
            continue;
        }
        
        // Get global pose from SE3 (using compatibility methods)
        Eigen::Matrix3d r_global = kf.r_global();
        Eigen::Vector3d t_global = kf.t_global();
        
        // Transform points directly into ret to avoid intermediate allocation
        for (const auto& pt : kf.body_cloud->points) {
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            p = r_global * p + t_global;
            PointType p_world;
            p_world.x = static_cast<float>(p.x());
            p_world.y = static_cast<float>(p.y());
            p_world.z = static_cast<float>(p.z());
            p_world.intensity = pt.intensity;
            ret->points.push_back(p_world);
        }
    }
    
    // Use small_gicp's voxelgrid_sampling instead of PCL VoxelGrid
    // This is more stable and avoids PCL VoxelGrid memory issues
    if (resolution > 0 && !ret->empty()) {
        // Convert to pcl::PointXYZ for small_gicp downsampling
        auto xyz_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        xyz_cloud->reserve(ret->size());
        for (const auto& pt : ret->points) {
            pcl::PointXYZ p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            xyz_cloud->push_back(p);
        }
        
        // Use small_gicp downsampling (thread-safe, single-threaded)
        auto downsampled_xyz = small_gicp::voxelgrid_sampling(*xyz_cloud, resolution);
        
        // Convert back to PointCloudXYZI
        PointCloudXYZI::Ptr filtered = pcl::make_shared<PointCloudXYZI>();
        filtered->reserve(downsampled_xyz->size());
        for (const auto& pt : downsampled_xyz->points) {
            PointType p_out;
            p_out.x = pt.x;
            p_out.y = pt.y;
            p_out.z = pt.z;
            p_out.intensity = 0;  // Intensity info is lost in downsampling
            filtered->points.push_back(p_out);
        }
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;
        return filtered;
    }
    
    ret->width = ret->points.size();
    ret->height = 1;
    ret->is_dense = true;
    return ret;
}

std::optional<LoopConstraint> LoopDetector::detect(
    const KeyFrameVector& keyframes,
    size_t current_idx) {
    if (keyframes.size() < 10 || current_idx >= keyframes.size()) {
        return std::nullopt;
    }

    if (options_.min_detect_interval > 0.0 && !history_pairs_.empty()) {
        double current_time = keyframes[current_idx].timestamp;
        size_t last_source = history_pairs_.back().second;
        double last_time = keyframes[last_source].timestamp;
        if (current_time - last_time < options_.min_detect_interval) {
            return std::nullopt;
        }
    }

    const KeyFrame& current_kf = keyframes[current_idx];
    Eigen::Vector3d current_t = current_kf.t_global();
    
    pcl::PointXYZ current_pt;
    current_pt.x = current_t.x();
    current_pt.y = current_t.y();
    current_pt.z = current_t.z();

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (size_t i = 0; i < current_idx; ++i) {
        Eigen::Vector3d t = keyframes[i].t_global();
        pcl::PointXYZ pt;
        pt.x = t.x();
        pt.y = t.y();
        pt.z = t.z();
        key_poses_cloud->push_back(pt);
    }
    if (key_poses_cloud->empty()) {
        return std::nullopt;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int num_neighbors = kdtree.radiusSearch(
        current_pt, options_.search_radius, ids, sqdists);
    if (num_neighbors == 0) {
        if (options_.verbose && current_idx % 100 == 0) {
            std::cout << "[LoopDetector] kf " << current_idx
                      << ": no keyframe within search_radius " << options_.search_radius << " m"
                      << std::endl;
        }
        return std::nullopt;
    }

    int loop_idx = -1;
    double max_dt = 0.0;
    for (size_t i = 0; i < ids.size(); ++i) {
        int idx = ids[i];
        double dt = std::abs(current_kf.timestamp - keyframes[idx].timestamp);
        if (dt > max_dt) max_dt = dt;
        if (dt > options_.time_threshold) {
            loop_idx = idx;
            break;
        }
    }
    if (loop_idx < 0) {
        if (options_.verbose && current_idx % 20 == 0) {
            std::cout << "[LoopDetector] kf " << current_idx
                      << ": " << num_neighbors << " neighbor(s) within " << options_.search_radius
                      << " m but none with dt>" << options_.time_threshold
                      << " s (max_dt=" << max_dt << " s)" << std::endl;
        }
        return std::nullopt;
    }

    PointCloudXYZI::Ptr target_cloud = buildSubmap(
        keyframes, loop_idx, options_.submap_half_range, options_.submap_resolution);
    PointCloudXYZI::Ptr source_cloud = buildSubmap(
        keyframes, static_cast<int>(current_idx), 0, options_.submap_resolution);

    if (!target_cloud || !source_cloud || target_cloud->empty() || source_cloud->empty()) {
        return std::nullopt;
    }

    // Convert PointXYZINormal to PointXYZ for small_gicp (simpler and more robust)
    auto convert_to_xyz = [](const PointCloudXYZI::Ptr& input) {
        auto output = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(input->size());
        for (const auto& pt : input->points) {
            pcl::PointXYZ p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            output->push_back(p);
        }
        return output;
    };

    auto target_xyz = convert_to_xyz(target_cloud);
    auto source_xyz = convert_to_xyz(source_cloud);

    // Variables to store results (extracted before gicp goes out of scope)
    bool converged = false;
    double fitness_score = std::numeric_limits<double>::max();
    Eigen::Matrix4f T_gicp = Eigen::Matrix4f::Identity();

    // Use a scope block to ensure gicp is destroyed before we use the results
    // This ensures proper destruction order: gicp -> internal KdTrees -> point cloud refs
    {
        // Use small_gicp's RegistrationPCL (drop-in replacement for pcl::GICP)
        // Note: Use single thread to avoid potential OpenMP conflicts
        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setNumThreads(1);  // Single thread to avoid memory issues
        gicp.setCorrespondenceRandomness(options_.gicp_correspondence_randomness);
        gicp.setMaxCorrespondenceDistance(options_.gicp_max_correspondence_dist);
        gicp.setMaximumIterations(options_.gicp_max_iterations);
        gicp.setRegistrationType("GICP");  // Use GICP for point-to-plane ICP

        gicp.setInputTarget(target_xyz);
        gicp.setInputSource(source_xyz);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        gicp.align(*aligned);

        converged = gicp.hasConverged();
        if (converged) {
            // Get fitness score from registration result
            fitness_score = gicp.getRegistrationResult().error;
            // Normalize fitness score by number of inliers if available
            if (gicp.getRegistrationResult().num_inliers > 0) {
                fitness_score /= static_cast<double>(gicp.getRegistrationResult().num_inliers);
            }
            T_gicp = gicp.getFinalTransformation();
        }
    }  // gicp is destroyed here, releasing internal KdTrees

    if (!converged) {
        if (options_.verbose) {
            std::cout << "[LoopDetector] kf " << current_idx
                      << ": GICP not converged (candidate kf " << loop_idx << ")" << std::endl;
        }
        return std::nullopt;
    }

    if (fitness_score > options_.fitness_threshold) {
        if (options_.verbose) {
            std::cout << "[LoopDetector] kf " << current_idx
                      << ": fitness " << fitness_score << " > threshold " << options_.fitness_threshold
                      << " (candidate kf " << loop_idx << ")" << std::endl;
        }
        return std::nullopt;
    }
    
    // Get current keyframe's global pose
    Eigen::Matrix3d current_r = current_kf.r_global();
    Eigen::Vector3d current_t_vec = current_kf.t_global();
    
    // Apply ICP transformation to refine the pose
    Eigen::Matrix3d r_refined = T_gicp.block<3, 3>(0, 0).cast<double>() * current_r;
    Eigen::Vector3d t_refined = T_gicp.block<3, 3>(0, 0).cast<double>() * current_t_vec
                               + T_gicp.block<3, 1>(0, 3).cast<double>();

    // Get target keyframe's global pose
    const KeyFrame& target_kf = keyframes[loop_idx];
    Eigen::Matrix3d target_r = target_kf.r_global();
    Eigen::Vector3d target_t = target_kf.t_global();
    
    // Compute relative pose from target to source (T_target_source)
    Eigen::Matrix3d r_rel = target_r.transpose() * r_refined;
    Eigen::Vector3d t_rel = target_r.transpose() * (t_refined - target_t);
    // GICP can return slightly non-orthogonal rotation; orthogonalize for Sophus::SE3
    r_rel = orthogonalizeRotation(r_rel);

    // Create loop constraint with SE3
    LoopConstraint loop;
    loop.source_id = current_idx;
    loop.target_id = static_cast<size_t>(loop_idx);
    loop.T_target_source = SE3(r_rel, t_rel);
    loop.fitness_score = fitness_score;

    history_pairs_.emplace_back(loop.target_id, loop.source_id);
    last_detect_time_ = current_kf.timestamp;
    
    if (options_.verbose) {
        std::cout << "[LoopDetector] Loop detected: " << loop.target_id 
                  << " -> " << loop.source_id 
                  << " (fitness=" << fitness_score << ")" << std::endl;
    }

    return loop;
}

}  // namespace livo2_offline
