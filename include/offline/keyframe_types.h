/*
 * Keyframe and loop constraint types for offline PGO.
 * Separate header to avoid circular includes with backend.
 */

#ifndef OFFLINE_KEYFRAME_TYPES_H
#define OFFLINE_KEYFRAME_TYPES_H

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "utils/types.h"

namespace livo2_offline {

// Type aliases for Sophus SE3 (local definition, compatible with MIAO's lightning::SE3)
using SE3 = Sophus::SE3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat3d = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;

/**
 * @brief Loop closure constraint (relative pose from target to source)
 */
struct LoopConstraint {
    size_t source_id = 0;
    size_t target_id = 0;
    SE3 T_target_source;     // Relative pose T_ij (from target frame to source frame)
    double fitness_score = 0.0;
    
    // Backward compatibility getters
    Eigen::Matrix3d r_offset() const { return T_target_source.rotationMatrix(); }
    Eigen::Vector3d t_offset() const { return T_target_source.translation(); }
};

/**
 * @brief Keyframe structure for PGO preparation
 * Uses EIGEN_MAKE_ALIGNED_OPERATOR_NEW so that KeyFrame in std::vector
 * (with Eigen::aligned_allocator) avoids "double free or corruption" from misalignment.
 */
struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<KeyFrame>;
    
    size_t id = 0;
    double timestamp = 0.0;
    
    // SE3 poses (primary storage)
    SE3 pose_local;   // Local frame pose (from frontend LIO)
    SE3 pose_global;  // Global frame pose (updated after PGO)
    
    PointCloudXYZI::Ptr body_cloud;
    Mat6d pose_covariance = Mat6d::Identity() * 0.01;
    
    KeyFrame()
        : body_cloud(new PointCloudXYZI()) {}
    
    // Backward compatibility getters/setters for r/t format
    Eigen::Matrix3d r_local() const { return pose_local.rotationMatrix(); }
    Eigen::Vector3d t_local() const { return pose_local.translation(); }
    Eigen::Matrix3d r_global() const { return pose_global.rotationMatrix(); }
    Eigen::Vector3d t_global() const { return pose_global.translation(); }
    
    void set_local(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) {
        pose_local = SE3(r, t);
    }
    void set_global(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) {
        pose_global = SE3(r, t);
    }
};

/// Vector of keyframes with aligned allocator for Eigen members
using KeyFrameVector = std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>>;

}  // namespace livo2_offline

#endif  // OFFLINE_KEYFRAME_TYPES_H
