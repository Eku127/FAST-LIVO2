/*
 * Keyframe and loop constraint types for offline PGO.
 * Separate header to avoid circular includes with backend.
 */

#ifndef OFFLINE_KEYFRAME_TYPES_H
#define OFFLINE_KEYFRAME_TYPES_H

#include <vector>
#include <Eigen/Core>
#include "utils/types.h"

namespace livo2_offline {

/**
 * @brief Loop closure constraint (relative pose from target to source)
 */
struct LoopConstraint {
    size_t source_id;
    size_t target_id;
    Eigen::Matrix3d r_offset;
    Eigen::Vector3d t_offset;
    double fitness_score;
};

/**
 * @brief Keyframe structure for PGO preparation
 * Uses EIGEN_MAKE_ALIGNED_OPERATOR_NEW so that KeyFrame in std::vector
 * (with Eigen::aligned_allocator) avoids "double free or corruption" from misalignment.
 */
struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t id;
    double timestamp;
    Eigen::Matrix3d r_local;
    Eigen::Vector3d t_local;
    Eigen::Matrix3d r_global;
    Eigen::Vector3d t_global;
    PointCloudXYZI::Ptr body_cloud;
    Eigen::Matrix<double, 6, 6> pose_covariance;

    KeyFrame()
        : id(0),
          timestamp(0.0),
          r_local(Eigen::Matrix3d::Identity()),
          t_local(Eigen::Vector3d::Zero()),
          r_global(Eigen::Matrix3d::Identity()),
          t_global(Eigen::Vector3d::Zero()),
          body_cloud(new PointCloudXYZI()),
          pose_covariance(Eigen::Matrix<double, 6, 6>::Identity() * 0.01) {}
};

/// Vector of keyframes with aligned allocator for Eigen members
using KeyFrameVector = std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>>;

}  // namespace livo2_offline

#endif  // OFFLINE_KEYFRAME_TYPES_H
