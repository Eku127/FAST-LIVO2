/*
 * Eigen and Sophus type definitions for FAST-LIVO2 backend
 * Provides SE3/SO3 and common matrix types for pose graph optimization
 */

#ifndef COMMON_EIGEN_TYPES_H
#define COMMON_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <memory>
#include <mutex>

// MIAO backend expects types in namespace lightning (Eigen + Sophus aliases)
namespace lightning {
using MatrixX = Eigen::MatrixXd;
using VectorX = Eigen::VectorXd;
using VecXd = Eigen::VectorXd;
using MatXd = Eigen::MatrixXd;
using Vec3d = Eigen::Vector3d;
using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Quatd = Eigen::Quaterniond;
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using UL = std::unique_lock<std::mutex>;
}  // namespace lightning

namespace livo2_offline {

// Sophus pose types
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

// Eigen vector types
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec6d = Eigen::Matrix<double, 6, 1>;

// Eigen matrix types
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// Quaternion type
using Quatd = Eigen::Quaterniond;

// Thread safety
using UL = std::unique_lock<std::mutex>;

}  // namespace livo2_offline

#endif  // COMMON_EIGEN_TYPES_H
