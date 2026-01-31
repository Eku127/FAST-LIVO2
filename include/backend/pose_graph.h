/*
 * Pose Graph Optimization backend using MIAO optimizer
 * Replaces GTSAM ISAM2 with MIAO for more flexibility and lighter dependencies
 * Design inspired by lightning-lm's PGO implementation
 */

#ifndef BACKEND_POSE_GRAPH_H
#define BACKEND_POSE_GRAPH_H

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "offline/keyframe_types.h"

// Get SE3 from keyframe_types.h
#include "offline/keyframe_types.h"

// MIAO headers (from thirdparty/miao - note: includes must use full path from thirdparty/miao)
#include "common/eigen_types.h"  // SE3, SO3 types from miao
#include "common/std_types.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_prior.h"
#include "core/types/vertex_se3.h"
#include "core/robust_kernel/cauchy.h"
#include "core/robust_kernel/huber.h"

namespace livo2_offline {

// Import Quatd from lightning namespace
using lightning::Quatd;

// MIAO namespace alias
namespace miao = lightning::miao;

/**
 * @brief Pose graph optimization using MIAO optimizer
 * 
 * Features:
 * - Incremental optimization mode for real-time applications
 * - Built-in robust kernels (Cauchy, Huber) for outlier rejection
 * - SE3 parameterization for poses
 */
class PoseGraph {
public:
    /**
     * @brief Configuration options for PoseGraph
     * Follows lightning-lm's Options pattern for clean configuration
     */
    struct Options {
        // Optimizer configuration
        bool verbose{false};
        bool incremental_mode{true};
        int max_iterations{20};
        
        // Noise parameters (translation first, then rotation - MIAO/g2o convention)
        // Units: meters for translation, radians for rotation
        double prior_trans_noise{0.1};
        double prior_rot_noise{1.0 * M_PI / 180.0};  // 1 degree
        double odom_trans_noise{0.1};
        double odom_rot_noise{1.0 * M_PI / 180.0};   // 1 degree
        double loop_trans_noise{0.2};
        double loop_rot_noise{3.0 * M_PI / 180.0};   // 3 degrees
        
        // Robust Kernel configuration
        bool use_robust_kernel{true};
        double robust_kernel_delta{5.2};  // Chi2 threshold for Cauchy kernel
    };

    PoseGraph();
    explicit PoseGraph(const Options& options);
    ~PoseGraph() = default;

    /**
     * @brief Add keyframe with odometry constraint
     * @param kf New keyframe
     * @param prev_kf Previous keyframe (nullptr for first)
     * @return true if keyframe was added
     */
    bool addKeyframe(const KeyFrame& kf, const KeyFrame* prev_kf = nullptr);

    /**
     * @brief Add loop closure constraint (relative pose from target to source)
     */
    void addLoopConstraint(const LoopConstraint& loop);

    /**
     * @brief Run optimization and update internal state
     */
    void optimize();

    /**
     * @brief Get optimized poses (one per keyframe)
     */
    std::vector<SE3> getOptimizedPoses() const;
    
    /**
     * @brief Get optimized poses as Isometry3d (for compatibility)
     */
    std::vector<Eigen::Isometry3d> getOptimizedPosesIsometry() const;

    /**
     * @brief Get correction offset for converting local to global pose
     * global = r_offset * local_rot, global_t = r_offset * local_t + t_offset
     */
    void getOffset(Mat3d& r_offset, Vec3d& t_offset) const;
    Mat3d offsetR() const { return r_offset_; }
    Vec3d offsetT() const { return t_offset_; }

    /**
     * @brief Save factor graph to g2o format
     */
    void saveG2o(const std::string& filename) const;

    /**
     * @brief Number of keyframes in the graph
     */
    size_t size() const { return keyframe_count_; }
    
    /**
     * @brief Get current options
     */
    const Options& options() const { return options_; }

private:
    // Build information matrix from noise parameters
    Mat6d buildInfoMatrix(double trans_noise, double rot_noise) const;
    
    // Update offset after optimization
    void updateOffset();
    
private:
    Options options_;
    
    // MIAO optimizer
    std::shared_ptr<miao::Optimizer> optimizer_;
    
    // Vertex and edge caches
    std::vector<std::shared_ptr<miao::VertexSE3>> vertices_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> odom_edges_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> loop_edges_;
    std::shared_ptr<miao::EdgeSE3Prior> prior_edge_;
    
    // Keyframe data
    std::vector<SE3> keyframe_poses_;  // Initial poses
    size_t keyframe_count_ = 0;
    
    // Pending loop constraints to be added in next optimize()
    std::vector<LoopConstraint> pending_loops_;
    
    // Offset for converting local poses to global (updated after optimization)
    // global = r_offset * local_rot, global_t = r_offset * local_t + t_offset
    Mat3d r_offset_ = Mat3d::Identity();
    Vec3d t_offset_ = Vec3d::Zero();
    SE3 last_local_pose_;
    
    // Pre-computed information matrices
    Mat6d info_prior_;
    Mat6d info_odom_;
    Mat6d info_loop_;
};

}  // namespace livo2_offline

#endif  // BACKEND_POSE_GRAPH_H
