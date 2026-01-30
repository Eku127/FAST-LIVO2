/*
 * Pose Graph Optimization backend using GTSAM ISAM2
 */

#ifndef BACKEND_POSE_GRAPH_H
#define BACKEND_POSE_GRAPH_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "offline/keyframe_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace livo2_offline {

/**
 * @brief Pose graph optimization using GTSAM ISAM2
 */
class PoseGraph {
public:
    struct Config {
        double relinearize_threshold = 0.01;
        int relinearize_skip = 1;
        bool use_isam2 = true;
        double prior_noise = 1e-12;
        double odom_rot_noise = 1e-6;
        double odom_trans_noise = 1e-4;
    };

    explicit PoseGraph(const Config& config);

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
    std::vector<Eigen::Isometry3d> getOptimizedPoses() const;

    /**
     * @brief Get correction offset for converting local to global pose
     * global = r_offset * local_rot, global_t = r_offset * local_t + t_offset
     */
    void getOffset(Eigen::Matrix3d& r_offset, Eigen::Vector3d& t_offset) const;
    Eigen::Matrix3d offsetR() const { return r_offset_; }
    Eigen::Vector3d offsetT() const { return t_offset_; }

    /**
     * @brief Save factor graph and values to g2o format
     */
    void saveG2o(const std::string& filename) const;

    /**
     * @brief Load factor graph from g2o and merge into ISAM2
     */
    void loadG2o(const std::string& filename);

    /**
     * @brief Number of keyframes in the graph
     */
    size_t size() const { return keyframe_count_; }

private:
    Config config_;
    std::shared_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    size_t keyframe_count_ = 0;
    std::vector<LoopConstraint> pending_loops_;
    Eigen::Matrix3d r_offset_;
    Eigen::Vector3d t_offset_;
    Eigen::Matrix3d last_r_local_;
    Eigen::Vector3d last_t_local_;
};

}  // namespace livo2_offline

#endif  // BACKEND_POSE_GRAPH_H
