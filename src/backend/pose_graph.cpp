/*
 * Pose Graph Optimization implementation using GTSAM ISAM2
 */

#include "backend/pose_graph.h"

#include <gtsam/slam/dataset.h>

#include <iostream>

namespace livo2_offline {

PoseGraph::PoseGraph(const Config& config) : config_(config) {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = config_.relinearize_threshold;
    params.relinearizeSkip = config_.relinearize_skip;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
    graph_.resize(0);
    initial_values_.clear();
    r_offset_.setIdentity();
    t_offset_.setZero();
    last_r_local_.setIdentity();
    last_t_local_.setZero();
}

bool PoseGraph::addKeyframe(const KeyFrame& kf, const KeyFrame* prev_kf) {
    const size_t idx = keyframe_count_;
    Eigen::Matrix3d init_r;
    Eigen::Vector3d init_t;
    if (idx == 0) {
        init_r = kf.r_local;
        init_t = kf.t_local;
    } else {
        init_r = r_offset_ * kf.r_local;
        init_t = r_offset_ * kf.t_local + t_offset_;
    }

    gtsam::Pose3 init_pose(gtsam::Rot3(init_r), gtsam::Point3(init_t.x(), init_t.y(), init_t.z()));
    initial_values_.insert(idx, init_pose);

    if (idx == 0) {
        auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
            gtsam::Vector6::Ones() * config_.prior_noise);
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(idx, init_pose, prior_noise));
    } else {
        Eigen::Matrix3d r_between = prev_kf->r_local.transpose() * kf.r_local;
        Eigen::Vector3d t_between = prev_kf->r_local.transpose() * (kf.t_local - prev_kf->t_local);
        gtsam::Pose3 between_pose(gtsam::Rot3(r_between), gtsam::Point3(t_between.x(), t_between.y(), t_between.z()));
        auto odom_noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << config_.odom_rot_noise, config_.odom_rot_noise, config_.odom_rot_noise,
             config_.odom_trans_noise, config_.odom_trans_noise, config_.odom_rot_noise).finished());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, between_pose, odom_noise));
    }

    last_r_local_ = kf.r_local;
    last_t_local_ = kf.t_local;
    keyframe_count_++;
    return true;
}

void PoseGraph::addLoopConstraint(const LoopConstraint& loop) {
    pending_loops_.push_back(loop);
}

void PoseGraph::optimize() {
    const size_t num_loops = pending_loops_.size();
    for (const auto& loop : pending_loops_) {
        gtsam::Pose3 loop_pose(gtsam::Rot3(loop.r_offset), gtsam::Point3(loop.t_offset.x(), loop.t_offset.y(), loop.t_offset.z()));
        double score = std::max(loop.fitness_score, 1e-6);
        auto loop_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * score);
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            static_cast<gtsam::Key>(loop.target_id),
            static_cast<gtsam::Key>(loop.source_id),
            loop_pose, loop_noise));
    }
    pending_loops_.clear();

    // Only call ISAM2::update when we have at least 2 keyframes (prior + between factor).
    // With a single keyframe (prior only), ISAM2::update() can trigger double-free/corruption.
    if (graph_.size() > 0 && keyframe_count_ > 1) {
        isam2_->update(graph_, initial_values_);
        // Do NOT clear graph_/initial_values_ here: ISAM2 may still reference them.
        if (num_loops > 0) {
            for (int i = 0; i < 4; ++i) {
                isam2_->update();
            }
        }
    }

    if (keyframe_count_ > 0) {
        if (keyframe_count_ == 1) {
            // Single keyframe: no optimization; keep identity offset (local = global).
            r_offset_.setIdentity();
            t_offset_.setZero();
        } else {
            gtsam::Values estimate = isam2_->calculateBestEstimate();
            gtsam::Pose3 last_pose = estimate.at<gtsam::Pose3>(keyframe_count_ - 1);
            Eigen::Matrix3d R_opt = last_pose.rotation().matrix();
            Eigen::Vector3d t_opt(last_pose.translation().x(), last_pose.translation().y(), last_pose.translation().z());
            r_offset_ = R_opt * last_r_local_.transpose();
            t_offset_ = t_opt - r_offset_ * last_t_local_;
        }
    }

    // Clear graph/values only after we are done with ISAM2 for this round (avoids double-free).
    if (graph_.size() > 0) {
        graph_.resize(0);
        initial_values_.clear();
    }
}

std::vector<Eigen::Isometry3d> PoseGraph::getOptimizedPoses() const {
    std::vector<Eigen::Isometry3d> poses;
    if (keyframe_count_ == 0) return poses;
    poses.reserve(keyframe_count_);
    if (keyframe_count_ == 1) {
        // Single keyframe: no ISAM2 estimate; use local pose (global = local).
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = last_r_local_;
        T.translation() = last_t_local_;
        poses.push_back(T);
        return poses;
    }
    gtsam::Values estimate = isam2_->calculateBestEstimate();
    for (size_t i = 0; i < keyframe_count_; ++i) {
        gtsam::Pose3 p = estimate.at<gtsam::Pose3>(i);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = p.rotation().matrix();
        T.translation() = Eigen::Vector3d(p.translation().x(), p.translation().y(), p.translation().z());
        poses.push_back(T);
    }
    return poses;
}

void PoseGraph::getOffset(Eigen::Matrix3d& r_offset, Eigen::Vector3d& t_offset) const {
    r_offset = r_offset_;
    t_offset = t_offset_;
}

void PoseGraph::saveG2o(const std::string& filename) const {
    if (keyframe_count_ == 0) {
        std::cerr << "[PoseGraph] Cannot save g2o: no keyframes" << std::endl;
        return;
    }
    gtsam::NonlinearFactorGraph full_graph;
    for (size_t i = 0; i < isam2_->getFactorsUnsafe().size(); ++i) {
        full_graph.add(isam2_->getFactorsUnsafe().at(i));
    }
    gtsam::Values all_values = isam2_->calculateBestEstimate();
    gtsam::writeG2o(full_graph, all_values, filename);
    std::cout << "[PoseGraph] Saved " << keyframe_count_ << " poses to g2o: " << filename << std::endl;
}

void PoseGraph::loadG2o(const std::string& filename) {
    auto result = gtsam::readG2o(filename, true);
    gtsam::NonlinearFactorGraph::shared_ptr graph = result.first;
    gtsam::Values::shared_ptr values = result.second;
    if (!graph || !values || graph->size() == 0) {
        std::cerr << "[PoseGraph] Failed to load g2o or empty graph: " << filename << std::endl;
        return;
    }
    isam2_->update(*graph, *values);
    keyframe_count_ = values->size();
    // Offset is not updated when loading from file (no local poses available)
    std::cout << "[PoseGraph] Loaded g2o: " << filename << " (" << keyframe_count_ << " poses)" << std::endl;
}

}  // namespace livo2_offline
