/*
 * Pose Graph Optimization implementation using MIAO optimizer
 * 
 * Replaces GTSAM ISAM2 with MIAO for more flexibility and lighter dependencies.
 * Reference: lightning-lm's PGO and loop_closing implementation
 * 
 * Key differences from GTSAM:
 * - Information matrix format: translation first (0-2), rotation second (3-5)
 * - Built-in robust kernels (Cauchy, Huber) for outlier rejection
 * - Incremental mode for real-time applications
 */

#include "backend/pose_graph.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace livo2_offline {

PoseGraph::PoseGraph() : PoseGraph(Options{}) {}

PoseGraph::PoseGraph(const Options& options) : options_(options) {
    // Initialize MIAO optimizer
    miao::OptimizerConfig config(
        miao::AlgorithmType::LEVENBERG_MARQUARDT,
        miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
        false);  // Non-dense problem
    config.incremental_mode_ = options_.incremental_mode;
    
    optimizer_ = miao::SetupOptimizer<6, 3>(config);
    optimizer_->SetVerbose(options_.verbose);
    
    // Pre-compute information matrices
    info_prior_ = buildInfoMatrix(options_.prior_trans_noise, options_.prior_rot_noise);
    info_odom_ = buildInfoMatrix(options_.odom_trans_noise, options_.odom_rot_noise);
    info_loop_ = buildInfoMatrix(options_.loop_trans_noise, options_.loop_rot_noise);
    
    // Initialize offset
    r_offset_.setIdentity();
    t_offset_.setZero();
}

Mat6d PoseGraph::buildInfoMatrix(double trans_noise, double rot_noise) const {
    // MIAO/g2o format: translation first (0-2), rotation second (3-5)
    Mat6d info = Mat6d::Identity();
    info.block<3,3>(0,0) = Mat3d::Identity() / (trans_noise * trans_noise);
    info.block<3,3>(3,3) = Mat3d::Identity() / (rot_noise * rot_noise);
    return info;
}

bool PoseGraph::addKeyframe(const KeyFrame& kf, const KeyFrame* prev_kf) {
    (void)prev_kf;  // Unused - we track poses internally
    const size_t idx = keyframe_count_;
    
    // Get local pose from keyframe (now using SE3)
    SE3 local_pose = kf.pose_local;
    
    // Compute initial pose in global frame
    SE3 global_pose;
    if (idx == 0) {
        // First keyframe: global = local
        global_pose = local_pose;
    } else {
        // Apply current offset to get global pose
        SE3 offset(r_offset_, t_offset_);
        global_pose = offset * local_pose;
    }
    
    // Create vertex
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(static_cast<int>(idx));
    v->SetEstimate(global_pose);
    optimizer_->AddVertex(v);
    vertices_.push_back(v);
    
    // Store for later use
    keyframe_poses_.push_back(global_pose);
    last_local_pose_ = local_pose;
    keyframe_count_++;
    
    // Add prior edge for first keyframe
    if (idx == 0) {
        prior_edge_ = std::make_shared<miao::EdgeSE3Prior>();
        prior_edge_->SetVertex(0, v);
        prior_edge_->SetMeasurement(global_pose);
        prior_edge_->SetInformation(info_prior_);
        optimizer_->AddEdge(prior_edge_);
    }
    
    // Add odometry edge
    if (idx > 0) {
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, vertices_[idx - 1]);
        e->SetVertex(1, v);
        
        // Compute relative pose: T_prev_curr = T_prev^-1 * T_curr
        SE3 relative = keyframe_poses_[idx - 1].inverse() * global_pose;
        e->SetMeasurement(relative);
        e->SetInformation(info_odom_);
        optimizer_->AddEdge(e);
        odom_edges_.push_back(e);
    }
    
    return true;
}

void PoseGraph::addLoopConstraint(const LoopConstraint& loop) {
    pending_loops_.push_back(loop);
}

void PoseGraph::optimize() {
    if (keyframe_count_ < 2) return;
    
    const size_t num_loops = pending_loops_.size();
    
    // Add pending loop constraints
    for (const auto& loop : pending_loops_) {
        auto e = std::make_shared<miao::EdgeSE3>();
        
        // Get vertices by ID
        auto v0 = optimizer_->GetVertex(static_cast<int>(loop.target_id));
        auto v1 = optimizer_->GetVertex(static_cast<int>(loop.source_id));
        
        if (!v0 || !v1) {
            std::cerr << "[PoseGraph] Warning: Invalid vertex ID in loop constraint: "
                      << loop.target_id << " -> " << loop.source_id << std::endl;
            continue;
        }
        
        e->SetVertex(0, v0);
        e->SetVertex(1, v1);
        e->SetMeasurement(loop.T_target_source);
        e->SetInformation(info_loop_);
        
        // Add Robust Kernel for outlier rejection
        if (options_.use_robust_kernel) {
            auto rk = std::make_shared<miao::RobustKernelCauchy>();
            rk->SetDelta(options_.robust_kernel_delta);
            e->SetRobustKernel(rk);
        }
        
        optimizer_->AddEdge(e);
        loop_edges_.push_back(e);
    }
    
    bool has_loops = !pending_loops_.empty();
    pending_loops_.clear();
    
    // Log loop closure info
    if (has_loops && options_.verbose) {
        std::cout << "[PoseGraph] *** Loop closure PGO: adding " << num_loops
                  << " loop constraint(s)" << std::endl;
    }
    
    // Run optimization
    double chi2_before = optimizer_->ActiveChi2();
    optimizer_->InitializeOptimization();
    int iterations = optimizer_->Optimize(options_.max_iterations);
    double chi2_after = optimizer_->ActiveChi2();
    
    if (options_.verbose) {
        std::cout << std::fixed << std::setprecision(6)
                  << "[PoseGraph] MIAO optimize: keyframes=" << keyframe_count_
                  << " iterations=" << iterations
                  << " chi2_before=" << chi2_before
                  << " chi2_after=" << chi2_after
                  << " loop_edges=" << loop_edges_.size() << std::endl;
    }
    
    if (has_loops && options_.verbose) {
        std::cout << "[PoseGraph] *** Loop closure PGO done." << std::endl;
    }
    
    // Update offset
    updateOffset();
}

void PoseGraph::updateOffset() {
    if (keyframe_count_ > 0 && !vertices_.empty()) {
        // Get optimized pose for last keyframe
        SE3 opt_pose = vertices_.back()->Estimate();
        
        // Compute offset: offset = opt_pose * local_pose^-1
        SE3 offset = opt_pose * last_local_pose_.inverse();
        r_offset_ = offset.rotationMatrix();
        t_offset_ = offset.translation();
    }
}

std::vector<SE3> PoseGraph::getOptimizedPoses() const {
    std::vector<SE3> poses;
    if (keyframe_count_ == 0) return poses;
    poses.reserve(keyframe_count_);
    
    for (const auto& v : vertices_) {
        poses.push_back(v->Estimate());
    }
    return poses;
}

std::vector<Eigen::Isometry3d> PoseGraph::getOptimizedPosesIsometry() const {
    std::vector<Eigen::Isometry3d> poses;
    if (keyframe_count_ == 0) return poses;
    poses.reserve(keyframe_count_);
    
    for (const auto& v : vertices_) {
        SE3 se3_pose = v->Estimate();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = se3_pose.rotationMatrix();
        T.translation() = se3_pose.translation();
        poses.push_back(T);
    }
    return poses;
}

void PoseGraph::getOffset(Mat3d& r_offset, Vec3d& t_offset) const {
    r_offset = r_offset_;
    t_offset = t_offset_;
}

void PoseGraph::saveG2o(const std::string& filename) const {
    if (keyframe_count_ == 0) {
        std::cerr << "[PoseGraph] Cannot save g2o: no keyframes" << std::endl;
        return;
    }
    
    std::ofstream fout(filename);
    if (!fout) {
        std::cerr << "[PoseGraph] Cannot open file for writing: " << filename << std::endl;
        return;
    }
    
    // Write vertices
    for (const auto& v : vertices_) {
        SE3 pose = v->Estimate();
        Vec3d t = pose.translation();
        Quatd q = pose.unit_quaternion();
        
        // g2o format: VERTEX_SE3:QUAT id x y z qx qy qz qw
        fout << "VERTEX_SE3:QUAT " << v->GetId() << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    
    // Write odometry edges
    for (const auto& e : odom_edges_) {
        SE3 meas = e->GetMeasurement();
        Vec3d t = meas.translation();
        Quatd q = meas.unit_quaternion();
        Mat6d info = e->Information();
        
        // g2o format: EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw info(upper triangle)
        fout << "EDGE_SE3:QUAT " << e->GetVertex(0)->GetId() << " " << e->GetVertex(1)->GetId() << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        
        // Write upper triangle of information matrix
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                fout << info(i, j) << " ";
            }
        }
        fout << std::endl;
    }
    
    // Write loop edges
    for (const auto& e : loop_edges_) {
        SE3 meas = e->GetMeasurement();
        Vec3d t = meas.translation();
        Quatd q = meas.unit_quaternion();
        Mat6d info = e->Information();
        
        fout << "EDGE_SE3:QUAT " << e->GetVertex(0)->GetId() << " " << e->GetVertex(1)->GetId() << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                fout << info(i, j) << " ";
            }
        }
        fout << std::endl;
    }
    
    fout.close();
    std::cout << "[PoseGraph] Saved " << keyframe_count_ << " poses to g2o: " << filename << std::endl;
}

}  // namespace livo2_offline
