/*
 * GICP-based loop closure detection using small_gicp
 * Replaces PCL ICP with small_gicp for better stability and performance.
 * Design follows the Options pattern for consistent API with PoseGraph.
 */

#ifndef BACKEND_LOOP_DETECTOR_H
#define BACKEND_LOOP_DETECTOR_H

#include <optional>
#include <vector>

#include "offline/keyframe_types.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// small_gicp headers for GICP registration
#include <small_gicp/pcl/pcl_registration.hpp>

namespace livo2_offline {

/**
 * @brief GICP-based loop closure detection using submap matching
 * Uses small_gicp instead of PCL ICP for improved stability and performance.
 */
class LoopDetector {
public:
    /**
     * @brief Configuration options for LoopDetector
     * Follows the Options pattern for consistent API with PoseGraph
     */
    struct Options {
        // General settings
        bool verbose{false};
        
        // Candidate search parameters
        double search_radius{15.0};         // meters, radius to search for candidates
        double time_threshold{60.0};        // seconds, minimum time gap for loop candidates
        double min_detect_interval{10.0};   // seconds, minimum interval between detections
        
        // Submap building parameters
        int submap_half_range{3};           // keyframes on each side of candidate
        double submap_resolution{0.12};     // meters, voxel size for downsampling
        
        // GICP parameters
        double fitness_threshold{0.15};     // maximum fitness score to accept loop
        int gicp_max_iterations{50};
        double gicp_max_correspondence_dist{10.0};
        int gicp_num_threads{4};
        int gicp_correspondence_randomness{20};
    };

    LoopDetector();
    explicit LoopDetector(const Options& options);

    /**
     * @brief Search for loop closure for the current keyframe
     * @param keyframes All keyframes (with global poses and body clouds)
     * @param current_idx Index of the current (new) keyframe
     * @return Loop constraint if a loop was detected, nullopt otherwise
     */
    std::optional<LoopConstraint> detect(
        const KeyFrameVector& keyframes,
        size_t current_idx);

    /**
     * @brief History of detected loop pairs (target_id, source_id)
     */
    const std::vector<std::pair<size_t, size_t>>& historyPairs() const {
        return history_pairs_;
    }
    
    /**
     * @brief Get current options
     */
    const Options& options() const { return options_; }

private:
    Options options_;
    std::vector<std::pair<size_t, size_t>> history_pairs_;
    double last_detect_time_ = 0.0;

    PointCloudXYZI::Ptr buildSubmap(
        const KeyFrameVector& keyframes,
        int center_idx, int half_range, double resolution);
};

}  // namespace livo2_offline

#endif  // BACKEND_LOOP_DETECTOR_H
