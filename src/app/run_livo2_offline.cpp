/*
 * FAST-LIVO2 Offline Processing
 * 
 * Processes ROS2 bag files offline for high-quality mapping
 * Outputs trajectory (TUM format) and keyframe point clouds
 * 
 * Usage:
 *   ./run_livo2_offline <bag_path> <config_path> [output_dir]
 */

#include <iostream>
#include <filesystem>
#include <chrono>
#include <csignal>

#include "offline/bag_reader.h"
#include "offline/offline_liv_mapper.h"

using namespace livo2_offline;

// Global flag for graceful shutdown
static bool g_shutdown = false;

void signalHandler(int signum) {
    std::cout << "\n[Main] Interrupt signal (" << signum << ") received." << std::endl;
    g_shutdown = true;
}

void printUsage(const char* program) {
    std::cout << "FAST-LIVO2 Offline Mapping\n"
              << "===========================\n\n"
              << "Usage:\n"
              << "  " << program << " <bag_path> <config_path> [output_dir]\n\n"
              << "Arguments:\n"
              << "  bag_path    - Path to ROS2 bag file/directory (mcap or sqlite3)\n"
              << "  config_path - Path to YAML config file (same as online LIVO2)\n"
              << "  output_dir  - Output directory (default: ./output)\n\n"
              << "Example:\n"
              << "  " << program << " /data/bag.mcap ./config/mid360.yaml ./output\n"
              << std::endl;
}

int main(int argc, char** argv) {
    // Register signal handler
    signal(SIGINT, signalHandler);
    
    // Parse arguments
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string bag_path = argv[1];
    std::string config_path = argv[2];
    std::string output_dir = (argc > 3) ? argv[3] : "./output";
    
    // Validate paths
    namespace fs = std::filesystem;
    
    if (!fs::exists(bag_path)) {
        std::cerr << "[Main] Bag file not found: " << bag_path << std::endl;
        return 1;
    }
    
    if (!fs::exists(config_path)) {
        std::cerr << "[Main] Config file not found: " << config_path << std::endl;
        return 1;
    }
    
    // Create output directories
    fs::create_directories(output_dir);
    fs::create_directories(output_dir + "/keyframes");
    
    std::cout << "================================================\n"
              << "        FAST-LIVO2 Offline Mapping\n"
              << "================================================\n"
              << "Bag:     " << bag_path << "\n"
              << "Config:  " << config_path << "\n"
              << "Output:  " << output_dir << "\n"
              << "================================================\n" << std::endl;
    
    // Initialize mapper
    OfflineLIVMapper mapper(config_path);
    
    // Get topic names from config
    std::string imu_topic = mapper.config().imu_topic;
    std::string lidar_topic = mapper.config().lidar_topic;
    
    std::cout << "[Main] Topics:\n"
              << "  IMU:   " << imu_topic << "\n"
              << "  LiDAR: " << lidar_topic << "\n" << std::endl;
    
    // Initialize bag reader
    BagReader reader(bag_path);
    
    // Print available topics
    auto topics = reader.GetTopics();
    std::cout << "[Main] Available topics in bag:" << std::endl;
    for (const auto& topic : topics) {
        std::cout << "  - " << topic << std::endl;
    }
    std::cout << std::endl;
    
    // Counters
    size_t imu_count = 0;
    size_t lidar_count = 0;
    
    // Register IMU handler
    reader.AddImuHandle(imu_topic, [&](sensor_msgs::msg::Imu::SharedPtr msg) {
        if (g_shutdown) return false;
        mapper.addImu(msg);
        imu_count++;
        return true;
    });
    
    // Register LiDAR handler
    // Try Livox first, then standard PointCloud2
    bool livox_found = false;
    for (const auto& topic : topics) {
        if (topic == lidar_topic) {
            // Check if it's Livox by topic name convention
            if (topic.find("livox") != std::string::npos) {
                livox_found = true;
            }
            break;
        }
    }
    
    if (livox_found) {
        reader.AddLivoxHandle(lidar_topic, [&](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
            if (g_shutdown) return false;
            mapper.addLivox(msg);
            lidar_count++;
            
            // Process after each LiDAR frame
            while (mapper.processOnce()) {
                // Continue processing while data available
            }
            return true;
        });
        std::cout << "[Main] Using Livox CustomMsg handler" << std::endl;
    } else {
        reader.AddPointCloud2Handle(lidar_topic, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            if (g_shutdown) return false;
            mapper.addPointCloud2(msg);
            lidar_count++;
            
            // Process after each LiDAR frame
            while (mapper.processOnce()) {
                // Continue processing while data available
            }
            return true;
        });
        std::cout << "[Main] Using PointCloud2 handler" << std::endl;
    }
    
    // Start processing
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "\n[Main] Processing started..." << std::endl;
    
    reader.Go();
    
    // Process any remaining data
    while (mapper.processOnce() && !g_shutdown) {}
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();
    
    // Print summary
    std::cout << "\n================================================\n"
              << "                Processing Complete\n"
              << "================================================\n"
              << "Time:       " << std::fixed << std::setprecision(2) << elapsed << " seconds\n"
              << "IMU msgs:   " << imu_count << "\n"
              << "LiDAR msgs: " << lidar_count << "\n"
              << "Frames:     " << mapper.frameCount() << "\n"
              << "Keyframes:  " << mapper.keyframes().size() << "\n"
              << "Trajectory: " << mapper.trajectory().size() << " poses\n"
              << "================================================\n" << std::endl;
    
    // Save results
    std::cout << "[Main] Saving results..." << std::endl;
    
    // Save full trajectory (TUM format)
    mapper.savePosesTUM(output_dir + "/poses.txt");
    
    // Save keyframe poses
    mapper.saveKeyframePoses(output_dir + "/keyframes.txt");
    
    // Save keyframe point clouds
    mapper.saveKeyframeClouds(output_dir + "/keyframes");
    
    // Save global map
    mapper.saveGlobalMap(output_dir + "/map.pcd", 0.1);
    
    std::cout << "\n[Main] All results saved to: " << output_dir << std::endl;
    std::cout << "  - poses.txt       : Full trajectory (TUM format)\n"
              << "  - keyframes.txt   : Keyframe poses\n"
              << "  - keyframes/      : Keyframe point clouds (PCD)\n"
              << "  - map.pcd         : Global map\n" << std::endl;
    
    std::cout << "[Main] Done!" << std::endl;
    
    return g_shutdown ? 130 : 0;  // Return 130 if interrupted
}
