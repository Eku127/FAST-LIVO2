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
#include <fstream>

#include "offline/bag_reader.h"
#include "offline/offline_liv_mapper.h"

using namespace livo2_offline;

// Global flag for graceful shutdown
static bool g_shutdown = false;
static OfflineLIVMapper* g_mapper = nullptr;
static std::string g_output_dir;

void signalHandler(int signum) {
    std::cout << "\n[Main] Interrupt signal (" << signum << ") received." << std::endl;
    g_shutdown = true;
    
    // Save intermediate results on interrupt
    if (g_mapper && !g_output_dir.empty()) {
        std::cout << "[Main] Saving intermediate results before exit..." << std::endl;
        g_mapper->savePosesTUM(g_output_dir + "/poses.txt");
        g_mapper->saveKeyframePoses(g_output_dir + "/keyframes.txt");
#ifdef USE_BACKEND
        g_mapper->saveBackendOutput();
#endif
        // Note: keyframe clouds are saved incrementally, no need to save here
        std::cout << "[Main] Intermediate results saved." << std::endl;
    }
}

/**
 * @brief Check if output directory is non-empty and prompt for overwrite
 * @return true if safe to proceed, false if user wants to abort
 */
bool checkOutputDirectory(const std::string& output_dir) {
    namespace fs = std::filesystem;
    
    if (!fs::exists(output_dir)) {
        return true;  // Directory doesn't exist, safe to create
    }
    
    // Check for specific output files
    std::vector<std::string> existing_files;
    
    if (fs::exists(output_dir + "/poses.txt")) {
        existing_files.push_back("poses.txt");
    }
    if (fs::exists(output_dir + "/keyframes.txt")) {
        existing_files.push_back("keyframes.txt");
    }
    if (fs::exists(output_dir + "/map.pcd")) {
        existing_files.push_back("map.pcd");
    }
    if (fs::exists(output_dir + "/keyframes") && !fs::is_empty(output_dir + "/keyframes")) {
        existing_files.push_back("keyframes/");
    }
    
    if (existing_files.empty()) {
        return true;  // No conflicting output files
    }
    
    std::cout << "\n[Warning] Output directory already contains results: " << output_dir << std::endl;
    std::cout << "Existing files:" << std::endl;
    for (const auto& f : existing_files) {
        std::cout << "  - " << f << std::endl;
    }
    
    std::cout << "\nOverwrite? [y/N]: ";
    
    std::string choice;
    std::getline(std::cin, choice);
    
    return (!choice.empty() && (choice[0] == 'y' || choice[0] == 'Y'));
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
    
    // Store output_dir globally for signal handler
    g_output_dir = output_dir;
    
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
    
    // Check output directory before proceeding
    if (!checkOutputDirectory(output_dir)) {
        std::cout << "[Main] Aborted by user." << std::endl;
        return 0;
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
    
    // Set output directory for incremental saving
    mapper.setOutputDirectory(output_dir);
    
    // Store mapper pointer globally for signal handler
    g_mapper = &mapper;
    
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
    
    // Save final results
    std::cout << "[Main] Saving final results..." << std::endl;
    // Save full trajectory (TUM format)
    mapper.savePosesTUM(output_dir + "/poses.txt");
    
    // Save keyframe poses
    mapper.saveKeyframePoses(output_dir + "/keyframes.txt");

#ifdef USE_BACKEND
    // Save backend output (g2o, loop constraints)
    mapper.saveBackendOutput();
#endif

    // Note: keyframe point clouds are saved incrementally during processing
    // Only save global map at the end (requires all keyframes)
    mapper.saveGlobalMap(output_dir + "/map.pcd", 0.1);
    std::cout << "\n[Main] All results saved to: " << output_dir << std::endl;
    std::cout << "  - poses.txt       : Full trajectory (TUM format)\n"
              << "  - keyframes.txt   : Keyframe poses\n"
              << "  - keyframes/      : Keyframe point clouds (saved incrementally)\n"
              << "  - map.pcd         : Global map\n" << std::endl;
    
    // Clear global mapper pointer before exit
    g_mapper = nullptr;
    
    // Explicitly shutdown mapper to cleanup resources before destructor
    mapper.shutdown();
    std::cout << "[Main] Done!" << std::endl;
    return g_shutdown ? 130 : 0;  // Return 130 if interrupted
}
