/*
 * Offline Bag Reader for FAST-LIVO2
 * Implementation
 */

#include "offline/bag_reader.h"

#include <filesystem>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

namespace livo2_offline {

BagReader::BagReader(const std::string& bag_path, const std::string& storage_id)
    : bag_path_(bag_path), storage_id_(storage_id) {
    if (storage_id_.empty()) {
        storage_id_ = DetectStorageFormat();
    }
}

std::string BagReader::DetectStorageFormat() const {
    namespace fs = std::filesystem;
    
    fs::path p(bag_path_);
    
    // Check if it's a directory (typical for sqlite3 bags)
    if (fs::is_directory(p)) {
        // Look for metadata.yaml which indicates rosbag2 format
        if (fs::exists(p / "metadata.yaml")) {
            // Check for mcap or db3 files
            for (const auto& entry : fs::directory_iterator(p)) {
                std::string ext = entry.path().extension().string();
                if (ext == ".mcap") return "mcap";
                if (ext == ".db3") return "sqlite3";
            }
        }
        return "sqlite3"; // Default for directories
    }
    
    // Check file extension
    std::string ext = p.extension().string();
    if (ext == ".mcap") return "mcap";
    if (ext == ".db3") return "sqlite3";
    
    // Default to mcap for modern ROS2
    return "mcap";
}

BagReader& BagReader::AddHandle(const std::string& topic_name, MessageHandler handler) {
    handlers_[topic_name] = std::move(handler);
    return *this;
}

BagReader& BagReader::AddImuHandle(const std::string& topic_name, ImuCallback callback) {
    return AddHandle(topic_name, [this, callback](const MsgPtr& msg) -> bool {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        seri_imu_.deserialize_message(&serialized_msg, imu_msg.get());
        return callback(imu_msg);
    });
}

BagReader& BagReader::AddPointCloud2Handle(const std::string& topic_name, PointCloud2Callback callback) {
    return AddHandle(topic_name, [this, callback](const MsgPtr& msg) -> bool {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        seri_cloud2_.deserialize_message(&serialized_msg, cloud_msg.get());
        return callback(cloud_msg);
    });
}

BagReader& BagReader::AddLivoxHandle(const std::string& topic_name, LivoxCallback callback) {
    return AddHandle(topic_name, [this, callback](const MsgPtr& msg) -> bool {
        auto livox_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        seri_livox_.deserialize_message(&serialized_msg, livox_msg.get());
        return callback(livox_msg);
    });
}

BagReader& BagReader::AddOdomHandle(const std::string& topic_name, OdomCallback callback) {
    return AddHandle(topic_name, [this, callback](const MsgPtr& msg) -> bool {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        seri_odom_.deserialize_message(&serialized_msg, odom_msg.get());
        return callback(odom_msg);
    });
}

void BagReader::Go() {
    rosbag2_cpp::Reader reader;
    
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = storage_id_;
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        std::cerr << "[BagReader] Failed to open bag: " << bag_path_ 
                  << " with storage: " << storage_id_ 
                  << "\nError: " << e.what() << std::endl;
        throw;
    }
    
    std::cout << "[BagReader] Opened bag: " << bag_path_ 
              << " (format: " << storage_id_ << ")" << std::endl;
    
    size_t msg_count = 0;
    size_t processed_count = 0;
    
    while (reader.has_next()) {
        auto msg = reader.read_next();
        msg_count++;
        
        auto it = handlers_.find(msg->topic_name);
        if (it != handlers_.end()) {
            try {
                if (it->second(msg)) {
                    processed_count++;
                }
            } catch (const std::exception& e) {
                std::cerr << "[BagReader] Error processing message on topic " 
                          << msg->topic_name << ": " << e.what() << std::endl;
            }
        }
        
        // Progress indicator every 1000 messages
        if (msg_count % 1000 == 0) {
            std::cout << "\r[BagReader] Processed " << msg_count << " messages..." << std::flush;
        }
    }
    
    std::cout << "\n[BagReader] Finished. Total: " << msg_count 
              << " messages, Handled: " << processed_count << std::endl;
}

std::vector<std::string> BagReader::GetTopics() const {
    std::vector<std::string> topics;
    
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = storage_id_;
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    
    try {
        reader.open(storage_options, converter_options);
        auto metadata = reader.get_metadata();
        for (const auto& topic_info : metadata.topics_with_message_count) {
            topics.push_back(topic_info.topic_metadata.name);
        }
    } catch (const std::exception& e) {
        std::cerr << "[BagReader] Failed to get topics: " << e.what() << std::endl;
    }
    
    return topics;
}

size_t BagReader::GetMessageCount() const {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = storage_id_;
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    
    try {
        reader.open(storage_options, converter_options);
        auto metadata = reader.get_metadata();
        return metadata.message_count;
    } catch (const std::exception& e) {
        std::cerr << "[BagReader] Failed to get message count: " << e.what() << std::endl;
        return 0;
    }
}

void BagReader::ClearHandlers() {
    handlers_.clear();
}

} // namespace livo2_offline
