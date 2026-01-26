/*
 * Offline Bag Reader for FAST-LIVO2
 * Reads ROS2 bag files (mcap/sqlite3) and dispatches messages via callbacks
 */

#ifndef OFFLINE_BAG_READER_H
#define OFFLINE_BAG_READER_H

#include <functional>
#include <map>
#include <string>
#include <memory>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace livo2_offline {

/**
 * @brief ROS2 Bag reader for offline processing
 * 
 * Supports mcap and sqlite3 formats.
 * Reads messages sequentially and dispatches to registered callbacks.
 */
class BagReader {
public:
    using MsgPtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
    using MessageHandler = std::function<bool(const MsgPtr&)>;
    
    // Typed callbacks
    using ImuCallback = std::function<bool(sensor_msgs::msg::Imu::SharedPtr)>;
    using PointCloud2Callback = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using LivoxCallback = std::function<bool(livox_ros_driver2::msg::CustomMsg::SharedPtr)>;
    using OdomCallback = std::function<bool(nav_msgs::msg::Odometry::SharedPtr)>;

    /**
     * @brief Construct a new Bag Reader
     * @param bag_path Path to the bag file or directory
     * @param storage_id Storage format: "mcap" or "sqlite3" (auto-detected if empty)
     */
    explicit BagReader(const std::string& bag_path, const std::string& storage_id = "");

    ~BagReader() = default;

    /**
     * @brief Add generic message handler
     * @param topic_name Topic to handle
     * @param handler Callback function
     * @return Reference to this for chaining
     */
    BagReader& AddHandle(const std::string& topic_name, MessageHandler handler);

    /**
     * @brief Add IMU message handler
     * @param topic_name Topic to handle
     * @param callback Callback function receiving deserialized IMU message
     * @return Reference to this for chaining
     */
    BagReader& AddImuHandle(const std::string& topic_name, ImuCallback callback);

    /**
     * @brief Add PointCloud2 message handler
     * @param topic_name Topic to handle
     * @param callback Callback function receiving deserialized PointCloud2 message
     * @return Reference to this for chaining
     */
    BagReader& AddPointCloud2Handle(const std::string& topic_name, PointCloud2Callback callback);

    /**
     * @brief Add Livox CustomMsg handler
     * @param topic_name Topic to handle
     * @param callback Callback function receiving deserialized Livox message
     * @return Reference to this for chaining
     */
    BagReader& AddLivoxHandle(const std::string& topic_name, LivoxCallback callback);

    /**
     * @brief Add Odometry message handler
     * @param topic_name Topic to handle
     * @param callback Callback function receiving deserialized Odometry message
     * @return Reference to this for chaining
     */
    BagReader& AddOdomHandle(const std::string& topic_name, OdomCallback callback);

    /**
     * @brief Process all messages in the bag sequentially
     * Calls registered handlers for each message.
     */
    void Go();

    /**
     * @brief Get the list of topics in the bag
     * @return Vector of topic names
     */
    std::vector<std::string> GetTopics() const;

    /**
     * @brief Get total message count
     * @return Number of messages in the bag
     */
    size_t GetMessageCount() const;

    /**
     * @brief Clear all registered handlers
     */
    void ClearHandlers();

private:
    std::string bag_path_;
    std::string storage_id_;
    std::map<std::string, MessageHandler> handlers_;

    // Serializers for different message types
    rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud2_;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> seri_livox_;
    rclcpp::Serialization<nav_msgs::msg::Odometry> seri_odom_;

    /**
     * @brief Auto-detect storage format from file extension
     * @return Storage ID string
     */
    std::string DetectStorageFormat() const;
};

} // namespace livo2_offline

#endif // OFFLINE_BAG_READER_H
