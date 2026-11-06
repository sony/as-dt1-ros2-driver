/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class PointCloudCombiner : public rclcpp::Node
{
public:
    explicit PointCloudCombiner(const rclcpp::NodeOptions& options);

private:
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PointCloudConstPtr = PointCloudMsg::ConstSharedPtr;
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImuConstPtr = ImuMsg::ConstSharedPtr;
    static constexpr int SYNC_TOLERANCE_MS = 25;
    static constexpr int IMU_SYNC_TOLERANCE_MS = 10;

    struct offsets
    {
        int x = 0;
        int y = 0;
        int z = 0;
        int intensity = -1;
        int time = -1;
        int ring = -1;
    };

    struct target_transform
    {
        tf2::Matrix3x3 rotation = tf2::Matrix3x3::getIdentity();
        tf2::Vector3 translation{0.0, 0.0, 0.0};
    };

    static std::optional<offsets> find_offsets(const PointCloudMsg& cloud);
    bool get_transform_to_target_frame(std::size_t input_index, const PointCloudMsg& in_cloud,
                                       target_transform& transform);
    bool transform_to_target_frame(std::size_t input_index, const PointCloudMsg& in_cloud, PointCloudMsg& out_cloud,
                                   int& ring_offset);
    bool transform_imu_vector(std::size_t imu_index, const geometry_msgs::msg::Vector3& vector,
                              tf2::Vector3& transformed);
    void on_point_cloud(std::size_t index, PointCloudConstPtr msg);
    void on_imu(std::size_t index, ImuConstPtr msg);
    void try_publish_combined();
    void try_publish_combined_imu();
    void on_get_licenses(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool build_combined_cloud(const std::vector<PointCloudMsg>& clouds, PointCloudMsg& out_msg);

    std::vector<std::string> _input_topics;
    std::vector<std::string> _imu_topics;
    std::string _output_topic;
    std::string _output_imu_topic;
    std::string _target_frame;

    std::vector<rclcpp::Subscription<PointCloudMsg>::SharedPtr> _subscriptions;
    std::vector<rclcpp::Subscription<ImuMsg>::SharedPtr> _imu_subscriptions;

    std::vector<PointCloudConstPtr> _latest_clouds;
    std::vector<ImuConstPtr> _latest_imus;
    mutable std::mutex _clouds_mutex;
    mutable std::mutex _imus_mutex;

    rclcpp::Publisher<PointCloudMsg>::SharedPtr _publisher;
    rclcpp::Publisher<ImuMsg>::SharedPtr _imu_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _get_licenses_service;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::vector<std::optional<target_transform>> _target_transforms;
    mutable std::mutex _target_transforms_mutex;

};
