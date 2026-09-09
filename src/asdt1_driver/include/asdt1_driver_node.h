/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "asdt1_types.h"
#include "connect_status.h"
#include "dt1_device.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ASDT1_Driver : public rclcpp::Node
{
public:
    explicit ASDT1_Driver(const rclcpp::NodeOptions& options);

private:
    static inline constexpr char COLOR_CAMERA_FRAME_ID[] = "camera_system_optical";
    static inline constexpr char DEPTH_FRAME_ID[] = "asdt1_optical";

    void on_cam_connection(connect_status info);
    void on_point_cloud(uint64_t device_ts, const points_data& points);
    void on_imu_samples(const std::vector<imu_sample>& imu_samples);
    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& parameters);
    void on_reboot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_get_device_info(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_get_driver_version(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_get_licenses(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_start_streaming(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_stop_streaming(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pcloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reboot_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _get_driver_version_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _get_device_info_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _get_licenses_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _start_streaming_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _stop_streaming_service;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _set_parameters_callback_handle;

    std::vector<sensor_msgs::msg::PointField> _msg_fields;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_broadcaster;

    std::string _depth_frame_id = DEPTH_FRAME_ID;
    std::string _parent_frame_id = COLOR_CAMERA_FRAME_ID;
    rclcpp::Time _last_publish_log{0, 0, RCL_ROS_TIME};

    std::unique_ptr<dt1_device> _dt1_device;
};
