/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "point_cloud_combiner_node.h"
#include "licenses.h"
#include "licenses_info.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <functional>
#include <cstdint>
#include <cstring>

PointCloudCombiner::PointCloudCombiner(const rclcpp::NodeOptions& options) : Node("point_cloud_combiner", options)
{
    const std::vector<std::string> default_input_topics;
    _input_topics = this->declare_parameter<std::vector<std::string>>("input_topics", default_input_topics);
    _imu_topics = this->declare_parameter<std::vector<std::string>>("imu_topics", default_input_topics);
    _output_topic = this->declare_parameter<std::string>("output_topic", "combined_point_cloud");
    _output_imu_topic = this->declare_parameter<std::string>("output_imu_topic", "combined_imu");
    _target_frame = this->declare_parameter<std::string>("target_frame", "base_link");
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    _get_licenses_service = create_service<std_srvs::srv::Trigger>(
        std::string("get_licenses"),
        std::bind(&PointCloudCombiner::on_get_licenses, this, std::placeholders::_1, std::placeholders::_2));

    if (_input_topics.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'input_topics' is empty. No subscriptions will be created.");
        return;
    }

    const rclcpp::QoS qos = rclcpp::SensorDataQoS();
    _publisher = this->create_publisher<PointCloudMsg>(_output_topic, qos);
    _target_transforms.resize(_input_topics.size());
    _latest_clouds.resize(_input_topics.size());

    for (std::size_t i = 0; i < _input_topics.size(); ++i)
    {
        _subscriptions.push_back(
            this->create_subscription<PointCloudMsg>(
                _input_topics[i], qos,
                [this, i](PointCloudConstPtr msg) { on_point_cloud(i, std::move(msg)); }));
    }

    if (!_imu_topics.empty())
    {
        _latest_imus.resize(_imu_topics.size());
        _imu_publisher = this->create_publisher<ImuMsg>(_output_imu_topic, qos);
        for (std::size_t i = 0; i < _imu_topics.size(); ++i)
        {
            _imu_subscriptions.push_back(
                this->create_subscription<ImuMsg>(
                    _imu_topics[i], qos,
                    [this, i](ImuConstPtr msg) { on_imu(i, std::move(msg)); }));
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Point cloud combiner started with %zu input topic(s), output topic: %s, sync tolerance: %d ms",
                _input_topics.size(), _output_topic.c_str(), SYNC_TOLERANCE_MS);
    RCLCPP_INFO(this->get_logger(), "IMU fusion %s, output topic: %s, sync tolerance: %d ms",
                _imu_publisher ? "enabled" : "disabled", _output_imu_topic.c_str(), IMU_SYNC_TOLERANCE_MS);
}

void PointCloudCombiner::on_point_cloud(std::size_t index, PointCloudConstPtr msg)
{
    std::scoped_lock lock(_clouds_mutex);
    _latest_clouds[index] = std::move(msg);
    try_publish_combined();
}

void PointCloudCombiner::on_imu(std::size_t index, ImuConstPtr msg)
{
    std::scoped_lock lock(_imus_mutex);
    _latest_imus[index] = std::move(msg);
    try_publish_combined_imu();
}

void PointCloudCombiner::try_publish_combined()
{
    // Check all slots filled
    for (const auto& c : _latest_clouds)
    {
        if (!c) { return; }
    }

    // Check all within tolerance of the newest stamp
    rclcpp::Time newest(_latest_clouds[0]->header.stamp);
    for (const auto& c : _latest_clouds)
    {
        rclcpp::Time t(c->header.stamp);
        if (t > newest) { newest = t; }
    }
    for (const auto& c : _latest_clouds)
    {
        const double age_ms = (newest - rclcpp::Time(c->header.stamp)).seconds() * 1000.0;
        if (age_ms > SYNC_TOLERANCE_MS) { return; }
    }

    std::vector<PointCloudMsg> transformed;
    transformed.reserve(_latest_clouds.size());
    int ring_offset = 0;
    for (std::size_t i = 0; i < _latest_clouds.size(); ++i)
    {
        PointCloudMsg out;
        if (!transform_to_target_frame(i, *_latest_clouds[i], out, ring_offset)) { return; }
        transformed.push_back(std::move(out));
    }

    PointCloudMsg combined;
    if (!build_combined_cloud(transformed, combined)) { return; }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Publishing combined point cloud...");
    _publisher->publish(combined);
}

void PointCloudCombiner::try_publish_combined_imu()
{
    if (!_imu_publisher) { return; }

    for (const auto& m : _latest_imus)
    {
        if (!m) { return; }
    }

    rclcpp::Time newest(_latest_imus[0]->header.stamp);
    for (const auto& m : _latest_imus)
    {
        rclcpp::Time t(m->header.stamp);
        if (t > newest) { newest = t; }
    }
    for (const auto& m : _latest_imus)
    {
        const double age_ms = (newest - rclcpp::Time(m->header.stamp)).seconds() * 1000.0;
        if (age_ms > IMU_SYNC_TOLERANCE_MS) { return; }
    }

    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < _latest_imus.size(); ++i)
    {
        Eigen::Vector3d av, la;
        if (!transform_imu_vector(i, _latest_imus[i]->angular_velocity, av) ||
            !transform_imu_vector(i, _latest_imus[i]->linear_acceleration, la))
        {
            return;
        }
        angular_velocity += av;
        linear_acceleration += la;
    }

    const double n = static_cast<double>(_latest_imus.size());
    angular_velocity /= n;
    linear_acceleration /= n;

    const auto& first = _latest_imus[0];
    sensor_msgs::msg::Imu fused = *first;
    fused.header.stamp = first->header.stamp;
    fused.header.frame_id = _target_frame;
    fused.orientation_covariance[0] = -1.0;
    fused.angular_velocity.x = angular_velocity.x();
    fused.angular_velocity.y = angular_velocity.y();
    fused.angular_velocity.z = angular_velocity.z();
    fused.linear_acceleration.x = linear_acceleration.x();
    fused.linear_acceleration.y = linear_acceleration.y();
    fused.linear_acceleration.z = linear_acceleration.z();

    if (first->angular_velocity_covariance[0] >= 0.0)
    {
        for (std::size_t i = 0; i < fused.angular_velocity_covariance.size(); ++i)
        {
            fused.angular_velocity_covariance[i] = first->angular_velocity_covariance[i] / n;
        }
    }
    else { fused.angular_velocity_covariance[0] = -1.0; }

    if (first->linear_acceleration_covariance[0] >= 0.0)
    {
        for (std::size_t i = 0; i < fused.linear_acceleration_covariance.size(); ++i)
        {
            fused.linear_acceleration_covariance[i] = first->linear_acceleration_covariance[i] / n;
        }
    }
    else { fused.linear_acceleration_covariance[0] = -1.0; }

    _imu_publisher->publish(fused);
}

std::optional<PointCloudCombiner::offsets> PointCloudCombiner::find_offsets(const sensor_msgs::msg::PointCloud2& cloud)
{
    int x = -1;
    int y = -1;
    int z = -1;
    int intensity = -1;
    int time = -1;
    int ring = -1;
    for (const auto& field : cloud.fields)
    {
        if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) { x = field.offset; }
        if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) { y = field.offset; }
        if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) { z = field.offset; }
        if (field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::UINT32)
        {
            intensity = field.offset;
        }
        if (field.name == "time" && field.datatype == sensor_msgs::msg::PointField::UINT32) { time = field.offset; }
        if (field.name == "ring" && field.datatype == sensor_msgs::msg::PointField::UINT16) { ring = field.offset; }
    }

    if (x < 0 || y < 0 || z < 0) { return std::nullopt; }
    return offsets{x, y, z, intensity, time, ring};
}

bool PointCloudCombiner::get_transform_to_target_frame(const std::size_t input_index,
                                                       const sensor_msgs::msg::PointCloud2& in_cloud,
                                                       target_transform& transform)
{
    {
        std::scoped_lock lock(_target_transforms_mutex);
        if (input_index < _target_transforms.size() && _target_transforms[input_index])
        {
            transform = _target_transforms[input_index].value();
            return true;
        }
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try
    {
        tf_msg = _tf_buffer->lookupTransform(_target_frame, in_cloud.header.frame_id, in_cloud.header.stamp,
                                             rclcpp::Duration::from_seconds(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed TF transform %s -> %s: %s",
                             in_cloud.header.frame_id.c_str(), _target_frame.c_str(), ex.what());
        return false;
    }

    const auto& translation = tf_msg.transform.translation;
    const auto& rotation = tf_msg.transform.rotation;
    tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 rot(q);

    transform.translation = Eigen::Vector3d(translation.x, translation.y, translation.z);
    transform.rotation << rot[0][0], rot[0][1], rot[0][2], rot[1][0], rot[1][1], rot[1][2], rot[2][0], rot[2][1],
        rot[2][2];

    {
        std::scoped_lock lock(_target_transforms_mutex);
        if (input_index < _target_transforms.size()) { _target_transforms[input_index] = transform; }
    }

    return true;
}

bool PointCloudCombiner::transform_to_target_frame(const std::size_t input_index,
                                                   const sensor_msgs::msg::PointCloud2& in_cloud,
                                                   sensor_msgs::msg::PointCloud2& out_cloud,
                                                   int& ring_offset)
{
    out_cloud = in_cloud;

    target_transform transform;
    if (!get_transform_to_target_frame(input_index, in_cloud, transform)) { return false; }

    auto offsets = find_offsets(in_cloud);
    if (!offsets)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Point cloud in frame '%s' has no float32 x/y/z fields. Skipping cloud.",
                             in_cloud.header.frame_id.c_str());
        return false;
    }

    int max_ring = 0;
    const std::size_t point_count =
        static_cast<std::size_t>(in_cloud.width) * static_cast<std::size_t>(in_cloud.height);
    for (std::size_t i = 0; i < point_count; ++i)
    {
        const std::size_t base = i * in_cloud.point_step;

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        std::memcpy(&x, &out_cloud.data[base + offsets->x], sizeof(float));
        std::memcpy(&y, &out_cloud.data[base + offsets->y], sizeof(float));
        std::memcpy(&z, &out_cloud.data[base + offsets->z], sizeof(float));

        const Eigen::Vector3d transformed = transform.rotation * Eigen::Vector3d(x, y, z) + transform.translation;
        const float xt = static_cast<float>(transformed.x());
        const float yt = static_cast<float>(transformed.y());
        const float zt = static_cast<float>(transformed.z());
        std::memcpy(&out_cloud.data[base + offsets->x], &xt, sizeof(float));
        std::memcpy(&out_cloud.data[base + offsets->y], &yt, sizeof(float));
        std::memcpy(&out_cloud.data[base + offsets->z], &zt, sizeof(float));

        if (offsets->ring >= 0)
        {
            uint16_t ring = 0;
            std::memcpy(&ring, &out_cloud.data[base + offsets->ring], sizeof(uint16_t));
            if (ring > max_ring) { max_ring = ring; }
            const uint16_t r = ring + ring_offset;
            std::memcpy(&out_cloud.data[base + offsets->ring], &r, sizeof(uint16_t));
        }
    }

    if (offsets->ring >= 0) { ring_offset += max_ring + 1; }
    out_cloud.header.frame_id = _target_frame;
    return true;
}

bool PointCloudCombiner::transform_imu_vector(const std::size_t imu_index, const geometry_msgs::msg::Vector3& vector,
                                              Eigen::Vector3d& transformed)
{
    const Eigen::Vector3d source(vector.x, vector.y, vector.z);
    std::optional<target_transform> transform;
    {
        std::scoped_lock lock(_target_transforms_mutex);
        if (imu_index < _target_transforms.size()) { transform = _target_transforms[imu_index]; }
    }

    if (!transform)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No cached point cloud transform for IMU index %zu. Skipping IMU fusion.", imu_index);
        return false;
    }

    transformed = transform->rotation * source;
    return true;
}

bool PointCloudCombiner::build_combined_cloud(const std::vector<PointCloudMsg>& clouds,
                                              sensor_msgs::msg::PointCloud2& out_msg)
{
    const sensor_msgs::msg::PointCloud2* base = nullptr;
    std::size_t total_points = 0;

    for (std::size_t i = 0; i < clouds.size(); ++i)
    {
        const auto& cloud = clouds[i];
        if (base == nullptr) { base = &cloud; }
        else
        {
            if (cloud.point_step != base->point_step || cloud.fields != base->fields)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Skipping cloud from topic %s due to field/point_step mismatch.",
                                     _input_topics[i].c_str());
                continue;
            }
        }
        total_points += static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
    }

    if (base == nullptr || total_points == 0) { return false; }

    out_msg.header = base->header;
    out_msg.header.frame_id = _target_frame;
    out_msg.height = 1;
    out_msg.width = static_cast<uint32_t>(total_points);
    out_msg.fields = base->fields;
    out_msg.is_bigendian = base->is_bigendian;
    out_msg.point_step = base->point_step;
    out_msg.row_step = out_msg.point_step * out_msg.width;
    out_msg.is_dense = true;
    out_msg.data.clear();
    out_msg.data.reserve(static_cast<std::size_t>(total_points) * base->point_step);

    for (const auto& cloud : clouds)
    {
        if (cloud.point_step != base->point_step || cloud.fields != base->fields) { continue; }

        out_msg.data.insert(out_msg.data.end(), cloud.data.begin(), cloud.data.end());
        out_msg.is_dense = out_msg.is_dense && cloud.is_dense;
        if (rclcpp::Time(cloud.header.stamp) > rclcpp::Time(out_msg.header.stamp))
        {
            out_msg.header.stamp = cloud.header.stamp;
        }
    }

    return !out_msg.data.empty();
}

void PointCloudCombiner::on_get_licenses(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    response->success = true;
    response->message = get_licenses_info(licenses);
    RCLCPP_INFO(this->get_logger(), "\n%s", response->message.c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudCombiner)
