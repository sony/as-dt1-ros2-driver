/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "asdt1_driver_node.h"
#include "calibration_parser.h"
#include "connect_status.h"
#include "dt1_modes.h"
#include "licenses.h"
#include "licenses_info.h"

#include "generated/version.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>
#include <regex>
#include <string>

#include <Eigen/Dense>

static float constexpr mm_to_m = 0.001f;

static std::string get_driver_version()
{
    std::string version = DRIVER_VERSION;
    if (!version.empty()) { return version; }

    try
    {
        const std::string share_dir =
            ament_index_cpp::get_package_share_directory("asdt1_ros2_driver");
        std::ifstream f(share_dir + "/package.xml");
        const std::string xml((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());
        std::smatch m;
        if (std::regex_search(xml, m, std::regex("<version>([^<]+)</version>")))
        {
            return m[1].str();
        }
    }
    catch (const std::exception&) {}

    return {};
}

static sync_mode string_to_sync_mode(const std::string& mode_str)
{
    static const std::unordered_map<std::string, sync_mode> sync_mode_map = {
        {"none", sync_mode::none}, {"master", sync_mode::master}, {"slave", sync_mode::slave}};
    auto mode = sync_mode_map.at(mode_str);

    return mode;
}

ASDT1_Driver::ASDT1_Driver(const rclcpp::NodeOptions& options) : Node("asdt1_driver", options)
{
    RCLCPP_INFO(this->get_logger(), "AS-DT1 ROS2 driver version: %s", get_driver_version().c_str());

    // ROS parameters
    declare_parameter<std::string>("calibration_file", "config/calibration.yml");
    declare_parameter<int>("serial_number", -1);
    declare_parameter<std::string>("mode", "30MSTD");
    declare_parameter<int>("framerate", 30);
    declare_parameter<std::string>("sync_mode", "none");
    declare_parameter<std::string>("output_format", "XYZ");

    std::string calib_file = get_parameter("calibration_file").as_string();
    std::string pc_type = get_parameter("output_format").as_string();
    int serial_number = get_parameter("serial_number").as_int();

    calibration_parser::init(calib_file);

    // Create the device instance with callbacks for connection and frame reception
    auto logger = this->get_logger();
    _dt1_device = dt1_device::create(
        pc_type, serial_number > 0 ? std::to_string(serial_number) : "",
        [this](connect_status status) { this->on_cam_connection(status); },
        [this](uint64_t device_ts, const points_data& points) { this->on_point_cloud(device_ts, points); },
        [this](const std::vector<imu_sample>& imu_samples) { this->on_imu_samples(imu_samples); },
        [logger](log_level level, const std::string& msg) {
            switch (level)
            {
                case log_level::debug: RCLCPP_DEBUG(logger, "%s", msg.c_str()); break;
                case log_level::info:  RCLCPP_INFO(logger,  "%s", msg.c_str()); break;
                case log_level::warn:  RCLCPP_WARN(logger,  "%s", msg.c_str()); break;
                case log_level::error: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
            }
        });

    // Set input parameters to device

    // Frame rate
    int framerate = static_cast<int>(get_parameter("framerate").as_int());
    _dt1_device->set_frame_rate(framerate);

    // Sync mode
    sync_mode sync;
    try
    {
        sync = string_to_sync_mode(get_parameter("sync_mode").as_string());
    }
    catch (const std::out_of_range& e)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid sync_mode '%s', defaulting to 'none'",
                    get_parameter("sync_mode").as_string().c_str());
        sync = sync_mode::none;
    }
    _dt1_device->set_sync_mode(sync);

    // Sensor mode
    std::string new_mode = get_parameter("mode").as_string();
    int mode_idx = get_mode_nbr(new_mode);
    if (mode_idx < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid mode '%s', defaulting to '30MSTD'", new_mode.c_str());
        mode_idx = get_mode_nbr("30MSTD");
    }
    _dt1_device->set_mode(mode_idx);

    _get_driver_version_service = create_service<std_srvs::srv::Trigger>(
        std::string("get_driver_version"),
        std::bind(&ASDT1_Driver::on_get_driver_version, this, std::placeholders::_1, std::placeholders::_2));

    _get_licenses_service = create_service<std_srvs::srv::Trigger>(
        std::string("get_licenses"),
        std::bind(&ASDT1_Driver::on_get_licenses, this, std::placeholders::_1, std::placeholders::_2));

    const rclcpp::QoS qos_pc = rclcpp::SensorDataQoS().keep_last(30);
    const rclcpp::QoS qos_imu = rclcpp::SensorDataQoS().keep_last(25);
    _pcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("asdt1_driver/point_cloud", qos_pc);
    _imu_publisher = create_publisher<sensor_msgs::msg::Imu>("asdt1_driver/imu", qos_imu);

    _set_parameters_callback_handle =
        add_on_set_parameters_callback(std::bind(&ASDT1_Driver::on_parameters_set, this, std::placeholders::_1));
    _reboot_service = create_service<std_srvs::srv::Trigger>(
        std::string("reboot"),
        std::bind(&ASDT1_Driver::on_reboot, this, std::placeholders::_1, std::placeholders::_2));
    _get_device_info_service = create_service<std_srvs::srv::Trigger>(
        std::string("get_device_info"),
        std::bind(&ASDT1_Driver::on_get_device_info, this, std::placeholders::_1, std::placeholders::_2));
    _start_streaming_service = create_service<std_srvs::srv::Trigger>(
        std::string("start_streaming"),
        std::bind(&ASDT1_Driver::on_start_streaming, this, std::placeholders::_1, std::placeholders::_2));
    _stop_streaming_service = create_service<std_srvs::srv::Trigger>(
        std::string("stop_streaming"),
        std::bind(&ASDT1_Driver::on_stop_streaming, this, std::placeholders::_1, std::placeholders::_2));

    _msg_fields.resize(3);

    int offset = 0;
    _msg_fields[0].name = "x";
    _msg_fields[0].offset = offset;
    _msg_fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _msg_fields[0].count = 1;
    offset += sizeof(float);

    _msg_fields[1].name = "y";
    _msg_fields[1].offset = offset;
    _msg_fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _msg_fields[1].count = 1;
    offset += sizeof(float);

    _msg_fields[2].name = "z";
    _msg_fields[2].offset = offset;
    _msg_fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _msg_fields[2].count = 1;
    offset += sizeof(float);

    if (pc_type == "XYZITR")
    {
        _msg_fields.resize(6);
        _msg_fields[3].name = "intensity";
        _msg_fields[3].offset = offset;
        _msg_fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
        _msg_fields[3].count = 1;
        offset += sizeof(uint32_t);

        _msg_fields[4].name = "time";
        _msg_fields[4].offset = offset;
        _msg_fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
        _msg_fields[4].count = 1;
        offset += sizeof(uint32_t);

        _msg_fields[5].name = "ring";
        _msg_fields[5].offset = offset;
        _msg_fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
        _msg_fields[5].count = 1;
    }

    _tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    _dt1_device->start();
}

void ASDT1_Driver::on_cam_connection(connect_status status)
{

    if (status.is_connected())
    {
        RCLCPP_INFO(this->get_logger(), "ASDT1_Driver: on_cam_connection: serial: %s",
                    status.get_device_info().hw_id.c_str());

        camera_calibration calibration;
        if (calibration_parser::get().get_calibration(status.get_device_info().hw_id, calibration))
        {
            _depth_frame_id = calibration.frame_id.empty() ? DEPTH_FRAME_ID : calibration.frame_id;
            _parent_frame_id =
                calibration.parent_frame_id.empty() ? COLOR_CAMERA_FRAME_ID : calibration.parent_frame_id;
        }
        else
        {
            _depth_frame_id = DEPTH_FRAME_ID;
            _parent_frame_id = COLOR_CAMERA_FRAME_ID;
        }

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->get_clock()->now();
        ts.header.frame_id = _parent_frame_id;
        ts.child_frame_id = _depth_frame_id;

        const auto translation = calibration.extrinsics.get_translation() * mm_to_m;
        ts.transform.translation.x = translation.x();
        ts.transform.translation.y = translation.y();
        ts.transform.translation.z = translation.z();

        const Eigen::Quaternionf eig_q(calibration.extrinsics.extrinsic_matrix.block<3, 3>(0, 0));
        ts.transform.rotation.w = eig_q.w();
        ts.transform.rotation.x = eig_q.x();
        ts.transform.rotation.y = eig_q.y();
        ts.transform.rotation.z = eig_q.z();

        _tf_broadcaster->sendTransform(ts);
    }
}

#pragma pack(push, 1)
struct xyzitr
{
    float x;
    float y;
    float z;
    uint32_t i;
    uint32_t t;
    uint16_t r;
};
#pragma pack(pop)

void ASDT1_Driver::on_point_cloud(uint64_t frame_ts, const points_data& points)
{
    if (!_pcloud_publisher) { return; }

    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = rclcpp::Time(frame_ts);
    msg.header.frame_id = _depth_frame_id;

    std::visit(
        [this, &msg](const auto& ref) {
            const auto& point_data = ref.get();
            using T = std::decay_t<decltype(point_data)>;

            msg.height = 1;
            msg.width = point_data.size();
            msg.is_bigendian = false;

            if constexpr (std::is_same_v<T, std::vector<Eigen::Vector3f>>)
            {
                msg.fields = _msg_fields;
                msg.point_step = sizeof(Eigen::Vector3f);
                msg.row_step = msg.point_step * msg.width;
                std::size_t byte_count = msg.row_step * msg.height;
                msg.data.resize(byte_count);
                std::memcpy(msg.data.data(), point_data.data(), byte_count);
            }
            else if constexpr (std::is_same_v<T, std::vector<Vector6f>>)
            {
                msg.fields = _msg_fields;
                msg.point_step = sizeof(Vector3f) + 2 * sizeof(uint32_t) + sizeof(uint16_t);
                msg.row_step = msg.point_step * msg.width;
                std::size_t byte_count = msg.row_step * msg.height;
                msg.data.resize(byte_count);
                xyzitr* pos = reinterpret_cast<xyzitr*>(msg.data.data());
                for (const Vector6f& p : point_data)
                {
                    pos->x = p[0];
                    pos->y = p[1];
                    pos->z = p[2];
                    pos->i = static_cast<uint32_t>(p[3]);
                    pos->t = static_cast<uint32_t>(p[4]);
                    pos->r = static_cast<uint16_t>(p[5]);
                    pos++;
                }
            }
        },
        points);

    msg.is_dense = false; // could contain invalid points (NAN)

    auto now = this->get_clock()->now();
    if ((now - _last_publish_log).seconds() >= 5.0)
    {
        RCLCPP_INFO(this->get_logger(), "AS-DT1 is publishing...");
        _last_publish_log = now;
    }

    _pcloud_publisher->publish(msg);
}

void ASDT1_Driver::on_imu_samples(const std::vector<imu_sample>& imu_samples)
{
    if (_imu_publisher && !imu_samples.empty())
    {
        for (const auto& sample : imu_samples)
        {
            sensor_msgs::msg::Imu imu_msg;

            imu_msg.header.stamp = rclcpp::Time(sample.timestamp);
            imu_msg.header.frame_id = _depth_frame_id;

            imu_msg.orientation_covariance[0] = -1;

            // Variance is (noise denity)^2 / dt , where dt is 100Hz (1/100)
            // Noise denities friom imu.yml
            imu_msg.linear_acceleration_covariance = {3.25895e-06, 0.0, 0.0, 0.0,        3.25895e-06,
                                                      0.0,         0.0, 0.0, 3.25895e-06};

            imu_msg.angular_velocity_covariance = {6.43051e-06, 0.0, 0.0, 0.0, 6.43051e-06, 0.0, 0.0, 0.0, 6.43051e-06};

            imu_msg.angular_velocity.x = sample.gyro.x();
            imu_msg.angular_velocity.y = sample.gyro.y();
            imu_msg.angular_velocity.z = sample.gyro.z();

            imu_msg.linear_acceleration.x = sample.accel.x();
            imu_msg.linear_acceleration.y = sample.accel.y();
            imu_msg.linear_acceleration.z = sample.accel.z();

            _imu_publisher->publish(imu_msg);
        }
    }
}

rcl_interfaces::msg::SetParametersResult ASDT1_Driver::on_parameters_set(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool reboot = false;
    for (const auto& parameter : parameters)
    {
        // Avoid non runtime changeable parameters
        if (parameter.get_name() == "calibration_file" || parameter.get_name() == "serial_number")
        {
            result.successful = false;
            result.reason = "Parameter '" + parameter.get_name() + "' cannot be changed at runtime";
            return result;
        }

        // Mode
        if (parameter.get_name() == "mode")
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = false;
                result.reason = "Parameter 'mode' must be an string";
                return result;
            }
            int mode_idx = get_mode_nbr(parameter.as_string());
            if (mode_idx < 0)
            {
                result.successful = false;
                result.reason = "Parameter 'mode' must be one of: " + get_available_modes_str();
                return result;
            }
            _dt1_device->stop();
            _dt1_device->set_mode(mode_idx);
            reboot = true;
        }

        // Frame rate
        if (parameter.get_name() == "framerate")
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                result.successful = false;
                result.reason = "Parameter 'framerate' must be an integer";
                return result;
            }
            int new_framerate = static_cast<int>(parameter.as_int());
            if (new_framerate < 0 || new_framerate > 30)
            {
                result.successful = false;
                result.reason = "Parameter 'framerate' must be >= 0";
                return result;
            }
            _dt1_device->set_frame_rate(new_framerate);
            reboot = true;
        }

        // Sync mode
        if (parameter.get_name() == "sync_mode")
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = false;
                result.reason = "Parameter 'sync_mode' must be a string";
                return result;
            }

            sync_mode sync;
            try
            {
                sync = string_to_sync_mode(parameter.as_string());
            }
            catch (const std::out_of_range& e)
            {
                result.successful = false;
                result.reason = "Parameter 'sync_mode' must be one of: master, slave, none";
                return result;
            }

            _dt1_device->set_sync_mode(sync);
            reboot = true;
        }
    }

    if (reboot) { _dt1_device->reboot(); }

    return result;
}

void ASDT1_Driver::on_reboot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    try
    {
        _dt1_device->reboot();
        response->success = true;
        response->message = "Reboot requested";
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = std::string("Failed to reboot: ") + e.what();
    }
}

void ASDT1_Driver::on_get_device_info(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    const auto& info = _dt1_device->get_info();

    std::ostringstream oss;
    oss << "hw_id=" << info.hw_id << ", version=" << info.device_version
        << ", sync_mode=" << static_cast<int>(_dt1_device->get_sync_mode());

    response->success = true;
    response->message = oss.str();
}

void ASDT1_Driver::on_get_driver_version(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    const std::string version = get_driver_version();
    response->success = !version.empty();
    response->message = response->success ? version : "Driver version unavailable";
}

void ASDT1_Driver::on_get_licenses(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    response->success = true;
    response->message = get_licenses_info(licenses);
    RCLCPP_INFO(this->get_logger(), "\n%s", response->message.c_str());
}

void ASDT1_Driver::on_start_streaming(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    _dt1_device->start();
    response->success = true;
    response->message = "Streaming enabled";
}

void ASDT1_Driver::on_stop_streaming(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    _dt1_device->stop();
    response->success = true;
    response->message = "Streaming disabled";
}

RCLCPP_COMPONENTS_REGISTER_NODE(ASDT1_Driver)
