/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <variant>
#include <vector>

using Vector3f = Eigen::Vector3f;
using Vector6f = Eigen::Matrix<float, 6, 1>;

enum class sync_mode
{
    none = 0,
    master = 1,
    slave = 2
};

struct imu_sample
{
    uint64_t timestamp;
    Vector3f accel;
    Vector3f gyro;
    int temp;
};


using points_data = std::variant<
    std::reference_wrapper<const std::vector<Vector3f>>,
    std::reference_wrapper<const std::vector<Vector6f>>
>;

using point_cloud_callback = std::function<void(uint64_t frame_ts, const points_data& points)>;
using imu_callback = std::function<void(const std::vector<imu_sample>& imu_samples)>;

enum class log_level { debug, info, warn, error };
using log_callback = std::function<void(log_level, const std::string&)>;
