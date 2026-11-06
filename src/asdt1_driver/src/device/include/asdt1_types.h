/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <variant>
#include <vector>

struct Vector3f
{
    float x;
    float y;
    float z;
};
static_assert(sizeof(Vector3f) == 3 * sizeof(float), "Vector3f must be tightly packed");

// x, y, z, intensity, time, ring
using Vector6f = std::array<float, 6>;
static_assert(sizeof(Vector6f) == 6 * sizeof(float), "Vector6f must be tightly packed");

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
