/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dt1_device_cdc.h"

#include "dt1_modes.h"
#include "logger.h"

#include <cinttypes>
#include <limits>
#include <stdexcept>

static constexpr int line_order[] = {0, 2, 12, 14, 4, 6, 16, 18, 8, 10, 20, 22,
                                     1, 3, 13, 15, 5, 7, 17, 19, 9, 11, 21, 23};

static void extract_first_peak(const std::vector<uint8_t>& data, size_t offset, Eigen::Vector3f& peak1)
{
    if (offset + 15 > data.size()) { throw std::out_of_range("Not enough data to extract 15 bytes"); }

    int x1 = data[offset + 0] << 12 | data[offset + 1] << 4 | data[offset + 2] >> 4;
    int y1 = (data[offset + 2] & 0x0f) << 16 | data[offset + 3] << 8 | data[offset + 4];
    int z1 = data[offset + 5] << 12 | data[offset + 6] << 4 | data[offset + 7] >> 4;

    if (x1 & 0x80000) { x1 = 0 - (0x100000 - x1); }
    if (y1 & 0x80000) { y1 = 0 - (0x100000 - y1); }
    if (z1 & 0x80000) { z1 = 0 - (0x100000 - z1); }

    peak1 = {static_cast<float>(-y1) / 1000.f, static_cast<float>(x1) / 1000.f, static_cast<float>(z1) / 1000.f};
    peak1 *= 0.25f;

    if (peak1 == Vector3f::Zero())
    {
        peak1 = Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
}

static inline int get_offset_full_res(int point_nbr)
{
    return 15 * (24 * line_order[point_nbr / 24] + point_nbr % 24);
}

static inline int get_offset_half_res(int point_nbr)
{
    return 15 * (24 * line_order[point_nbr / 24] / 2 + point_nbr % 24);
}

dt1_device_cdc::dt1_device_cdc(const std::string& serial_number, const std::function<void(connect_status)>& connect_cb,
                               point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb) :
    dt1_device(serial_number, connect_cb, std::move(point_cloud_cb), std::move(imu_cb), std::move(log_cb))
{
}

dt1_device_cdc::~dt1_device_cdc()
{
    stop();
}

void dt1_device_cdc::start_impl()
{
    int point_count_x = dt1_modes_list[_mode].nbr_points_x;
    int point_count_y = dt1_modes_list[_mode].nbr_points_y;
    _point_cloud.resize(point_count_x * point_count_y);
    _receive_buf.resize(point_count_x * point_count_y * 15);
}

void dt1_device_cdc::read_serial_point_cloud()
{
    // Read point cloud
    read_line_until("BEGIN MP");

    size_t n = 0;
    while (n < _receive_buf.size())
    {
        n += _serial->read(_receive_buf.data() + n, _receive_buf.size() - n);
        LOG_DEBUG("Read %ld bytes", n);

        if (!_running) return;
    }
}

void dt1_device_cdc::process_serial_data()
{
    LOG_DEBUG("Read frame from camera %s", _device_info.hw_id.c_str());

    int point_count_x = dt1_modes_list[_mode].nbr_points_x;
    int point_count_y = dt1_modes_list[_mode].nbr_points_y;

    uint64_t frame_ts = read_timestamp() * 1'000'000LL;
    if (!_running) return;

    read_serial_point_cloud();
    read_imu();

    int64_t end_ts = read_timestamp() * 1'000'000LL;
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t receive_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
    _clock_offset = receive_ts - end_ts;

    // Unpack point cloud
    int (*offset)(int) = (point_count_y == 24 ? get_offset_full_res : get_offset_half_res);
    for (int i = 0; i < point_count_x * point_count_y; i++)
    {
        extract_first_peak(_receive_buf, offset(i), _point_cloud[i]);
    }

    // Unpack IMU data
    unpack_imu_data(frame_ts);

    if (_running && _point_cloud_cb) { _point_cloud_cb(frame_ts + _clock_offset, std::cref(_point_cloud)); }
    if (_running && _imu_cb && !_imu_samples.empty()) { _imu_cb(_imu_samples); }
}
