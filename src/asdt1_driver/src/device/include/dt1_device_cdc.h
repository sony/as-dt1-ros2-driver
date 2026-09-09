/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "dt1_device.h"

class dt1_device_cdc : public dt1_device
{
public:
    explicit dt1_device_cdc(const std::string& serial_number, const std::function<void(connect_status)>& connect_cb,
                            point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb);
    ~dt1_device_cdc() override;

protected:
    void start_impl() override;
    void process_serial_data() override;
    const char* expected_flstart_mode() const override { return "cdc"; }

private:
    void read_serial_point_cloud();

    std::vector<Eigen::Vector3f> _point_cloud;
    std::vector<uint8_t> _receive_buf;
};
