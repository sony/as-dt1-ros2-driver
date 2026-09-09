/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "dt1_device.h"
#include "connect_status.h"
#include "hist_frame.h"

#include <Eigen/Core>
#include <linux/media.h>
#include <vector>
#include <optional>

class dt1_device_hist : public dt1_device
{
public:
    explicit dt1_device_hist(const std::string& serial_number, const std::function<void(connect_status)>& connect_cb,
                             point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb);
    ~dt1_device_hist() override;

protected:
    void process_serial_data() override;
    const char* expected_flstart_mode() const override { return "hist"; }
    void start_impl() override;
    void stop_impl() override;

private:
    struct mapped_buffer
    {
        void* start = nullptr;
        size_t length = 0;
    };
    struct video_node_info
    {
        std::string devnode; // e.g. /dev/video4
        std::string entity_name;
        bool is_video_capture = false;
        bool is_metadata_capture = false;
    };

    struct media_match_result
    {
        std::string media_dev;               // e.g. /dev/media0
        media_device_info mdi{};             // serial, model, bus_info, ...
        std::vector<video_node_info> videos; // all associated /dev/videoX nodes
    };

    void run_v4l2();
    bool v4l2_stream_on();
    void v4l2_stream_off();
    void unpack_hist_frame(const hist_frame_data* hist_frame);

    int open_video_device_by_serial(const std::string& serial);
    std::optional<media_match_result> find_media_by_serial(const std::string& wanted_serial);

    std::thread _v4l2_thread;
    int _fd = -1;
    std::vector<mapped_buffer> _buffers;

    std::vector<Vector6f> _points_xyzitr;
};
