/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "asdt1_types.h"
#include "connect_status.h"

#include "serial/serial.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class dt1_device
{
public:
    static std::unique_ptr<dt1_device> create(const std::string& point_cloud_type, const std::string& serial_number,
                                              const std::function<void(connect_status)>& connect_cb,
                                              point_cloud_callback point_cloud_cb, imu_callback imu_cb,
                                              log_callback log_cb);

    explicit dt1_device(const std::string& serial_number, const std::function<void(connect_status)>& connect_cb,
                        point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb);
    virtual ~dt1_device();

    void start();
    virtual void stop();

    bool set_mode(int mode);
    int get_mode() const { return _mode; }

    void set_frame_rate(int frame_rate_hz);
    int get_frame_rate() const;

    void set_sync_mode(sync_mode sync_mode);
    sync_mode get_sync_mode() const;

    void reboot();
    const device_info& get_info() const { return _device_info; }

protected:
    virtual void process_serial_data() = 0;
    virtual const char* expected_flstart_mode() const = 0;
    virtual void start_impl() {}
    virtual void stop_impl() {}

    void run();
    void stop_if_streaming();

    bool connect(const std::string& port);
    bool send_and_receive_ack(const std::string& cmd) const;
    void send_and_receive(const std::string& cmd) const;

    std::string read_line() const;
    std::string read_line_until(const std::string& find_str) const;
    bool read_to_prompt() const;
    bool verify_device_mode();

    device_info get_device_info();
    device_info _device_info;

    void read_imu();
    uint64_t read_timestamp();

    void unpack_imu_data(uint64_t frame_ts);

    int _mode = 0;

    std::thread _serial_thread;
    std::atomic<bool> _running = false;
    std::atomic<int> _frame_rate_hz = 30;
    std::string _serial_number;
    sync_mode _sync_mode = sync_mode::none;

    std::vector<imu_sample> _imu_samples;
    std::vector<std::array<uint8_t, 16>> _imu_data;

    const std::function<void(connect_status)> _connect_cb;
    const point_cloud_callback _point_cloud_cb;
    const imu_callback _imu_cb;
    const log_callback _log_cb;

    std::unique_ptr<serial::Serial> _serial = nullptr;

    int64_t _latest_imu_sample_ts = -1;
    std::atomic<uint64_t> _clock_offset = 0; // current offset between local and asdt1 clock
};

class unsupported_device_error : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};
