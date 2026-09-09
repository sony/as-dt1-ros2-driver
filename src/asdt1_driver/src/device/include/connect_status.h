/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <string>

struct device_info
{
    std::string device_version;
    std::string hw_id;
    int mode = 0;
};

class connect_status
{
public:
    connect_status(bool connected, const device_info& dev_info ) : _connected(connected), _device_info(dev_info) {};

    bool is_connected() const { return _connected; }
    const device_info& get_device_info() const { return _device_info; }
private:
    bool _connected = false;
    const device_info _device_info;
};
