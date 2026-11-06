/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct camera_extrinsics
{
    // Row-major 3x3 rotation matrix
    std::array<std::array<float, 3>, 3> rotation = {{{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}}};
    // Translation in millimetres
    std::array<float, 3> translation = {0.f, 0.f, 0.f};
};

struct camera_calibration
{
    std::string hw_id;
    std::string description;
    std::string frame_id;
    std::string parent_frame_id;
    camera_extrinsics extrinsics;
};

class calibration_parser
{
public:
    static calibration_parser& get();
    static void init(const std::string& file);

    /// Get list of calibrated cameras
    const std::vector<camera_calibration>& get_cameras() const;

    std::string get_calibration_filename() { return _filename; }
    bool have_calibration() const { return !_cameras.empty(); }
    int get_nbr_cameras() const { return _cameras.size(); }
    bool get_calibration(const std::string& hardware_id, camera_calibration& calibration);
    std::string get_description(const std::string& hardware_id);

    void update_json(const std::vector<camera_calibration>& calibration);
    void update_yaml(const std::vector<camera_calibration>& calibration);

    // Non-copyable
    calibration_parser(const calibration_parser&) = delete;
    calibration_parser& operator=(const calibration_parser&) = delete;

private:
    static std::unique_ptr<calibration_parser> _instance;
    static std::mutex _mutex;
    YAML::Node _yaml;
    std::string _filename;
    std::vector<camera_calibration> _cameras;

    calibration_parser(const std::string& filepath);
    void load(const std::string& filepath);
    void create_calibration_file(const std::vector<camera_calibration>& calibration);
};
