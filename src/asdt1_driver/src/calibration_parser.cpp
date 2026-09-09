/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "calibration_parser.h"

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <stdexcept>

static rclcpp::Logger logger() { return rclcpp::get_logger("calibration_parser"); }

std::unique_ptr<calibration_parser> calibration_parser::_instance = nullptr;
std::mutex calibration_parser::_mutex;

void calibration_parser::init(const std::string& file)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_instance) { _instance = std::unique_ptr<calibration_parser>(new calibration_parser(file)); }
}

calibration_parser& calibration_parser::get()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_instance) { throw std::runtime_error("calibration_parser not initialized. Call init() first."); }
    return *_instance;
}

calibration_parser::calibration_parser(const std::string& filepath)
{
    _filename = filepath;
    load(filepath);
}

const std::vector<camera_calibration>& calibration_parser::get_cameras() const
{ return _cameras; }

void calibration_parser::load(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file)
    {
        RCLCPP_INFO(logger(), "Did not find %s", filepath.c_str());
        return;
    }

    try
    {
        _yaml = YAML::LoadFile(filepath);
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(logger(), "Failed to parse calibration file %s: %s", filepath.c_str(), e.what());
        return;
    }

    _cameras.clear();

    const YAML::Node cameras = _yaml["cameras"];
    if (!cameras || !cameras.IsSequence())
    {
        RCLCPP_WARN(logger(), "Calibration file %s does not contain a valid 'cameras' list", filepath.c_str());
        return;
    }

    for (const auto& cam : cameras)
    {
        try
        {
            if (!cam["id"]) { throw std::runtime_error("missing required field 'id'"); }
            if (!cam["extrinsics"]) { throw std::runtime_error("missing required field 'extrinsics'"); }
            if (!cam["extrinsics"]["rotation_matrix"])
            {
                throw std::runtime_error("missing required field 'extrinsics.rotation_matrix'");
            }
            if (!cam["extrinsics"]["translation_vector"])
            {
                throw std::runtime_error("missing required field 'extrinsics.translation_vector'");
            }

            camera_calibration calib;
            calib.hw_id = cam["id"].as<std::string>();
            if (cam["description"]) { calib.description = cam["description"].as<std::string>(); }
            // Accept frame IDs either at camera level or nested under extrinsics.
            if (cam["frame_id"]) { calib.frame_id = cam["frame_id"].as<std::string>(); }
            if (cam["parent_frame_id"]) { calib.parent_frame_id = cam["parent_frame_id"].as<std::string>(); }
            if (cam["extrinsics"]["frame_id"]) { calib.frame_id = cam["extrinsics"]["frame_id"].as<std::string>(); }
            if (cam["extrinsics"]["parent_frame_id"])
            {
                calib.parent_frame_id = cam["extrinsics"]["parent_frame_id"].as<std::string>();
            }

            const auto& R = cam["extrinsics"]["rotation_matrix"];
            if (!R.IsSequence() || R.size() != 3) { throw std::runtime_error("rotation_matrix must be 3x3"); }
            for (int row = 0; row < 3; ++row)
            {
                if (!R[row].IsSequence() || R[row].size() != 3)
                {
                    throw std::runtime_error("rotation_matrix must be 3x3");
                }
                for (int col = 0; col < 3; ++col)
                {
                    calib.extrinsics.get_rotation()(row, col) = R[row][col].as<float>();
                }
            }

            const auto& T = cam["extrinsics"]["translation_vector"];
            if (!T.IsSequence() || T.size() != 3) { throw std::runtime_error("translation_vector must have 3 values"); }
            for (int i = 0; i < 3; ++i)
            {
                calib.extrinsics.get_translation()(i) = T[i].as<float>();
            }

            _cameras.emplace_back(std::move(calib));
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(logger(), "Skipping invalid calibration entry: %s", e.what());
        }
    }

    RCLCPP_INFO(logger(), "Calibration: Loaded %zu cameras from %s", _cameras.size(), filepath.c_str());
    for (const auto& c : _cameras)
    {
        RCLCPP_INFO(logger(), "Found calibration for id: %s", c.hw_id.c_str());
    }
}

bool calibration_parser::get_calibration(const std::string& hardware_id, camera_calibration& calibration)
{
    for (const auto& c : _cameras)
    {
        if (c.hw_id == hardware_id)
        {
            calibration = c;
            return true;
        }
    }
    return false;
}

std::string calibration_parser::get_description(const std::string& hardware_id)
{
    for (const auto& c : _cameras)
    {
        if (c.hw_id == hardware_id) { return c.description; }
    }
    return "";
}
