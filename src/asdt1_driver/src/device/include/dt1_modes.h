/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <string>
#include <array>

enum
{
    cdc_mode_20m,
    cdc_mode_30mstd,
    cdc_mode_30m15f,
    cdc_mode_30m30f,
    cdc_mode_40m,
};

struct mode_info
{
    std::string mode_str;
    int nbr_points_x;
    int nbr_points_y;
    int hist_frame_width;
    int hist_frame_height;
    int nbr_banks;
    float tot_exposure;
};

extern const std::array<mode_info, 5> dt1_modes_list;

int get_mode_nbr(const std::string& mode_str);
std::string get_available_modes_str();
