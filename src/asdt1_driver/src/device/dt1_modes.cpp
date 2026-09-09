/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dt1_modes.h"

#include <array>
#include <string>
#include <algorithm>

const std::array<mode_info, 5> dt1_modes_list = {
    mode_info{"20m", 24, 24, 800, 504, 24, 32},    mode_info{"30mstd", 24, 24, 800, 252, 12, 32},
    mode_info{"30m15f", 24, 24, 800, 252, 12, 65}, mode_info{"30m30f", 24, 12, 800, 126, 6, 32},
    mode_info{"40m", 24, 24, 800, 252, 12, 65},
};

static bool equals_ignore_case(std::string a, std::string b)
{
    std::transform(a.begin(), a.end(), a.begin(), [](unsigned char c) { return std::tolower(c); });
    std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c) { return std::tolower(c); });
    return a == b;
}

int get_mode_nbr(const std::string& mode_str)
{
    for (size_t i = 0; i < dt1_modes_list.size(); i++)
    {
        if (equals_ignore_case(dt1_modes_list[i].mode_str, mode_str)) { return (int)i; }
    }
    return -1;
}

std::string get_available_modes_str()
{
    std::string modes_str;
    for (const auto& mode : dt1_modes_list)
    {
        modes_str += mode.mode_str + " ";
    }
    return modes_str;
}