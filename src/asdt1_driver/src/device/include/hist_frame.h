/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <cstdint>

#pragma pack(push, 1)

struct peak_data
{
    uint8_t radial[4];
    uint8_t x[4];
    uint8_t y[4];
    uint8_t z[4];
    uint8_t height[4];
    uint32_t err;
};

struct mp_data
{
    uint8_t reserved1[4];
    uint32_t bin[144];
    uint32_t reserved2[4];
    uint8_t noise_level[4];
    peak_data first;
    peak_data second;
    peak_data third;
};

struct ebd_data
{
   uint8_t reserved1[138];
   uint8_t curr_bank;
   uint8_t reserved2[5];
   uint8_t curr_ws;
   uint8_t reserved3[79];
   uint8_t sensor_temp;
   uint8_t reserved4[7];
   uint8_t sensor_ts[8];
   uint8_t reserved5[410];
   uint8_t pattern[4];
   uint8_t cpu_ts[4];
   uint8_t input_ts[4];
   uint8_t reserved6[10];
};

struct bank_data
{
    ebd_data ebd;
    uint8_t reserved[672];
    mp_data mp[48];
};

struct hist_frame_data
{
    bank_data bank[24]; // 6, 12 or 24 banks are used depending on mode
};

#pragma pack(pop)
static_assert(sizeof(ebd_data) == 672, "EBD data must be 672 bytes");
static_assert(sizeof(hist_frame_data) == 672 * 50 * 24, "Histogram frame data must be 806400 bytes");
