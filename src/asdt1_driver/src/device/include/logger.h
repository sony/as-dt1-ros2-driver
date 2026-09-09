/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "asdt1_types.h"

#include <cstdio>
#include <string>

#define LOG_IMPL(level, msg, ...)                                              \
    do {                                                                       \
        if (_log_cb) {                                                         \
            char _buf[512];                                                    \
            std::snprintf(_buf, sizeof(_buf), msg __VA_OPT__(,) __VA_ARGS__); \
            _log_cb(level, std::string(_buf));                                 \
        }                                                                      \
    } while (0)

#define LOG_DEBUG(msg, ...) LOG_IMPL(log_level::debug, msg __VA_OPT__(,) __VA_ARGS__)
#define LOG_INFO(msg, ...)  LOG_IMPL(log_level::info,  msg __VA_OPT__(,) __VA_ARGS__)
#define LOG_WARN(msg, ...)  LOG_IMPL(log_level::warn,  msg __VA_OPT__(,) __VA_ARGS__)
#define LOG_ERROR(msg, ...) LOG_IMPL(log_level::error, msg __VA_OPT__(,) __VA_ARGS__)
