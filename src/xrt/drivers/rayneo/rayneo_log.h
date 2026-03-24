// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief RayNeo protocol and device logging helpers.
 * @author Claude Opus 4.6
 * @ingroup drv_rayneo
 */

#pragma once

#include "rayneo_proto.h"

#include "util/u_logging.h"

struct xrt_fov;

#define RAYNEO_FW_LOG_LINE_MAX 256

typedef struct rayneo_fw_log_buf
{
	char line[RAYNEO_FW_LOG_LINE_MAX];
	size_t len;
} rayneo_fw_log_buf_t;

void
rayneo_log_device_info(enum u_logging_level log_level, const rayneo_device_info_t *info);

void
rayneo_log_device_state(enum u_logging_level log_level, const rayneo_device_state_t *state);

void
rayneo_log_imu_calibration(enum u_logging_level log_level, const rayneo_imu_calibration_t *cal);

void
rayneo_log_panel_fov(enum u_logging_level log_level, const rayneo_panel_fov_t *fov);

void
rayneo_log_fov(enum u_logging_level log_level, const struct xrt_fov *fov, uint32_t w_pixels, uint32_t h_pixels);

void
rayneo_log_fw_log(enum u_logging_level log_level, rayneo_fw_log_buf_t *buf, const rayneo_log_stream_t *log_stream);

void
rayneo_log_fw_trace(enum u_logging_level log_level, const rayneo_trace_report_t *trace);
