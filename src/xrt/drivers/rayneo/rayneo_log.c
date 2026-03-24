// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief RayNeo protocol and device logging helpers.
 * @author Claude Opus 4.6
 * @ingroup drv_rayneo
 */

#include "rayneo_log.h"

#include "math/m_api.h"

static void
to_printable_ascii(const uint8_t *src, size_t src_len, char *dst, size_t dst_len)
{
	if (dst_len == 0) {
		return;
	}

	size_t out_len = 0;
	size_t max = src_len < (dst_len - 1) ? src_len : (dst_len - 1);
	for (size_t i = 0; i < max; i++) {
		uint8_t c = src[i];
		if (c == 0) {
			break;
		}
		if (c >= 32 && c <= 126) {
			dst[out_len++] = (char)c;
		} else {
			dst[out_len++] = '.';
		}
	}
	dst[out_len] = '\0';
}

void
rayneo_log_device_info(enum u_logging_level log_level, const rayneo_device_info_t *info)
{
	U_LOG_IFL_D(log_level, "Device info: board=%u date=%04u-%02u-%02u fps=%u sbs=%u sensor_on=%u", info->board_id,
	            info->year, info->month, info->day, info->glasses_fps, info->side_by_side, info->sensor_on);
}

void
rayneo_log_device_state(enum u_logging_level log_level, const rayneo_device_state_t *state)
{
	U_LOG_IFL_D(log_level,
	            "Device state: brightness=%u count=%u vol=%u/%u fps=%u sbs=%u audio_mode=%u dp_status=%u",
	            state->luminance, state->max_luminance, state->volume, state->max_volume, state->glasses_fps,
	            state->side_by_side, state->audio_mode, state->dp_status);
}

void
rayneo_log_imu_calibration(enum u_logging_level log_level, const rayneo_imu_calibration_t *cal)
{
	U_LOG_IFL_D(log_level, "IMU calibration valid=%d", cal->valid);
	if (!cal->valid) {
		return;
	}
	U_LOG_IFL_D(log_level, "  tsb=[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]", cal->tsb[0][0], cal->tsb[0][1],
	            cal->tsb[0][2], cal->tsb[1][0], cal->tsb[1][1], cal->tsb[1][2], cal->tsb[2][0], cal->tsb[2][1],
	            cal->tsb[2][2]);
	U_LOG_IFL_D(log_level, "  ta=[%.4f %.4f %.4f]", cal->ta[0], cal->ta[1], cal->ta[2]);
}

void
rayneo_log_panel_fov(enum u_logging_level log_level, const rayneo_panel_fov_t *fov)
{
	U_LOG_IFL_D(log_level, "Panel FOV: L=%.1f R=%.1f T=%.1f B=%.1f deg", fov->left_deg, fov->right_deg,
	            fov->top_deg, fov->bottom_deg);
}

void
rayneo_log_fov(enum u_logging_level log_level, const struct xrt_fov *fov, uint32_t w_pixels, uint32_t h_pixels)
{
	float hfov = RAD_TO_DEG(fov->angle_right - fov->angle_left);
	float vfov = RAD_TO_DEG(fov->angle_up - fov->angle_down);
	float ppd_h = (hfov > 0.001f) ? ((float)w_pixels / hfov) : 0.0f;
	float ppd_v = (vfov > 0.001f) ? ((float)h_pixels / vfov) : 0.0f;
	U_LOG_IFL_D(log_level, "FOV: L=%0.6f R=%0.6f U=%0.6f D=%0.6f (rad)", fov->angle_left, fov->angle_right,
	            fov->angle_up, fov->angle_down);
	U_LOG_IFL_D(log_level, "PPD: horiz=%0.6f vert=%0.6f (px/deg)", ppd_h, ppd_v);
}

static void
fw_log_flush(enum u_logging_level log_level, rayneo_fw_log_buf_t *buf)
{
	if (buf->len == 0) {
		return;
	}
	buf->line[buf->len] = '\0';
	U_LOG_IFL_D(log_level, "FW: %s", buf->line);
	buf->len = 0;
}

void
rayneo_log_fw_log(enum u_logging_level log_level, rayneo_fw_log_buf_t *buf, const rayneo_log_stream_t *log_stream)
{
	for (size_t i = 0; i < log_stream->payload_len; i++) {
		uint8_t c = log_stream->payload[i];
		if (c == '\n') {
			fw_log_flush(log_level, buf);
			continue;
		}
		if (c == '\r' || c == '\0') {
			continue;
		}
		buf->line[buf->len++] = (char)c;
		if (buf->len >= RAYNEO_FW_LOG_LINE_MAX - 1) {
			fw_log_flush(log_level, buf);
		}
	}
}

void
rayneo_log_fw_trace(enum u_logging_level log_level, const rayneo_trace_report_t *trace)
{
	char text[128] = {0};
	to_printable_ascii(trace->payload, trace->payload_len, text, sizeof(text));
	U_LOG_IFL_D(log_level, "FW trace: tick=%u cmd=0x%02x idx=%u off=%u chunk=%u text=\"%s\"",
	            trace->device_tick_100us, trace->report_cmd, trace->chunk_index, trace->chunk_offset,
	            trace->chunk_len, text);
	U_LOG_IFL_HEX(U_LOGGING_TRACE, log_level, trace->raw, trace->raw_len);
}
