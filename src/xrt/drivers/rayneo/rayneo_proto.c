// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief RayNeo protocol decode and command helpers.
 * @author ChatGPT 5.3
 * @ingroup drv_rayneo
 */

#include "rayneo_proto.h"

#include <string.h>

#define RAYNEO_TICK_TO_NS 100000

enum rayneo_wire_type
{
	RAYNEO_WIRE_RESPONSE = 0xC8,
	RAYNEO_WIRE_LOG_STREAM = 0xC9,
	RAYNEO_WIRE_TRACE_REPORT = 0xCA,
	RAYNEO_WIRE_SENSOR = 0x65,
};

enum rayneo_command
{
	RAYNEO_CMD_DEVICE_INFO = 0x00,
	RAYNEO_CMD_IMU_OPEN = 0x01,
	RAYNEO_CMD_IMU_CLOSE = 0x02,
	RAYNEO_CMD_MODE_3D = 0x06,
	RAYNEO_CMD_MODE_2D = 0x07,
	RAYNEO_CMD_PANEL_PRESET_SET = 0x09,
	RAYNEO_CMD_PANEL_POWER_ON = 0x0E,
	RAYNEO_CMD_PANEL_POWER_OFF = 0x0F,
	RAYNEO_CMD_PANEL_FOV = 0x23,
	RAYNEO_CMD_IMU_CALIBRATION = 0x3C,
	RAYNEO_CMD_IMU_BYPASS_SW_CORRECTION = 0x3D,
	RAYNEO_CMD_GYRO_BIAS_GET = 0x3E,
	RAYNEO_CMD_VOLUME_UP = 0x51,
	RAYNEO_CMD_VOLUME_DOWN = 0x52,
	RAYNEO_CMD_LUMINANCE_UP = 0x55,
	RAYNEO_CMD_LUMINANCE_DOWN = 0x56,
	RAYNEO_CMD_LOG_ENABLE = 0x70,
	RAYNEO_CMD_FUNC_SUPPORT = 0xE0,
	RAYNEO_CMD_TIME_SYNC = 0xE1,
	RAYNEO_CMD_DEVICE_STATE = 0xE3,
};

enum rayneo_packet_layout
{
	RAYNEO_PKT_MAGIC = 0x99,
	RAYNEO_PKT_MIN_HEADER = 4,
	RAYNEO_PKT_TYPE_OFFSET = 1,
	RAYNEO_PKT_SIZE_OFFSET = 2,
	RAYNEO_PKT_RESPONSE_CMD_OFFSET = 8,
	RAYNEO_PKT_PAYLOAD_OFFSET = 9,
	RAYNEO_PKT_DEVICE_INFO_SIZE = 64,
	RAYNEO_PKT_LOG_STREAM_SIZE = 64,
	RAYNEO_PKT_TRACE_REPORT_SIZE = 64,
	RAYNEO_PKT_TIME_SYNC_MIN_SIZE = 16,
	RAYNEO_PKT_DEVICE_STATE_MIN_SIZE = 19,
};

/*
 * Brightness level -> cmd 0x09 hardware luminance arg.
 * Matches the app/firmware mapping.
 */
static const uint8_t rayneo_brightness_to_hw_luminance[RAYNEO_BRIGHTNESS_LEVEL_COUNT] = {
    1, 2, 0, 5, 6, 7, 3, 8, 9, 10, 11, 12, 4, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
};

static uint8_t
rayneo_encode_brightness_level(uint8_t brightness_level)
{
	if (brightness_level >= RAYNEO_BRIGHTNESS_LEVEL_COUNT) {
		return 0xFF;
	}

	return rayneo_brightness_to_hw_luminance[brightness_level];
}

static int32_t
rayneo_proto_decode_brightness_level(uint8_t hardware_level)
{
	for (size_t i = 0; i < RAYNEO_BRIGHTNESS_LEVEL_COUNT; i++) {
		if (rayneo_brightness_to_hw_luminance[i] == hardware_level) {
			return (int32_t)i;
		}
	}

	return -1;
}

static int32_t
rayneo_brightness_count_to_max(uint8_t brightness_count)
{
	if (brightness_count == 0) {
		return 0;
	}

	size_t count = (size_t)brightness_count;
	if (count > RAYNEO_BRIGHTNESS_LEVEL_COUNT) {
		count = RAYNEO_BRIGHTNESS_LEVEL_COUNT;
	}

	return (int32_t)count - 1;
}

static uint16_t
read_u16_le(const uint8_t *ptr)
{
	return (uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8);
}

static uint32_t
read_u32_le(const uint8_t *ptr)
{
	return (uint32_t)ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16) | ((uint32_t)ptr[3] << 24);
}

static float
read_f32_le(const uint8_t *ptr)
{
	uint32_t bits = read_u32_le(ptr);
	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

static void
copy_limited(uint8_t *dst, size_t dst_len, const uint8_t *src, size_t src_len, size_t *written)
{
	size_t n = src_len < dst_len ? src_len : dst_len;
	if (n > 0) {
		memcpy(dst, src, n);
	}
	if (written != NULL) {
		*written = n;
	}
}

static void
parse_sensor(const uint8_t *raw, rayneo_sensor_sample_t *out)
{
	out->accel_mps2[0] = read_f32_le(&raw[4]);  // accel x
	out->accel_mps2[1] = read_f32_le(&raw[8]);  // accel y
	out->accel_mps2[2] = read_f32_le(&raw[12]); // accel z

	out->gyro_dps[0] = read_f32_le(&raw[16]); // gyro x
	out->gyro_dps[1] = read_f32_le(&raw[20]); // gyro y
	out->gyro_dps[2] = read_f32_le(&raw[24]); // gyro z

	out->temperature_c = read_f32_le(&raw[28]);     // temperature
	out->magnet[0] = read_f32_le(&raw[32]);         // magnet x
	out->magnet[1] = read_f32_le(&raw[36]);         // magnet y
	out->device_tick_100us = read_u32_le(&raw[40]); // device tick (100us)
	out->psensor = read_f32_le(&raw[44]);           // proximity
	out->lsensor = read_f32_le(&raw[48]);           // light
	out->magnet[2] = read_f32_le(&raw[52]);         // magnet z (after psensor/lsensor on wire)
	out->device_time_ns = (uint64_t)out->device_tick_100us * RAYNEO_TICK_TO_NS;
}

static void
parse_device_info(const uint8_t *raw, rayneo_device_info_t *out)
{
	memset(out, 0, sizeof(*out));
	out->tick = read_u32_le(&raw[4]);
	out->value = raw[8];
	memcpy(out->cpuid, &raw[9], 12);
	out->board_id = raw[21];
	out->sensor_on = raw[22];
	out->support_fov = raw[23];
	memcpy(out->date, &raw[24], 12);
	out->year = read_u16_le(&raw[36]);
	out->month = raw[38];
	out->day = raw[39];
	out->glasses_fps = raw[40];
	out->luminance = raw[41];
	out->volume = raw[42];
	out->side_by_side = raw[43];
	out->psensor_enable = raw[44] == 0 ? 1 : 0;
	out->audio_mode = raw[45];
	out->dp_status = raw[46];
	out->status3 = raw[47];
	out->psensor_valid = raw[48];
	out->lsensor_valid = raw[49];
	out->gyro_valid = raw[50];
	out->magnet_valid = raw[51];
	out->reserve1 = read_f32_le(&raw[52]);
	out->reserve2 = read_f32_le(&raw[56]);
	out->max_luminance = raw[60];
	out->max_volume = raw[61];
	out->support_panel_color_adjust = raw[62];
	out->flag = raw[63];

	out->max_brightness = rayneo_brightness_count_to_max(out->max_luminance);
	int32_t brightness = rayneo_proto_decode_brightness_level(out->luminance);
	out->brightness = brightness >= 0 ? brightness : 0;
}

static void
parse_imu_calibration(const uint8_t *raw, rayneo_imu_calibration_t *out)
{
	memset(out, 0, sizeof(*out));
	out->valid = raw[9] != 0xFF;

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			int flat = row * 3 + col;
			out->tsb[row][col] = read_f32_le(&raw[9 + (size_t)flat * 4]); // 3x3 matrix
		}
	}
	for (int i = 0; i < 3; i++) {
		out->ta[i] = read_f32_le(&raw[9 + (size_t)(9 + i) * 4]); // accel bias
	}
}

static void
parse_panel_fov(const uint8_t *raw, size_t raw_len, rayneo_panel_fov_t *out)
{
	memset(out, 0, sizeof(*out));

	if (raw_len >= RAYNEO_PKT_PAYLOAD_OFFSET + 8) {
		out->left_deg = (float)read_u16_le(&raw[RAYNEO_PKT_PAYLOAD_OFFSET + 0]) / 10.0f;
		out->right_deg = (float)read_u16_le(&raw[RAYNEO_PKT_PAYLOAD_OFFSET + 2]) / 10.0f;
		out->top_deg = (float)read_u16_le(&raw[RAYNEO_PKT_PAYLOAD_OFFSET + 4]) / 10.0f;
		out->bottom_deg = (float)read_u16_le(&raw[RAYNEO_PKT_PAYLOAD_OFFSET + 6]) / 10.0f;
	}
}

static inline uint8_t
func_support_cap_at(const rayneo_func_support_t *func_support, size_t index)
{
	return index < func_support->cap_len ? func_support->cap[index] : 0;
}

static void
parse_func_support(const uint8_t *raw, size_t raw_len, rayneo_func_support_t *out)
{
	memset(out, 0, sizeof(*out));

	if (raw_len > RAYNEO_PKT_PAYLOAD_OFFSET) {
		size_t cap_len = raw_len - RAYNEO_PKT_PAYLOAD_OFFSET;
		if (cap_len > sizeof(out->cap)) {
			cap_len = sizeof(out->cap);
		}
		memcpy(out->cap, &raw[RAYNEO_PKT_PAYLOAD_OFFSET], cap_len);
		out->cap_len = cap_len;
	}

	out->support_fov_get = func_support_cap_at(out, 0);
	out->support_side_by_side = func_support_cap_at(out, 1);
	out->support_fps_120 = func_support_cap_at(out, 2);
	out->support_luminance_change = func_support_cap_at(out, 3);
	out->support_accumaster = func_support_cap_at(out, 4);
	out->support_accumaster_mode_change = func_support_cap_at(out, 5);
	out->support_accumaster_mode_eye = func_support_cap_at(out, 6);
	out->support_audio_volume_change = func_support_cap_at(out, 7);
	out->support_audio_quiet_mode = func_support_cap_at(out, 8);
	out->support_audio_disable = func_support_cap_at(out, 9);
	out->support_gsensor_raw = func_support_cap_at(out, 10);
	out->support_gsensor_angle_correction = func_support_cap_at(out, 11);
	out->support_gyro_temp_bias = func_support_cap_at(out, 12);
	out->support_time_sync_msg = func_support_cap_at(out, 13);
	out->support_get_orbit = func_support_cap_at(out, 14);
	out->support_user_orbit = func_support_cap_at(out, 15);
	out->support_panel_distance_adjust = func_support_cap_at(out, 16);
	out->support_panel_high_dynamic = func_support_cap_at(out, 17);
	out->support_audio_spatial_mode = func_support_cap_at(out, 18);
	out->support_audio_tube_mode = func_support_cap_at(out, 19);
	out->support_panel_hdr = func_support_cap_at(out, 20);
	out->support_panel_color_enhance = func_support_cap_at(out, 21);
}

static void
parse_time_sync(const uint8_t *raw, rayneo_time_sync_t *out)
{
	memset(out, 0, sizeof(*out));
	out->device_tick_100us = read_u32_le(&raw[4]);
	out->device_time_ns = (uint64_t)out->device_tick_100us * RAYNEO_TICK_TO_NS;
	out->index = raw[9];
	out->receive_tick_100us = read_u32_le(&raw[12]);
	out->receive_time_ns = (uint64_t)out->receive_tick_100us * RAYNEO_TICK_TO_NS;
}

static void
parse_log_stream(const uint8_t *raw, rayneo_log_stream_t *out)
{
	memset(out, 0, sizeof(*out));
	out->device_tick_100us = read_u32_le(&raw[4]);
	memcpy(out->payload, &raw[8], RAYNEO_LOG_STREAM_MAX_BYTES);
	out->payload_len = RAYNEO_LOG_STREAM_MAX_BYTES;
}

static void
parse_trace_report(const uint8_t *raw, rayneo_trace_report_t *out)
{
	memset(out, 0, sizeof(*out));
	memcpy(out->raw, raw, sizeof(out->raw));
	out->raw_len = sizeof(out->raw);
	out->device_tick_100us = read_u32_le(&raw[4]);
	out->report_cmd = raw[8];
	out->chunk_index = raw[9];
	out->chunk_offset = read_u16_le(&raw[10]);
	out->chunk_len = read_u16_le(&raw[12]);
	memcpy(out->payload, &raw[14], RAYNEO_TRACE_REPORT_MAX_BYTES);
	out->payload_len = RAYNEO_TRACE_REPORT_MAX_BYTES;
}

static void
parse_device_state(const uint8_t *raw, rayneo_device_state_t *out)
{
	memset(out, 0, sizeof(*out));
	out->max_luminance = raw[9];
	out->luminance = raw[10];
	out->max_volume = raw[11];
	out->volume = raw[12];
	out->audio_mode = raw[14];
	out->dp_status = raw[16];
	out->glasses_fps = raw[17];
	out->side_by_side = raw[18];
}

static void
parse_gyro_bias(const uint8_t *raw, size_t raw_len, rayneo_gyro_bias_t *out)
{
	memset(out, 0, sizeof(*out));
	if (raw_len < 13) {
		return;
	}

	out->chunk_index = raw[9];
	out->chunk_count = raw[10];
	out->temp_start_c = (int8_t)raw[11];
	out->point_count = raw[12];

	size_t available_points = (raw_len - 13) / 12;
	size_t point_count = out->point_count;
	if (point_count > available_points) {
		point_count = available_points;
	}
	if (point_count > RAYNEO_GYRO_BIAS_MAX_POINTS_PER_PACKET) {
		point_count = RAYNEO_GYRO_BIAS_MAX_POINTS_PER_PACKET;
	}
	out->point_count = (uint8_t)point_count;

	for (size_t i = 0; i < point_count; i++) {
		size_t base = 13 + i * 12;
		out->bias_dps[i][0] = read_f32_le(&raw[base + 0]);
		out->bias_dps[i][1] = read_f32_le(&raw[base + 4]);
		out->bias_dps[i][2] = read_f32_le(&raw[base + 8]);
	}
}

static void
parse_brightness_response(const uint8_t *raw, rayneo_brightness_response_t *out)
{
	out->brightness_level = raw[9];
}

static void
parse_volume_response(const uint8_t *raw, rayneo_volume_response_t *out)
{
	out->volume_level = raw[9];
}

static void
parse_response_fallback(const uint8_t *raw, size_t raw_len, uint8_t cmd, rayneo_packet_t *out)
{
	out->type = RAYNEO_PACKET_RESPONSE;
	out->response.cmd = cmd;
	copy_limited(out->response.raw, sizeof(out->response.raw), raw, raw_len, &out->response.raw_len);
}

static void
parse_one_packet(const uint8_t *raw, size_t raw_len, rayneo_packet_t *out)
{
	if (raw_len < RAYNEO_PKT_MIN_HEADER) {
		out->type = RAYNEO_PACKET_UNKNOWN;
		return;
	}

	uint8_t wire_type = raw[RAYNEO_PKT_TYPE_OFFSET];
	if (wire_type == RAYNEO_WIRE_SENSOR) {
		if (raw_len < 56) { // must include float at byte 52 (magnet z)
			out->type = RAYNEO_PACKET_UNKNOWN;
			return;
		}
		out->type = RAYNEO_PACKET_SENSOR;
		parse_sensor(raw, &out->sensor);
		return;
	}

	if (wire_type == RAYNEO_WIRE_LOG_STREAM) {
		if (raw_len < RAYNEO_PKT_LOG_STREAM_SIZE) {
			out->type = RAYNEO_PACKET_UNKNOWN;
			return;
		}
		out->type = RAYNEO_PACKET_LOG_STREAM;
		parse_log_stream(raw, &out->log_stream);
		return;
	}

	if (wire_type == RAYNEO_WIRE_TRACE_REPORT) {
		if (raw_len < RAYNEO_PKT_TRACE_REPORT_SIZE) {
			out->type = RAYNEO_PACKET_UNKNOWN;
			return;
		}
		out->type = RAYNEO_PACKET_TRACE_REPORT;
		parse_trace_report(raw, &out->trace_report);
		return;
	}

	if (wire_type != RAYNEO_WIRE_RESPONSE || raw_len <= RAYNEO_PKT_RESPONSE_CMD_OFFSET) {
		out->type = RAYNEO_PACKET_UNKNOWN;
		return;
	}

	uint8_t cmd = raw[RAYNEO_PKT_RESPONSE_CMD_OFFSET];
	switch (cmd) {
	case RAYNEO_CMD_DEVICE_INFO:
		if (raw_len >= RAYNEO_PKT_DEVICE_INFO_SIZE) {
			out->type = RAYNEO_PACKET_DEVICE_INFO;
			parse_device_info(raw, &out->device_info);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_IMU_CALIBRATION:
		if (raw_len >= 57) {
			out->type = RAYNEO_PACKET_IMU_CALIBRATION;
			parse_imu_calibration(raw, &out->imu_calibration);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_PANEL_FOV:
		out->type = RAYNEO_PACKET_PANEL_FOV;
		parse_panel_fov(raw, raw_len, &out->panel_fov);
		return;
	case RAYNEO_CMD_FUNC_SUPPORT:
		out->type = RAYNEO_PACKET_FUNC_SUPPORT;
		parse_func_support(raw, raw_len, &out->func_support);
		return;
	case RAYNEO_CMD_TIME_SYNC:
		if (raw_len >= RAYNEO_PKT_TIME_SYNC_MIN_SIZE) {
			out->type = RAYNEO_PACKET_TIME_SYNC;
			parse_time_sync(raw, &out->time_sync);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_DEVICE_STATE:
		if (raw_len >= RAYNEO_PKT_DEVICE_STATE_MIN_SIZE) {
			out->type = RAYNEO_PACKET_DEVICE_STATE;
			parse_device_state(raw, &out->device_state);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_GYRO_BIAS_GET:
		if (raw_len >= 13) {
			out->type = RAYNEO_PACKET_GYRO_BIAS;
			parse_gyro_bias(raw, raw_len, &out->gyro_bias);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_PANEL_PRESET_SET:
		if (raw_len > 9) {
			out->type = RAYNEO_PACKET_BRIGHTNESS_SET;
			parse_brightness_response(raw, &out->brightness_response);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_VOLUME_UP:
		if (raw_len > 9) {
			out->type = RAYNEO_PACKET_VOLUME_UP;
			parse_volume_response(raw, &out->volume_response);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_VOLUME_DOWN:
		if (raw_len > 9) {
			out->type = RAYNEO_PACKET_VOLUME_DOWN;
			parse_volume_response(raw, &out->volume_response);
		} else {
			parse_response_fallback(raw, raw_len, cmd, out);
		}
		return;
	case RAYNEO_CMD_IMU_OPEN: out->type = RAYNEO_PACKET_ACK_OPEN_IMU; return;
	case RAYNEO_CMD_IMU_CLOSE: out->type = RAYNEO_PACKET_ACK_CLOSE_IMU; return;
	case RAYNEO_CMD_MODE_3D: out->type = RAYNEO_PACKET_ACK_3D_MODE; return;
	case RAYNEO_CMD_MODE_2D: out->type = RAYNEO_PACKET_ACK_2D_MODE; return;
	case RAYNEO_CMD_IMU_BYPASS_SW_CORRECTION: out->type = RAYNEO_PACKET_ACK_IMU_BYPASS_SW_CORRECTION; return;
	case RAYNEO_CMD_LOG_ENABLE: out->type = RAYNEO_PACKET_ACK_LOG_ENABLE; return;
	default: parse_response_fallback(raw, raw_len, cmd, out); return;
	}
}

int
rayneo_proto_read_next(rayneo_device_t *dev, rayneo_packet_t *out, int timeout_ms)
{
	if (dev == NULL || dev->handle == NULL || out == NULL) {
		return -1;
	}

	memset(out, 0, sizeof(*out));
	out->type = RAYNEO_PACKET_NONE;

	uint8_t buf[256];
	size_t chunk = dev->in_mps > 0 ? (size_t)dev->in_mps : 64u;
	if (chunk > sizeof(buf)) {
		chunk = sizeof(buf);
	}

	int n = rayneo_usb_read_packet(dev, buf, chunk, timeout_ms > 0 ? timeout_ms : 1);
	if (n < 0) {
		return -1;
	}
	if (n == 0) {
		return 0;
	}

	parse_one_packet(buf, (size_t)n, out);
	return 1;
}

static int
rayneo_proto_send_command(rayneo_device_t *dev, uint8_t cmd, uint8_t arg, const void *payload, size_t payload_len)
{
	return rayneo_usb_send_request_with_payload(dev, cmd, arg, payload, payload_len);
}

int
rayneo_proto_request_device_info(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_DEVICE_INFO, 0, NULL, 0);
}

int
rayneo_proto_request_func_support(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_FUNC_SUPPORT, 0, NULL, 0);
}

int
rayneo_proto_request_time_sync(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_TIME_SYNC, 0, NULL, 0);
}

int
rayneo_proto_request_device_state(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_DEVICE_STATE, 0, NULL, 0);
}

int
rayneo_proto_request_panel_fov(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_PANEL_FOV, 0, NULL, 0);
}

int
rayneo_proto_request_open_imu(rayneo_device_t *dev, bool bypass_sw_correction)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_IMU_OPEN, bypass_sw_correction, NULL, 0);
}

int
rayneo_proto_request_close_imu(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_IMU_CLOSE, 0, NULL, 0);
}

int
rayneo_proto_request_imu_calibration(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_IMU_CALIBRATION, 0, NULL, 0);
}

int
rayneo_proto_request_gyro_bias_range(rayneo_device_t *dev, int temp_start_c, int temp_end_c)
{
	if (temp_end_c < temp_start_c) {
		return -1;
	}

	int span = temp_end_c - temp_start_c + 1;
	if (span <= 0 || span > 255) {
		return -1;
	}

	uint8_t payload[1] = {(uint8_t)span};
	return rayneo_proto_send_command(dev, RAYNEO_CMD_GYRO_BIAS_GET, (uint8_t)temp_start_c, payload,
	                                 sizeof(payload));
}

int
rayneo_proto_request_luminance_up(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_LUMINANCE_UP, 0, NULL, 0);
}

int
rayneo_proto_request_luminance_down(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_LUMINANCE_DOWN, 0, NULL, 0);
}

int
rayneo_proto_request_luminance_set(rayneo_device_t *dev, uint8_t hardware_level)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_PANEL_PRESET_SET, hardware_level, NULL, 0);
}

int
rayneo_proto_request_brightness_set(rayneo_device_t *dev, uint8_t brightness_level)
{
	uint8_t hw_level = rayneo_encode_brightness_level(brightness_level);
	return rayneo_proto_request_luminance_set(dev, hw_level);
}

int
rayneo_proto_request_volume_up(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_VOLUME_UP, 0, NULL, 0);
}

int
rayneo_proto_request_volume_down(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_VOLUME_DOWN, 0, NULL, 0);
}

int
rayneo_proto_request_set_mode(rayneo_device_t *dev, rayneo_mode_t mode)
{
	return rayneo_proto_send_command(dev, mode == RAYNEO_MODE_3D ? RAYNEO_CMD_MODE_3D : RAYNEO_CMD_MODE_2D, 0, NULL,
	                                 0);
}

int
rayneo_proto_request_panel_power_on(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_PANEL_POWER_ON, 0, NULL, 0);
}

int
rayneo_proto_request_panel_power_off(rayneo_device_t *dev)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_PANEL_POWER_OFF, 0, NULL, 0);
}

int
rayneo_proto_request_imu_bypass_sw_correction(rayneo_device_t *dev, bool bypass)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_IMU_BYPASS_SW_CORRECTION, bypass, NULL, 0);
}

int
rayneo_proto_request_log_enable(rayneo_device_t *dev, bool enable)
{
	return rayneo_proto_send_command(dev, RAYNEO_CMD_LOG_ENABLE, enable, NULL, 0);
}
