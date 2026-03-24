// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief RayNeo protocol decode and command helpers.
 * @author ChatGPT 5.3
 * @ingroup drv_rayneo
 */

#pragma once

#include "rayneo_usb.h"

/*!
 * Known board_id values reported in device info packets.
 *
 * This protocol-level codename list is intentionally incomplete and may grow
 * as new firmware/device variants are observed.
 *
 * Add a marketing-name alias only when the codename mapping is confirmed.
 */
typedef enum rayneo_board_id
{
	RAYNEO_BOARD_NEXTVIEW_PRO = 0x20,
	RAYNEO_BOARD_ARIES = 0x21,
	RAYNEO_BOARD_ARIES_1_5_SEEYA = 0x22,
	RAYNEO_BOARD_ARIES_1_5_SONY = 0x23,
	RAYNEO_BOARD_ARIES_1P8 = 0x24,

	RAYNEO_BOARD_TAURUS = 0x30,
	RAYNEO_BOARD_TAURUS_1_5 = 0x31,
	RAYNEO_BOARD_TAURUS_2_0 = 0x35,
	RAYNEO_BOARD_TAURUS_3_0_LOW = 0x36,
	RAYNEO_BOARD_TAURUS_3_0_PRO = 0x37,
	RAYNEO_BOARD_TAURUS_2_0_OVERSEAS = 0x38,
	RAYNEO_BOARD_TAURUS_4_0 = 0x39,
	RAYNEO_BOARD_TAURUS_4_0_PRO = 0x3A,

	/* Confirmed marketing aliases. */
	RAYNEO_BOARD_AIR_4 = RAYNEO_BOARD_TAURUS_4_0,
	RAYNEO_BOARD_AIR_4_PRO = RAYNEO_BOARD_TAURUS_4_0_PRO,
} rayneo_board_id_t;

/*!
 * RayNeo display mode.
 */
typedef enum rayneo_mode
{
	RAYNEO_MODE_2D = 0,
	RAYNEO_MODE_3D = 1,
} rayneo_mode_t;

/*!
 * One decoded IMU sensor sample from the device.
 */
typedef struct rayneo_sensor_sample
{
	float accel_mps2[3];
	float gyro_dps[3];
	float temperature_c;
	float magnet[3];
	float psensor;
	float lsensor;
	uint32_t device_tick_100us;
	uint64_t device_time_ns;
} rayneo_sensor_sample_t;

/*!
 * IMU calibration payload returned by the firmware.
 */
typedef struct rayneo_imu_calibration
{
	bool valid;
	float tsb[3][3];
	float ta[3];
} rayneo_imu_calibration_t;

/*!
 * Device information block reported by firmware.
 *
 * Luminance values in this response are hardware-encoded panel preset values,
 * not linear brightness values. Parser also fills derived brightness fields.
 */
typedef struct rayneo_device_info
{
	uint32_t tick;
	uint8_t value;
	uint8_t cpuid[12];
	uint8_t board_id;
	uint8_t sensor_on;
	uint8_t support_fov;
	char date[12];
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t glasses_fps;
	uint8_t luminance;
	// Derived from luminance via known mapping table.
	int32_t brightness;
	uint8_t volume;
	uint8_t side_by_side;
	// Firmware uses inverted semantics here: raw 0 means enabled.
	uint8_t psensor_enable;
	uint8_t audio_mode;
	uint8_t dp_status;
	uint8_t status3;
	uint8_t psensor_valid;
	uint8_t lsensor_valid;
	uint8_t gyro_valid;
	uint8_t magnet_valid;
	float reserve1;
	float reserve2;
	uint8_t max_luminance;
	// Derived from max_luminance as max logical brightness index.
	int32_t max_brightness;
	uint8_t max_volume;
	uint8_t support_panel_color_adjust;
	uint8_t flag;
} rayneo_device_info_t;

/*!
 * Panel field-of-view values in degrees.
 */
typedef struct rayneo_panel_fov
{
	float left_deg;
	float right_deg;
	float top_deg;
	float bottom_deg;
} rayneo_panel_fov_t;

/*!
 * Runtime device state block (brightness, volume, mode, etc).
 *
 * These fields mirror the corresponding values in @ref rayneo_device_info_t,
 * but this response is smaller.
 *
 * Semantics:
 * - max_luminance is the brightness level count (same as device_info
 *   max_luminance).
 * - luminance is the current logical brightness level.
 */
typedef struct rayneo_device_state
{
	uint8_t max_luminance;
	uint8_t luminance;
	uint8_t max_volume;
	uint8_t volume;
	uint8_t audio_mode;
	uint8_t dp_status;
	uint8_t glasses_fps;
	uint8_t side_by_side;
} rayneo_device_state_t;

/*!
 * Feature capability bitmap and decoded flags.
 */
typedef struct rayneo_func_support
{
	uint8_t cap[55];
	size_t cap_len;
	// Decoded capability flags from cap[] (currently known mapping on Air 4 Pro).
	uint8_t support_fov_get;
	uint8_t support_side_by_side;
	uint8_t support_fps_120;
	uint8_t support_luminance_change;
	uint8_t support_accumaster;
	uint8_t support_accumaster_mode_change;
	uint8_t support_accumaster_mode_eye;
	uint8_t support_audio_volume_change;
	uint8_t support_audio_quiet_mode;
	uint8_t support_audio_disable;
	uint8_t support_gsensor_raw;
	uint8_t support_gsensor_angle_correction;
	uint8_t support_gyro_temp_bias;
	uint8_t support_time_sync_msg;
	uint8_t support_get_orbit;
	uint8_t support_user_orbit;
	uint8_t support_panel_distance_adjust;
	uint8_t support_panel_high_dynamic;
	uint8_t support_audio_spatial_mode;
	uint8_t support_audio_tube_mode;
	uint8_t support_panel_hdr;
	uint8_t support_panel_color_enhance;
} rayneo_func_support_t;

/*!
 * Device/host time synchronization sample.
 *
 * Current observed firmware behavior emits indexes 1..9.
 */
typedef struct rayneo_time_sync
{
	uint32_t device_tick_100us;
	uint64_t device_time_ns;
	uint8_t index;
	uint32_t receive_tick_100us;
	uint64_t receive_time_ns;
} rayneo_time_sync_t;

/*
 * Expected cmd 0xE1 index range for current firmware behavior.
 */
#define RAYNEO_TIME_SYNC_EXPECTED_FIRST_INDEX 1
#define RAYNEO_TIME_SYNC_EXPECTED_LAST_INDEX 9
#define RAYNEO_TIME_SYNC_EXPECTED_SAMPLES_PER_REQUEST 9

#define RAYNEO_LOG_STREAM_MAX_BYTES 56

/*!
 * Firmware log-stream packet.
 */
typedef struct rayneo_log_stream
{
	uint32_t device_tick_100us;
	uint8_t payload[RAYNEO_LOG_STREAM_MAX_BYTES];
	size_t payload_len;
} rayneo_log_stream_t;

#define RAYNEO_TRACE_REPORT_MAX_BYTES 50

/*!
 * Firmware trace-report packet.
 */
typedef struct rayneo_trace_report
{
	uint8_t raw[64];
	size_t raw_len;
	uint32_t device_tick_100us;
	uint8_t report_cmd;
	uint8_t chunk_index;
	uint16_t chunk_offset;
	uint16_t chunk_len;
	uint8_t payload[RAYNEO_TRACE_REPORT_MAX_BYTES];
	size_t payload_len;
} rayneo_trace_report_t;

#define RAYNEO_GYRO_BIAS_MAX_POINTS_PER_PACKET 4
#define RAYNEO_BRIGHTNESS_LEVEL_COUNT 29

/*!
 * One decoded gyro-bias packet chunk.
 */
typedef struct rayneo_gyro_bias
{
	uint8_t chunk_index;
	uint8_t chunk_count;
	int8_t temp_start_c;
	uint8_t point_count;
	float bias_dps[RAYNEO_GYRO_BIAS_MAX_POINTS_PER_PACKET][3];
} rayneo_gyro_bias_t;

/*!
 * Cmd 0x09 brightness response payload.
 *
 * This packet is sent when:
 * - user changes brightness with glasses hardware buttons.
 * - device handles luminance up/down commands (cmd 0x55/0x56).
 *
 * This packet is not sent as a direct response to luminance_set/brightness_set
 * requests (cmd 0x09).
 */
typedef struct rayneo_brightness_response
{
	uint8_t brightness_level;
} rayneo_brightness_response_t;

/*!
 * Cmd 0x51/0x52 volume response payload.
 */
typedef struct rayneo_volume_response
{
	uint8_t volume_level;
} rayneo_volume_response_t;

/*!
 * Decoded packet classification returned by @ref rayneo_proto_read_next.
 */
typedef enum rayneo_packet_type
{
	RAYNEO_PACKET_NONE = 0,
	RAYNEO_PACKET_SENSOR,
	RAYNEO_PACKET_DEVICE_INFO,
	RAYNEO_PACKET_IMU_CALIBRATION,
	RAYNEO_PACKET_PANEL_FOV,
	RAYNEO_PACKET_FUNC_SUPPORT,
	RAYNEO_PACKET_TIME_SYNC,
	RAYNEO_PACKET_LOG_STREAM,
	RAYNEO_PACKET_TRACE_REPORT,
	RAYNEO_PACKET_DEVICE_STATE,
	RAYNEO_PACKET_GYRO_BIAS,
	RAYNEO_PACKET_ACK_OPEN_IMU,
	RAYNEO_PACKET_ACK_CLOSE_IMU,
	RAYNEO_PACKET_ACK_3D_MODE,
	RAYNEO_PACKET_ACK_2D_MODE,
	RAYNEO_PACKET_BRIGHTNESS_SET,
	RAYNEO_PACKET_VOLUME_UP,
	RAYNEO_PACKET_VOLUME_DOWN,
	RAYNEO_PACKET_ACK_IMU_BYPASS_SW_CORRECTION,
	RAYNEO_PACKET_ACK_LOG_ENABLE,
	RAYNEO_PACKET_RESPONSE,
	RAYNEO_PACKET_UNKNOWN,
} rayneo_packet_type_t;

/*!
 * One decoded RayNeo protocol packet.
 */
typedef struct rayneo_packet
{
	rayneo_packet_type_t type;
	union {
		rayneo_sensor_sample_t sensor;
		rayneo_device_info_t device_info;
		rayneo_imu_calibration_t imu_calibration;
		rayneo_panel_fov_t panel_fov;
		rayneo_func_support_t func_support;
		rayneo_time_sync_t time_sync;
		rayneo_log_stream_t log_stream;
		rayneo_trace_report_t trace_report;
		rayneo_device_state_t device_state;
		rayneo_gyro_bias_t gyro_bias;
		rayneo_brightness_response_t brightness_response;
		rayneo_volume_response_t volume_response;
		struct
		{
			uint8_t cmd;
			uint8_t raw[64];
			size_t raw_len;
		} response;
	};
} rayneo_packet_t;

/*!
 * Read and decode one protocol packet from the device.
 *
 * @param dev Opened USB device state.
 * @param out Output packet.
 * @param timeout_ms Read timeout in milliseconds.
 *
 * @return 1 when a packet was decoded, 0 on timeout, -1 on error.
 */
int
rayneo_proto_read_next(rayneo_device_t *dev, rayneo_packet_t *out, int timeout_ms);

/*!
 * Request device information.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_device_info(rayneo_device_t *dev);
/*!
 * Request capability flags.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_func_support(rayneo_device_t *dev);
/*!
 * Request time synchronization samples.
 *
 * Current expected firmware behavior is
 * @ref RAYNEO_TIME_SYNC_EXPECTED_SAMPLES_PER_REQUEST packets per request,
 * with indexes @ref RAYNEO_TIME_SYNC_EXPECTED_FIRST_INDEX through
 * @ref RAYNEO_TIME_SYNC_EXPECTED_LAST_INDEX.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_time_sync(rayneo_device_t *dev);
/*!
 * Request current runtime device state.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_device_state(rayneo_device_t *dev);
/*!
 * Request panel field-of-view values.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_panel_fov(rayneo_device_t *dev);
/*!
 * Open IMU streaming.
 *
 * @param bypass_sw_correction Whether firmware should bypass software correction.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_open_imu(rayneo_device_t *dev, bool bypass_sw_correction);
/*!
 * Close IMU streaming.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_close_imu(rayneo_device_t *dev);
/*!
 * Request IMU calibration data.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_imu_calibration(rayneo_device_t *dev);
/*!
 * Request gyro bias points for a temperature range.
 *
 * @param temp_start_c Start temperature in celsius.
 * @param temp_end_c End temperature in celsius.
 *
 * @return 0 on success, -1 for invalid range, or libusb error code on failure.
 */
int
rayneo_proto_request_gyro_bias_range(rayneo_device_t *dev, int temp_start_c, int temp_end_c);
/*!
 * Request one luminance step up.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_luminance_up(rayneo_device_t *dev);
/*!
 * Request one luminance step down.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_luminance_down(rayneo_device_t *dev);
/*!
 * Request absolute panel luminance level.
 *
 * Cmd 0x09 argument is hardware-encoded luminance.
 *
 * Cmd 0x09 requests have no direct response packet on current firmware.
 * Brightness updates are reported by cmd 0x09 packets triggered by cmd
 * 0x55/0x56 and by hardware button presses.
 *
 * @param hardware_level Target hardware luminance level used as raw command arg.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_luminance_set(rayneo_device_t *dev, uint8_t hardware_level);
/*!
 * Request panel brightness by logical level.
 *
 * The brightness level is mapped to hardware luminance encoding used by
 * @ref rayneo_proto_request_luminance_set.
 *
 * @param brightness_level Target logical brightness level in range
 * [0, @ref RAYNEO_BRIGHTNESS_LEVEL_COUNT - 1].
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_brightness_set(rayneo_device_t *dev, uint8_t brightness_level);
/*!
 * Request one volume step up.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_volume_up(rayneo_device_t *dev);
/*!
 * Request one volume step down.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_volume_down(rayneo_device_t *dev);
/*!
 * Request display mode switch.
 *
 * @param mode Target mode.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_set_mode(rayneo_device_t *dev, rayneo_mode_t mode);
/*!
 * Request panel power on.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_panel_power_on(rayneo_device_t *dev);
/*!
 * Request panel power off.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_panel_power_off(rayneo_device_t *dev);
/*!
 * Disable firmware-side IMU correction.
 *
 * RayNeo Air 4 Pro note: this disables gyroscope temperature and idle
 * correction.
 *
 * @param bypass True to disable firmware-side IMU correction.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_imu_bypass_sw_correction(rayneo_device_t *dev, bool bypass);
/*!
 * Enable or disable firmware log stream.
 *
 * @param enable True to enable logs.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_proto_request_log_enable(rayneo_device_t *dev, bool enable);
