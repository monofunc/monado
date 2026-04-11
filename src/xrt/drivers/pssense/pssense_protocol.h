// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlayStation Sense controller prober and driver code.
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_byte_order.h"

#include "math/m_api.h"


const uint8_t INPUT_REPORT_ID = 0x31;
const uint8_t OUTPUT_REPORT_ID = 0x31;
const uint8_t OUTPUT_REPORT_TAG = 0x10;
const uint8_t CALIBRATION_DATA_FEATURE_REPORT_ID = 0x05;
const uint8_t CALIBRATION_DATA_PART_ID_1 = 0;
const uint8_t CALIBRATION_DATA_PART_ID_2 = 0x81;

const uint8_t INPUT_REPORT_CRC32_SEED = 0xa1;
const uint8_t OUTPUT_REPORT_CRC32_SEED = 0xa2;
const uint8_t FEATURE_REPORT_CRC32_SEED = 0xa3;

//! Gyro read value range is +-32768.
const double PSSENSE_GYRO_SCALE_DEG = 180.0 / 1024;
//! Accelerometer read value range is +-32768 and covers +-8 g.
const double PSSENSE_ACCEL_SCALE = MATH_GRAVITY_M_S2 / 4096;

//! Flag bits to enable setting vibration in an output report
const uint8_t VIBRATE_ENABLE_BITS = 0x03;
//! Pure 120Hz vibration
const uint8_t VIBRATE_MODE_HIGH_120HZ = 0x00;
//! Pure 60Hz vibration
const uint8_t VIBRATE_MODE_LOW_60HZ = 0x20;
//! Emulates a legacy vibration motor
const uint8_t VIBRATE_MODE_CLASSIC_RUMBLE = 0x40;
//! Softer rumble emulation, like an engine running
const uint8_t VIBRATE_MODE_DIET_RUMBLE = 0x60;

//! Flag bits to enable setting trigger feedback in an output report
const uint8_t TRIGGER_FEEDBACK_ENABLE_BITS = 0x04;
//! Clear the trigger feedback setting
const uint8_t TRIGGER_FEEDBACK_MODE_NONE = 0x00;
//! Constant resistance throughout the trigger movement
const uint8_t TRIGGER_FEEDBACK_MODE_CONSTANT = 0x01;
//! A single point of resistance at the beginning of the trigger, right before the click flag is activated
const uint8_t TRIGGER_FEEDBACK_MODE_CATCH = 0x02;

const uint8_t CHARGE_STATE_DISCHARGING = 0x00;
const uint8_t CHARGE_STATE_CHARGING = 0x01;
const uint8_t CHARGE_STATE_FULL = 0x02;
const uint8_t CHARGE_STATE_ABNORMAL_VOLTAGE = 0x0A;
const uint8_t CHARGE_STATE_ABNORMAL_TEMP = 0x0B;
const uint8_t CHARGE_STATE_CHARGING_ERROR = 0x0F;

/**
 * 16-bit little-endian int
 */
struct pssense_i16_le
{
	uint8_t low;
	uint8_t high;
};

/**
 * 32-bit little-endian int
 */
struct pssense_i32_le
{
	uint8_t lowest;
	uint8_t lower;
	uint8_t higher;
	uint8_t highest;
};

#define INPUT_REPORT_LENGTH 78
/*!
 * HID input report data packet.
 */
struct pssense_input_report
{
	uint8_t report_id;
	uint8_t bt_header;
	uint8_t thumbstick_x;
	uint8_t thumbstick_y;
	uint8_t trigger_value;
	uint8_t trigger_proximity;
	uint8_t squeeze_proximity;
	uint8_t unknown1[2]; // Always 0x0001
	uint8_t buttons[3];
	uint8_t unknown2; // Always 0x00
	struct pssense_i32_le seq_no;
	struct pssense_i16_le gyro[3];
	struct pssense_i16_le accel[3];
	struct pssense_i32_le imu_ticks;
	uint8_t temperature;
	uint8_t unknown3[9];
	uint8_t battery_state; // High bits charge level 0x00-0x0a, low bits battery state
	uint8_t plug_state;    // Flags for USB data and/or power connected
	struct pssense_i32_le host_timestamp;
	struct pssense_i32_le device_timestamp;
	uint8_t unknown4[4];
	uint8_t aes_cmac[8];
	uint8_t unknown5;
	uint8_t crc_failure_count;
	uint8_t padding[7];
	struct pssense_i32_le crc;
};
static_assert(sizeof(struct pssense_input_report) == INPUT_REPORT_LENGTH, "Incorrect input report struct length");

#define OUTPUT_REPORT_LENGTH 78
/**
 * HID output report data packet.
 */
struct pssense_output_report
{
	uint8_t report_id;
	uint8_t bt_seq_no;      // High bits only; low bits are always 0
	uint8_t tag;            // Needs to be 0x10 for this report
	uint8_t feedback_flags; // Vibrate mode and enable flags to set vibrate and trigger feedback in this report
	uint8_t unknown;
	uint8_t vibration_amplitude; // Vibration amplitude from 0x00-0xff. Sending 0 turns vibration off.
	uint8_t unknown2;
	uint8_t trigger_feedback_mode; // Constant or sticky trigger resistance
	uint8_t ffb[10];
	struct pssense_i32_le host_timestamp;
	uint8_t unknown3[19];
	uint8_t counter;
	uint8_t haptics[32];
	struct pssense_i32_le crc;
};
static_assert(sizeof(struct pssense_output_report) == OUTPUT_REPORT_LENGTH, "Incorrect output report struct length");

#define FEATURE_REPORT_LENGTH 64
#define CALIBRATION_DATA_LENGTH 116

/**
 * HID output report data packet.
 */
struct pssense_feature_report
{
	uint8_t report_id;
	uint8_t part_id;
	uint8_t data[CALIBRATION_DATA_LENGTH / 2];
	struct pssense_i32_le crc;
};
static_assert(sizeof(struct pssense_feature_report) == FEATURE_REPORT_LENGTH, "Incorrect feature report struct length");

static uint32_t
pssense_i32_le_to_u32(const struct pssense_i32_le *from)
{
	return (uint32_t)(from->lowest | from->lower << 8 | from->higher << 16 | from->highest << 24);
}

static struct pssense_i32_le
pssense_u32_to_i32_le(uint32_t from)
{
	struct pssense_i32_le ret = {
	    .lowest = (from >> 0) & 0x0ff,
	    .lower = (from >> 8) & 0x0ff,
	    .higher = (from >> 16) & 0x0ff,
	    .highest = (from >> 24) & 0x0ff,
	};

	return ret;
}

static int16_t
pssense_i16_le_to_i16(const struct pssense_i16_le *from)
{
	// The cast is important, sign extend properly.
	return (int16_t)(from->low | from->high << 8);
}
