// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Online magnetometer calibration using NXP/Freescale algorithm.
 *
 * Progressive 4/7/10 element calibration with accelerometer-based
 * spatial binning. Wraps the NXP Open-Source Sensor Fusion magnetic.c.
 *
 * @author Claude Opus 4.6
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "os/os_threading.h"

#include "magnetic.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct m_mag_calib_nxp_save_buf
{
	float fV[3];
	float finvW[3][3];
	float fB;
};

struct m_mag_calib_nxp
{
	struct MagCalibration cal;
	struct MagneticBuffer buf;
	struct MagSensor mag_sensor;
	struct AccelSensor accel_sensor;
	int32 loop_counter;
	int cal_level;

	// Persistence.
	struct os_thread_helper save_thread;
	struct m_mag_calib_nxp_save_buf save_buf;
	bool save_pending;
	float last_saved_error;
	char save_filename[256];
};

/*!
 * Initialize the NXP magnetometer calibration.
 *
 * @param cal             Calibration state.
 * @param ut_per_count    Conversion factor from raw counts to microtesla.
 *                        Use 1.0 if mag data is already in microtesla.
 */
void
m_mag_calib_nxp_init(struct m_mag_calib_nxp *cal, float ut_per_count);

/*!
 * Reset the calibration state.
 */
void
m_mag_calib_nxp_reset(struct m_mag_calib_nxp *cal);

/*!
 * Feed a raw magnetometer and accelerometer sample.
 *
 * The accelerometer is used for spatial binning of samples (determines
 * which orientation bin the measurement belongs to).
 *
 * @param cal    Calibration state.
 * @param mag    Raw magnetometer reading (any consistent units).
 * @param accel  Accelerometer reading (any consistent units, only direction matters).
 */
void
m_mag_calib_nxp_update(struct m_mag_calib_nxp *cal, const struct xrt_vec3 *mag, const struct xrt_vec3 *accel);

/*!
 * Apply the current calibration to a raw measurement.
 *
 * @param cal         Calibration state.
 * @param raw         Raw magnetometer reading.
 * @param[out] out    Calibrated magnetometer reading.
 */
void
m_mag_calib_nxp_apply(struct m_mag_calib_nxp *cal, const struct xrt_vec3 *raw, struct xrt_vec3 *out);

/*!
 * Return the current calibration level (0, 4, 7, or 10).
 */
int
m_mag_calib_nxp_level(const struct m_mag_calib_nxp *cal);

/*!
 * Return the current fit error percentage.
 */
float
m_mag_calib_nxp_fit_error(const struct m_mag_calib_nxp *cal);

/*!
 * Return the number of samples in the buffer.
 */
int
m_mag_calib_nxp_sample_count(const struct m_mag_calib_nxp *cal);

/*!
 * Load saved calibration state from a file in the Monado config directory.
 *
 * @param cal       Calibration state (must be initialized first).
 * @param filename  Filename within the Monado config dir (e.g. "rayneo_mag.json").
 * @return true if state was loaded successfully.
 */
bool
m_mag_calib_nxp_load(struct m_mag_calib_nxp *cal, const char *filename);

/*!
 * Start the background save thread.
 *
 * When the sensor thread accepts an improved calibration at level 10,
 * it copies the state and signals this thread to write it to disk.
 *
 * @param cal       Calibration state.
 * @param filename  Filename within the Monado config dir.
 */
void
m_mag_calib_nxp_save_start(struct m_mag_calib_nxp *cal, const char *filename);

/*!
 * Stop the background save thread. Call before destroying the calibration state.
 */
void
m_mag_calib_nxp_save_stop(struct m_mag_calib_nxp *cal);

#ifdef __cplusplus
}
#endif
