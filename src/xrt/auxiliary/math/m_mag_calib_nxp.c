// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Online magnetometer calibration using NXP/Freescale algorithm.
 * @author Claude Opus 4.6
 * @ingroup aux_math
 */

#include "m_mag_calib_nxp.h"

#include "util/u_file.h"
#include "util/u_json.h"
#include "util/u_logging.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void
maybe_trigger_save(struct m_mag_calib_nxp *cal);

void
m_mag_calib_nxp_init(struct m_mag_calib_nxp *cal, float ut_per_count)
{
	memset(cal, 0, sizeof(*cal));
	fInitMagCalibration(&cal->cal, &cal->buf);

	cal->mag_sensor.fuTPerCount = ut_per_count;
	if (ut_per_count > 0.0f) {
		cal->mag_sensor.fCountsPeruT = 1.0f / ut_per_count;
	} else {
		cal->mag_sensor.fCountsPeruT = 1.0f;
	}
}

void
m_mag_calib_nxp_reset(struct m_mag_calib_nxp *cal)
{
	float ut_per_count = cal->mag_sensor.fuTPerCount;
	m_mag_calib_nxp_init(cal, ut_per_count);
}

void
m_mag_calib_nxp_update(struct m_mag_calib_nxp *cal, const struct xrt_vec3 *mag, const struct xrt_vec3 *accel)
{
	// Convert to integer counts for the NXP buffer.
	// When fuTPerCount=1.0, counts == microtesla values.
	float counts_per_ut = cal->mag_sensor.fCountsPeruT;
	cal->mag_sensor.iBpFast[0] = (int16)(mag->x * counts_per_ut);
	cal->mag_sensor.iBpFast[1] = (int16)(mag->y * counts_per_ut);
	cal->mag_sensor.iBpFast[2] = (int16)(mag->z * counts_per_ut);

	cal->mag_sensor.fBp[0] = mag->x;
	cal->mag_sensor.fBp[1] = mag->y;
	cal->mag_sensor.fBp[2] = mag->z;
	cal->mag_sensor.fBpFast[0] = mag->x;
	cal->mag_sensor.fBpFast[1] = mag->y;
	cal->mag_sensor.fBpFast[2] = mag->z;

	// Accel as integer counts (scale to ~1000 counts per g for good binning resolution).
	cal->accel_sensor.iGp[0] = (int16)(accel->x * 1000.0f);
	cal->accel_sensor.iGp[1] = (int16)(accel->y * 1000.0f);
	cal->accel_sensor.iGp[2] = (int16)(accel->z * 1000.0f);

	cal->loop_counter++;

	// Update the magnetometer buffer with spatial binning.
	iUpdateMagnetometerBuffer(&cal->buf, &cal->accel_sensor, &cal->mag_sensor, cal->loop_counter);

	// Run progressive calibration. Never downgrade below the current level.
	int count = cal->buf.iMagBufferCount;
	int cur_level = cal->cal.iValidMagCal;
	if (count >= MINMEASUREMENTS10CAL && cal->loop_counter >= INTERVAL10CAL) {
		fUpdateCalibration10EIG(&cal->cal, &cal->buf, &cal->mag_sensor);
		cal->cal_level = 10;
	} else if (cur_level < 10 && count >= MINMEASUREMENTS7CAL && cal->loop_counter >= INTERVAL7CAL) {
		fUpdateCalibration7EIG(&cal->cal, &cal->buf, &cal->mag_sensor);
		cal->cal_level = 7;
	} else if (cur_level < 7 && count >= MINMEASUREMENTS4CAL && cal->loop_counter >= INTERVAL4CAL) {
		fUpdateCalibration4INV(&cal->cal, &cal->buf, &cal->mag_sensor);
		cal->cal_level = 4;
	} else {
		return;
	}

	// Accept the trial calibration if fit error is reasonable.
	if (cal->cal.ftrFitErrorpc < cal->cal.fFitErrorpc || cal->cal.ftrFitErrorpc < 20.0f) {
		for (int i = 0; i < 3; i++) {
			cal->cal.fV[i] = cal->cal.ftrV[i];
			for (int j = 0; j < 3; j++) {
				cal->cal.finvW[i][j] = cal->cal.ftrinvW[i][j];
			}
		}
		cal->cal.fB = cal->cal.ftrB;
		cal->cal.fFitErrorpc = cal->cal.ftrFitErrorpc;
		cal->cal.iValidMagCal = cal->cal_level;
		maybe_trigger_save(cal);
	}
}

void
m_mag_calib_nxp_apply(struct m_mag_calib_nxp *cal, const struct xrt_vec3 *raw, struct xrt_vec3 *out)
{
	if (cal->cal.iValidMagCal == 0) {
		*out = *raw;
		return;
	}

	// Apply: calibrated = invW * (raw - V)
	float tmp[3];
	tmp[0] = raw->x - cal->cal.fV[0];
	tmp[1] = raw->y - cal->cal.fV[1];
	tmp[2] = raw->z - cal->cal.fV[2];

	out->x = cal->cal.finvW[0][0] * tmp[0] + cal->cal.finvW[0][1] * tmp[1] + cal->cal.finvW[0][2] * tmp[2];
	out->y = cal->cal.finvW[1][0] * tmp[0] + cal->cal.finvW[1][1] * tmp[1] + cal->cal.finvW[1][2] * tmp[2];
	out->z = cal->cal.finvW[2][0] * tmp[0] + cal->cal.finvW[2][1] * tmp[1] + cal->cal.finvW[2][2] * tmp[2];
}

int
m_mag_calib_nxp_level(const struct m_mag_calib_nxp *cal)
{
	return cal->cal.iValidMagCal;
}

float
m_mag_calib_nxp_fit_error(const struct m_mag_calib_nxp *cal)
{
	return cal->cal.fFitErrorpc;
}

int
m_mag_calib_nxp_sample_count(const struct m_mag_calib_nxp *cal)
{
	return cal->buf.iMagBufferCount;
}

// --- Persistence ---

// Wait until there's work to do or the thread is stopped.
// Returns true if there's work, false if the thread should exit.
static bool
save_thread_wait(struct m_mag_calib_nxp *cal)
{
	os_thread_helper_lock(&cal->save_thread);
	while (os_thread_helper_is_running_locked(&cal->save_thread) && !cal->save_pending) {
		os_thread_helper_wait_locked(&cal->save_thread);
	}
	bool has_work = os_thread_helper_is_running_locked(&cal->save_thread) && cal->save_pending;
	os_thread_helper_unlock(&cal->save_thread);
	return has_work;
}

static void
save_write_file(struct m_mag_calib_nxp *cal)
{
	struct m_mag_calib_nxp_save_buf sb = cal->save_buf;

	FILE *f = u_file_open_file_in_config_dir(cal->save_filename, "w");
	if (f == NULL) {
		return;
	}
	fprintf(f,
	        "{\n"
	        "  \"hard_iron\": [%.6f, %.6f, %.6f],\n"
	        "  \"soft_iron\": [[%.6f, %.6f, %.6f], [%.6f, %.6f, %.6f], [%.6f, %.6f, %.6f]],\n"
	        "  \"field_strength\": %.6f\n"
	        "}\n",
	        sb.fV[0], sb.fV[1], sb.fV[2], sb.finvW[0][0], sb.finvW[0][1], sb.finvW[0][2], sb.finvW[1][0],
	        sb.finvW[1][1], sb.finvW[1][2], sb.finvW[2][0], sb.finvW[2][1], sb.finvW[2][2], sb.fB);
	fclose(f);
	U_LOG_D("Saved mag calibration to %s (error %.2f%%)", cal->save_filename, cal->last_saved_error);
}

static void *
save_thread_func(void *ptr)
{
	struct m_mag_calib_nxp *cal = (struct m_mag_calib_nxp *)ptr;

	while (save_thread_wait(cal)) {
		save_write_file(cal);
		os_thread_helper_lock(&cal->save_thread);
		cal->save_pending = false;
		os_thread_helper_unlock(&cal->save_thread);
	}

	return NULL;
}

static void
maybe_trigger_save(struct m_mag_calib_nxp *cal)
{
	if (!cal->save_thread.initialized) {
		return;
	}
	if (cal->cal.iValidMagCal < 10) {
		return;
	}
	if (cal->cal.fFitErrorpc >= cal->last_saved_error) {
		return;
	}

	os_thread_helper_lock(&cal->save_thread);
	if (!cal->save_pending) {
		for (int i = 0; i < 3; i++) {
			cal->save_buf.fV[i] = cal->cal.fV[i];
			for (int j = 0; j < 3; j++) {
				cal->save_buf.finvW[i][j] = cal->cal.finvW[i][j];
			}
		}
		cal->save_buf.fB = cal->cal.fB;
		cal->last_saved_error = cal->cal.fFitErrorpc;
		cal->save_pending = true;
		os_thread_helper_signal_locked(&cal->save_thread);
	}
	os_thread_helper_unlock(&cal->save_thread);
}

bool
m_mag_calib_nxp_load(struct m_mag_calib_nxp *cal, const char *filename)
{
	FILE *f = u_file_open_file_in_config_dir(filename, "r");
	if (f == NULL) {
		return false;
	}

	fseek(f, 0, SEEK_END);
	long len = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (len <= 0 || len > 4096) {
		fclose(f);
		return false;
	}

	char *buf = malloc((size_t)len + 1);
	if (buf == NULL) {
		fclose(f);
		return false;
	}
	size_t nread = fread(buf, 1, (size_t)len, f);
	fclose(f);
	buf[nread] = '\0';

	cJSON *root = cJSON_Parse(buf);
	free(buf);
	if (root == NULL) {
		return false;
	}

	struct xrt_vec3 hard_iron;
	const cJSON *j_hard = u_json_get(root, "hard_iron");
	if (j_hard == NULL || !u_json_get_vec3_array(j_hard, &hard_iron)) {
		cJSON_Delete(root);
		return false;
	}

	cal->cal.fV[0] = hard_iron.x;
	cal->cal.fV[1] = hard_iron.y;
	cal->cal.fV[2] = hard_iron.z;

	const cJSON *j_soft = u_json_get(root, "soft_iron");
	if (j_soft != NULL) {
		struct xrt_matrix_3x3 soft_iron;
		if (u_json_get_matrix_3x3(j_soft, &soft_iron)) {
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					cal->cal.finvW[i][j] = soft_iron.v[i * 3 + j];
				}
			}
		}
	}

	const cJSON *j_field = u_json_get(root, "field_strength");
	if (j_field != NULL) {
		float field = DEFAULTB;
		u_json_get_float(j_field, &field);
		cal->cal.fB = field;
	}

	cal->cal.fFourBsq = 4.0f * cal->cal.fB * cal->cal.fB;
	cal->cal.fFitErrorpc = 20.0f;
	cal->cal.iValidMagCal = 10;

	cJSON_Delete(root);
	U_LOG_D("Loaded mag calibration from %s", filename);
	return true;
}

void
m_mag_calib_nxp_save_start(struct m_mag_calib_nxp *cal, const char *filename)
{
	snprintf(cal->save_filename, sizeof(cal->save_filename), "%s", filename);
	cal->last_saved_error = 1000.0f;
	cal->save_pending = false;

	if (os_thread_helper_init(&cal->save_thread) != 0) {
		return;
	}
	if (os_thread_helper_start(&cal->save_thread, save_thread_func, cal) != 0) {
		return;
	}
}

void
m_mag_calib_nxp_save_stop(struct m_mag_calib_nxp *cal)
{
	if (!cal->save_thread.initialized) {
		return;
	}
	os_thread_helper_destroy(&cal->save_thread);
}
