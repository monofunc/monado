// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basic driver for RayNeo AR glasses
 * @author ChatGPT 5.3
 * @author Claude Opus 4.6
 * @ingroup drv_rayneo
 */

#include "rayneo_interface.h"
#include "rayneo_log.h"
#include "rayneo_proto.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_imu_xio_ahrs.h"
#include "math/m_mag_calib_nxp.h"
#include "math/m_predict.h"

#include "util/u_debug.h"
#include "util/u_linux.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include <string.h>


DEBUG_GET_ONCE_LOG_OPTION(rayneo_log, "RAYNEO_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_BOOL_OPTION(rayneo_fw_log, "RAYNEO_FW_LOG", false)
DEBUG_GET_ONCE_BOOL_OPTION(rayneo_disable_mag, "RAYNEO_DISABLE_MAG", false)


#define RAYNEO_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define RAYNEO_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define RAYNEO_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define RAYNEO_WARN(hmd, ...) U_LOG_XDEV_IFL_W(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define RAYNEO_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)

#define RAYNEO_POLL_TIMEOUT_MS 100
#define RAYNEO_RESPONSE_TIMEOUT_MS 1000
#define RAYNEO_RETRY_WAIT_MS 3000
#define RAYNEO_3D_MODE_SETTLE_MS 5000
#define RAYNEO_INIT_MAX_RETRIES 30
#define RAYNEO_SHUTDOWN_TIMEOUT_MS 500
#define RAYNEO_TIME_SYNC_TIMEOUT_MS 500
#define RAYNEO_CLOCK_TRACKER_WINDOW_SAMPLES 64

// Top-level thread phases.
enum rayneo_phase
{
	RAYNEO_PHASE_INIT,
	RAYNEO_PHASE_RUNNING,
	RAYNEO_PHASE_SHUTDOWN,
	RAYNEO_PHASE_DONE,
};

enum rayneo_init_state
{
	RAYNEO_INIT_OPEN_USB,
	RAYNEO_INIT_SWITCH_TO_3D,
	RAYNEO_INIT_TIME_SYNC,
	RAYNEO_INIT_PREPARE_TRACKING,
	RAYNEO_INIT_FINALIZE,
	RAYNEO_INIT_RETRY_WAIT,
	RAYNEO_INIT_OK,
	RAYNEO_INIT_FAILED,
};

enum rayneo_init_enter_3d_state
{
	RAYNEO_INIT_ENTER_3D_WAIT_CLOSE_IMU,
	RAYNEO_INIT_ENTER_3D_REQ_DEVICE_INFO,
	RAYNEO_INIT_ENTER_3D_WAIT_DEVICE_INFO,
	RAYNEO_INIT_ENTER_3D_REQ_3D_MODE,
	RAYNEO_INIT_ENTER_3D_WAIT_3D_MODE,
	RAYNEO_INIT_ENTER_3D_MODE_SETTLE,
};

enum rayneo_init_result
{
	RAYNEO_INIT_RESULT_PENDING,
	RAYNEO_INIT_RESULT_OK,
	RAYNEO_INIT_RESULT_FAILED,
};

typedef struct rayneo_device_params
{
	rayneo_board_id_t board_id;
	const char *name;
	uint32_t panel_w_pixels;
	uint32_t panel_h_pixels;
	struct xrt_vec3 imu_rot_deg;
	struct xrt_fov startup_fov;
	bool no_fov_request;
	bool imu_bypass_sw;
} rayneo_device_params_t;

#define RAYNEO_DEFAULT_PANEL_FIELDS .panel_w_pixels = 1920, .panel_h_pixels = 1080

#define RAYNEO_AIR_4_COMMON_FIELDS                                                                                     \
	RAYNEO_DEFAULT_PANEL_FIELDS, .imu_rot_deg = {.x = -20.0f},                                                     \
	                             .startup_fov = {.angle_left = -0.362156f,                                         \
	                                             .angle_right = 0.362156f,                                         \
	                                             .angle_up = 0.210482f,                                            \
	                                             .angle_down = -0.210482f},                                        \
	                             .no_fov_request = true

static const rayneo_device_params_t rayneo_known_devices[] = {
    /*
     * The only device verified with this table so far is RayNeo Air 4 Pro.
     */
    {.board_id = RAYNEO_BOARD_NEXTVIEW_PRO, .name = "NextView Pro", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_ARIES, .name = "Aries", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_ARIES_1_5_SEEYA, .name = "Aries 1.5 (Seeya)", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_ARIES_1_5_SONY, .name = "Aries 1.5 (Sony)", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_ARIES_1P8, .name = "Aries 1.8", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS, .name = "Taurus", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS_1_5, .name = "Taurus 1.5", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS_2_0, .name = "Taurus 2.0", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS_3_0_LOW, .name = "Taurus 3.0 Low", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS_3_0_PRO, .name = "Taurus 3.0 Pro", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_TAURUS_2_0_OVERSEAS, .name = "Taurus 2.0 Overseas", RAYNEO_DEFAULT_PANEL_FIELDS},
    {.board_id = RAYNEO_BOARD_AIR_4, .name = "RayNeo Air 4", RAYNEO_AIR_4_COMMON_FIELDS},
    {.board_id = RAYNEO_BOARD_AIR_4_PRO, .name = "RayNeo Air 4 Pro", RAYNEO_AIR_4_COMMON_FIELDS, .imu_bypass_sw = true},
};

#undef RAYNEO_DEFAULT_PANEL_FIELDS
#undef RAYNEO_AIR_4_COMMON_FIELDS

static inline const rayneo_device_params_t *
rayneo_find_device(uint8_t board_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(rayneo_known_devices); i++) {
		if ((uint8_t)rayneo_known_devices[i].board_id == board_id) {
			return &rayneo_known_devices[i];
		}
	}
	return NULL;
}

struct rayneo_hmd
{
	struct xrt_device base;
	enum u_logging_level log_level;

	// Mutex protects shared state + init signaling
	struct os_thread_helper thread;
	enum rayneo_init_result init_result;

	// The thread will release all resources and exit when the device is lost.
	xrt_atomic_s32_t device_lost;

	// IMU result
	struct xrt_space_relation last_relation;
	timepoint_ns last_update_ns;

	// Brightness control
	int32_t max_brightness;
	xrt_atomic_s32_t current_brightness;
	xrt_atomic_s32_t desired_brightness;

	struct
	{
		struct xrt_vec3 accel;
		struct xrt_vec3 gyro;
		struct xrt_vec3 mag;
		struct xrt_vec3 mag_raw;
		float temperature_c;
		int32_t mag_calib_level;
		float mag_calib_error;
		int32_t mag_calib_samples;
	} debug;
};

struct rayneo_thread_ctx
{
	struct rayneo_hmd *hmd;

	// USB Handle
	rayneo_device_t dev;

	// Allocated resources
	struct m_imu_xio_ahrs fusion;
	struct m_clock_windowed_skew_tracker *clock_tracker;

	// Online magnetometer calibration
	struct m_mag_calib_nxp mag_calib_online;

	// Pointer to hardware-specific parameters from the rayneo_known_devices
	const rayneo_device_params_t *device_params;

	// Parameters obtained from the device
	rayneo_device_info_t device_info;
	rayneo_panel_fov_t panel_fov;
	rayneo_imu_calibration_t imu_calib;
	struct xrt_matrix_3x3 imu_calib_matrix;
	struct xrt_vec3 imu_calib_accel_bias;

	// IMU frame rotation
	bool has_imu_rot;
	struct xrt_quat imu_rot;

	// Firmware log buffer
	rayneo_fw_log_buf_t fw_log_buf;
};

static inline struct rayneo_hmd *
rayneo_hmd(struct xrt_device *xdev)
{
	return (struct rayneo_hmd *)xdev;
}

static void
rayneo_sleep_ms(int ms)
{
	if (ms <= 0) {
		return;
	}

	os_nanosleep((int64_t)ms * U_TIME_1MS_IN_NS);
}

static uint64_t
rayneo_now_ms(void)
{
	return (uint64_t)(os_monotonic_get_ns() / U_TIME_1MS_IN_NS);
}

static bool
rayneo_timed_out(uint64_t start_ms, int timeout_ms)
{
	return (int)(rayneo_now_ms() - start_ms) >= timeout_ms;
}

// Check if thread should stop.
static bool
rayneo_thread_should_stop(struct rayneo_hmd *hmd)
{
	os_thread_helper_lock(&hmd->thread);
	bool running = os_thread_helper_is_running_locked(&hmd->thread);
	os_thread_helper_unlock(&hmd->thread);
	return !running;
}

static void
rayneo_compute_imu_calib_transform(const rayneo_imu_calibration_t *imu_calib,
                                   struct xrt_matrix_3x3 *out_matrix,
                                   struct xrt_vec3 *out_accel_bias)
{
	if (!imu_calib->valid) {
		math_matrix_3x3_identity(out_matrix);
		*out_accel_bias = (struct xrt_vec3)XRT_VEC3_ZERO;
		return;
	}

	const float *tsb = &imu_calib->tsb[0][0];
	*out_matrix = (struct xrt_matrix_3x3){{
	    // Convert tsb layout to the matrix convention used by math_matrix_3x3_transform_vec3.
	    tsb[0],
	    tsb[3],
	    tsb[6],
	    tsb[1],
	    tsb[4],
	    tsb[7],
	    tsb[2],
	    tsb[5],
	    tsb[8],
	}};
	*out_accel_bias = (struct xrt_vec3){
	    .x = imu_calib->ta[0],
	    .y = imu_calib->ta[1],
	    .z = imu_calib->ta[2],
	};
}

static struct xrt_vec3
rayneo_imu_apply_matrix(const struct xrt_vec3 *in, const struct xrt_matrix_3x3 *matrix, const struct xrt_vec3 *bias)
{
	struct xrt_vec3 out = XRT_VEC3_ZERO;
	math_matrix_3x3_transform_vec3(matrix, in, &out);

	if (bias != NULL) {
		out.x += bias->x;
		out.y += bias->y;
		out.z += bias->z;
	}

	return out;
}

static void
rayneo_publish_relation_locked(struct rayneo_thread_ctx *ctx,
                               timepoint_ns now_ns,
                               const struct xrt_quat *fusion_rot,
                               const struct xrt_vec3 *angular_velocity_ws)
{
	struct rayneo_hmd *hmd = ctx->hmd;

	hmd->last_relation.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
	                                    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	hmd->last_relation.pose.orientation = *fusion_rot;
	hmd->last_relation.angular_velocity = *angular_velocity_ws;
	hmd->last_update_ns = now_ns;
}

static void
rayneo_update_fusion(struct rayneo_thread_ctx *ctx,
                     const rayneo_sensor_sample_t *sample,
                     timepoint_ns observed_local_ts)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	struct xrt_quat fusion_rot = XRT_QUAT_IDENTITY;
	struct xrt_vec3 angular_velocity_ws = XRT_VEC3_ZERO;
	const float mps2_to_g = 1.0f / 9.80665f;
	struct xrt_vec3 accel = {
	    .x = sample->accel_mps2[0] * mps2_to_g,
	    .y = sample->accel_mps2[1] * mps2_to_g,
	    .z = sample->accel_mps2[2] * mps2_to_g,
	};
	struct xrt_vec3 gyro = {
	    .x = sample->gyro_dps[0],
	    .y = sample->gyro_dps[1],
	    .z = sample->gyro_dps[2],
	};
	struct xrt_vec3 mag = {
	    .x = sample->magnet[0],
	    .y = sample->magnet[1],
	    .z = sample->magnet[2],
	};
	if (ctx->imu_calib.valid) {
		accel = rayneo_imu_apply_matrix(&accel, &ctx->imu_calib_matrix, &ctx->imu_calib_accel_bias);
		gyro = rayneo_imu_apply_matrix(&gyro, &ctx->imu_calib_matrix, NULL);
	}

	// Capture raw mag before any processing for debug display.
	hmd->debug.mag_raw = mag;

	// Remap mag from chip frame to accel/gyro frame (MMC5633NJL mounting).
	mag = (struct xrt_vec3){.x = mag.y, .y = -mag.x, .z = mag.z};

	// Apply mounting rotation to all sensor vectors.
	if (ctx->has_imu_rot) {
		math_quat_rotate_vec3(&ctx->imu_rot, &accel, &accel);
		math_quat_rotate_vec3(&ctx->imu_rot, &gyro, &gyro);
		math_quat_rotate_vec3(&ctx->imu_rot, &mag, &mag);
	}

	// Online magnetometer calibration (after remap and imu_rot).
	bool has_mag = !debug_get_bool_option_rayneo_disable_mag() && (mag.x != 0.0f || mag.y != 0.0f || mag.z != 0.0f);
	if (has_mag) {
		m_mag_calib_nxp_update(&ctx->mag_calib_online, &mag, &accel);
		struct xrt_vec3 mag_cal;
		m_mag_calib_nxp_apply(&ctx->mag_calib_online, &mag, &mag_cal);
		mag = mag_cal;
		hmd->debug.mag_calib_level = m_mag_calib_nxp_level(&ctx->mag_calib_online);
		hmd->debug.mag_calib_error = m_mag_calib_nxp_fit_error(&ctx->mag_calib_online);
		hmd->debug.mag_calib_samples = m_mag_calib_nxp_sample_count(&ctx->mag_calib_online);
	}

	m_clock_windowed_skew_tracker_push(ctx->clock_tracker, observed_local_ts, sample->device_time_ns);
	timepoint_ns sample_local_ts = 0;
	if (!m_clock_windowed_skew_tracker_to_local(ctx->clock_tracker, sample->device_time_ns, &sample_local_ts)) {
		RAYNEO_DEBUG(hmd, "Clock conversion not ready yet; skipping sample");
		return;
	}
	if (sample_local_ts < hmd->last_update_ns) {
		sample_local_ts = hmd->last_update_ns;
	}

	hmd->debug.accel = accel;
	hmd->debug.gyro = gyro;
	hmd->debug.mag = mag;
	hmd->debug.temperature_c = sample->temperature_c;

	if (has_mag && m_mag_calib_nxp_level(&ctx->mag_calib_online) > 0) {
		m_imu_xio_ahrs_update_mag(&ctx->fusion, sample_local_ts, &accel, &gyro, &mag);
	} else {
		m_imu_xio_ahrs_update(&ctx->fusion, sample_local_ts, &accel, &gyro);
	}
	fusion_rot = ctx->fusion.rot;

	// Angular velocity for xrt_space_relation must be in rad/s.
	struct xrt_vec3 gyro_rad = {
	    .x = DEG_TO_RAD(gyro.x),
	    .y = DEG_TO_RAD(gyro.y),
	    .z = DEG_TO_RAD(gyro.z),
	};
	math_quat_rotate_vec3(&fusion_rot, &gyro_rad, &angular_velocity_ws);

	os_thread_helper_lock(&hmd->thread);
	rayneo_publish_relation_locked(ctx, sample_local_ts, &fusion_rot, &angular_velocity_ws);
	os_thread_helper_unlock(&hmd->thread);
}

// ------------------------------------------------------------
// Thread state machine
// ------------------------------------------------------------

static void
rayneo_signal_init_result(struct rayneo_hmd *hmd, enum rayneo_init_result result)
{
	os_thread_helper_lock(&hmd->thread);
	hmd->init_result = result;
	os_thread_helper_signal_locked(&hmd->thread);
	os_thread_helper_unlock(&hmd->thread);
}

static enum rayneo_init_state
rayneo_init_finalize(struct rayneo_thread_ctx *ctx);

static enum rayneo_init_state
rayneo_init_switch_to_3d(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	enum rayneo_init_enter_3d_state state = RAYNEO_INIT_ENTER_3D_WAIT_CLOSE_IMU;
	uint64_t state_enter_ms = rayneo_now_ms();

	// On RayNeo Air 4 Pro the firmware often fails to send responses while IMU stream is active.
	if (rayneo_proto_request_close_imu(dev) != 0) {
		RAYNEO_WARN(hmd, "Failed to send close IMU command");
		return RAYNEO_INIT_RETRY_WAIT;
	}

	while (true) {
		rayneo_packet_t pkt = {0};
		int rc = rayneo_proto_read_next(dev, &pkt, RAYNEO_POLL_TIMEOUT_MS);
		if (rc < 0) {
			RAYNEO_ERROR(hmd, "USB read error during switch-to-3D");
			return RAYNEO_INIT_RETRY_WAIT;
		}
		if (pkt.type == RAYNEO_PACKET_LOG_STREAM) {
			rayneo_log_fw_log(hmd->log_level, &ctx->fw_log_buf, &pkt.log_stream);
		}

		switch (state) {
		case RAYNEO_INIT_ENTER_3D_WAIT_CLOSE_IMU:
			if (pkt.type == RAYNEO_PACKET_ACK_CLOSE_IMU) {
				state = RAYNEO_INIT_ENTER_3D_REQ_DEVICE_INFO;
				break;
			}
			if (rayneo_timed_out(state_enter_ms, RAYNEO_RESPONSE_TIMEOUT_MS)) {
				RAYNEO_WARN(hmd, "Close IMU timeout, continuing");
				state = RAYNEO_INIT_ENTER_3D_REQ_DEVICE_INFO;
			}
			break;

		case RAYNEO_INIT_ENTER_3D_REQ_DEVICE_INFO:
			RAYNEO_DEBUG(hmd, "Requesting device info...");
			if (rayneo_proto_request_device_info(dev) != 0) {
				RAYNEO_WARN(hmd, "Failed to send device info request");
				return RAYNEO_INIT_RETRY_WAIT;
			}
			state = RAYNEO_INIT_ENTER_3D_WAIT_DEVICE_INFO;
			state_enter_ms = rayneo_now_ms();
			break;

		case RAYNEO_INIT_ENTER_3D_WAIT_DEVICE_INFO:
			if (pkt.type == RAYNEO_PACKET_DEVICE_INFO) {
				ctx->device_info = pkt.device_info;
				ctx->device_params = rayneo_find_device(ctx->device_info.board_id);
				if (ctx->device_params == NULL) {
					RAYNEO_WARN(hmd, "Unknown RayNeo board_id=%u, refusing to initialize",
					            ctx->device_info.board_id);
					return RAYNEO_INIT_FAILED;
				}
				rayneo_log_device_info(hmd->log_level, &ctx->device_info);
				if (ctx->device_info.side_by_side != 0) {
					return RAYNEO_INIT_TIME_SYNC;
				}
				state = RAYNEO_INIT_ENTER_3D_REQ_3D_MODE;
				break;
			}
			if (rayneo_timed_out(state_enter_ms, RAYNEO_RESPONSE_TIMEOUT_MS)) {
				RAYNEO_WARN(hmd, "Device info timeout");
				return RAYNEO_INIT_RETRY_WAIT;
			}
			break;

		case RAYNEO_INIT_ENTER_3D_REQ_3D_MODE:
			RAYNEO_DEBUG(hmd, "Switching to 3D mode...");
			if (rayneo_proto_request_set_mode(dev, RAYNEO_MODE_3D) != 0) {
				RAYNEO_WARN(hmd, "Failed to send 3D mode request");
				return RAYNEO_INIT_RETRY_WAIT;
			}
			state = RAYNEO_INIT_ENTER_3D_WAIT_3D_MODE;
			state_enter_ms = rayneo_now_ms();
			break;

		case RAYNEO_INIT_ENTER_3D_WAIT_3D_MODE:
			if (pkt.type == RAYNEO_PACKET_ACK_3D_MODE) {
				RAYNEO_DEBUG(hmd, "3D mode confirmed, waiting for settle...");
				state = RAYNEO_INIT_ENTER_3D_MODE_SETTLE;
				state_enter_ms = rayneo_now_ms();
				break;
			}
			if (rayneo_timed_out(state_enter_ms, RAYNEO_RESPONSE_TIMEOUT_MS)) {
				RAYNEO_WARN(hmd, "3D mode switch timeout");
				return RAYNEO_INIT_RETRY_WAIT;
			}
			break;

		case RAYNEO_INIT_ENTER_3D_MODE_SETTLE:
			if (rayneo_timed_out(state_enter_ms, RAYNEO_3D_MODE_SETTLE_MS)) {
				RAYNEO_DEBUG(hmd, "3D mode settle complete, re-checking device info...");
				state = RAYNEO_INIT_ENTER_3D_REQ_DEVICE_INFO;
			}
			break;
		}
	}
}

static enum rayneo_init_state
rayneo_init_time_sync(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	uint64_t state_enter_ms = 0;
	int sample_count = 0;

	if (rayneo_proto_request_time_sync(dev) != 0) {
		RAYNEO_WARN(hmd, "Failed to send time-sync request");
		return RAYNEO_INIT_RETRY_WAIT;
	}

	state_enter_ms = rayneo_now_ms();
	while (true) {
		rayneo_packet_t pkt = {0};
		int rc = rayneo_proto_read_next(dev, &pkt, RAYNEO_POLL_TIMEOUT_MS);
		if (rc < 0) {
			RAYNEO_ERROR(hmd, "USB read error during startup time-sync");
			return RAYNEO_INIT_RETRY_WAIT;
		}

		switch (pkt.type) {
		case RAYNEO_PACKET_LOG_STREAM:
			rayneo_log_fw_log(hmd->log_level, &ctx->fw_log_buf, &pkt.log_stream);
			break;
		case RAYNEO_PACKET_TIME_SYNC: {
			timepoint_ns receive_local_ns = os_monotonic_get_ns();
			m_clock_windowed_skew_tracker_push(ctx->clock_tracker, receive_local_ns,
			                                   pkt.time_sync.device_time_ns);
			sample_count++;
			break;
		}
		default: break;
		}

		if (sample_count >= RAYNEO_TIME_SYNC_EXPECTED_SAMPLES_PER_REQUEST ||
		    rayneo_timed_out(state_enter_ms, RAYNEO_TIME_SYNC_TIMEOUT_MS)) {
			break;
		}
	}

	if (sample_count == 0) {
		RAYNEO_WARN(hmd, "No startup time-sync packets received");
	} else {
		RAYNEO_DEBUG(hmd, "Startup time-sync packets=%d", sample_count);
	}

	return RAYNEO_INIT_PREPARE_TRACKING;
}

static enum rayneo_init_state
rayneo_init_prepare_tracking(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	const rayneo_device_params_t *device_params = ctx->device_params;
	uint64_t state_enter_ms = 0;
	bool got_imu_open_ack = false;
	bool got_imu_calib = false;
	bool got_panel_fov = device_params->no_fov_request;

	RAYNEO_DEBUG(hmd, "Preparing tracking ...");
	ctx->imu_calib.valid = 0;
	rayneo_compute_imu_calib_transform(&ctx->imu_calib, &ctx->imu_calib_matrix, &ctx->imu_calib_accel_bias);
	if (rayneo_proto_request_imu_calibration(dev) != 0) {
		RAYNEO_WARN(hmd, "Failed to send IMU calibration request");
		return RAYNEO_INIT_RETRY_WAIT;
	}
	if (!device_params->no_fov_request && rayneo_proto_request_panel_fov(dev) != 0) {
		RAYNEO_WARN(hmd, "Failed to send panel FOV request");
		return RAYNEO_INIT_RETRY_WAIT;
	}
	if (rayneo_proto_request_open_imu(dev, device_params->imu_bypass_sw) != 0) {
		RAYNEO_WARN(hmd, "Failed to send open IMU command");
		return RAYNEO_INIT_RETRY_WAIT;
	}
	state_enter_ms = rayneo_now_ms();

	while (true) {
		rayneo_packet_t pkt = {0};
		int rc = rayneo_proto_read_next(dev, &pkt, RAYNEO_POLL_TIMEOUT_MS);
		if (rc < 0) {
			RAYNEO_ERROR(hmd, "USB read error during prepare-tracking");
			return RAYNEO_INIT_RETRY_WAIT;
		}

		switch (pkt.type) {
		case RAYNEO_PACKET_LOG_STREAM:
			rayneo_log_fw_log(hmd->log_level, &ctx->fw_log_buf, &pkt.log_stream);
			break;
		case RAYNEO_PACKET_IMU_CALIBRATION:
			got_imu_calib = true;
			ctx->imu_calib = pkt.imu_calibration;
			rayneo_compute_imu_calib_transform(&ctx->imu_calib, &ctx->imu_calib_matrix,
			                                   &ctx->imu_calib_accel_bias);
			rayneo_log_imu_calibration(hmd->log_level, &pkt.imu_calibration);
			break;
		case RAYNEO_PACKET_PANEL_FOV:
			got_panel_fov = true;
			ctx->panel_fov = pkt.panel_fov;
			rayneo_log_panel_fov(hmd->log_level, &pkt.panel_fov);
			break;
		case RAYNEO_PACKET_SENSOR:
			// Sensor traffic implies IMU stream is active, even without explicit ACK.
			/* fallthrough */
		case RAYNEO_PACKET_ACK_OPEN_IMU:
			if (!got_imu_open_ack) {
				RAYNEO_DEBUG(hmd, "IMU stream opened");
			}
			got_imu_open_ack = true;
			break;
		default: break;
		}

		if (got_imu_open_ack && got_imu_calib && got_panel_fov) {
			return RAYNEO_INIT_FINALIZE;
		}

		if (rayneo_timed_out(state_enter_ms, RAYNEO_RESPONSE_TIMEOUT_MS)) {
			if (!got_imu_open_ack) {
				RAYNEO_WARN(hmd, "Open IMU timeout");
				return RAYNEO_INIT_RETRY_WAIT;
			}
			if (!got_imu_calib) {
				RAYNEO_WARN(hmd, "IMU calibration timeout, continuing");
			}
			if (!got_panel_fov) {
				RAYNEO_WARN(hmd, "Panel FOV timeout, continuing");
			}
			return RAYNEO_INIT_FINALIZE;
		}
	}
}

static enum rayneo_phase
rayneo_phase_init(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	enum rayneo_init_state state = RAYNEO_INIT_OPEN_USB;
	int retry_count = 0;

	while (true) {
		switch (state) {
		case RAYNEO_INIT_OPEN_USB:
			if (rayneo_usb_open(dev, RAYNEO_VID, RAYNEO_PID) != 0) {
				RAYNEO_WARN(hmd, "Failed to open USB device (%04x:%04x)", RAYNEO_VID, RAYNEO_PID);
				state = RAYNEO_INIT_RETRY_WAIT;
				break;
			}
			if (debug_get_bool_option_rayneo_fw_log()) {
				if (rayneo_proto_request_log_enable(dev, true) != 0) {
					state = RAYNEO_INIT_RETRY_WAIT;
					break;
				}
			}
			state = RAYNEO_INIT_SWITCH_TO_3D;
			break;

		case RAYNEO_INIT_SWITCH_TO_3D: {
			state = rayneo_init_switch_to_3d(ctx);
			break;
		}

		case RAYNEO_INIT_TIME_SYNC: {
			state = rayneo_init_time_sync(ctx);
			break;
		}

		case RAYNEO_INIT_PREPARE_TRACKING: {
			state = rayneo_init_prepare_tracking(ctx);
			break;
		}

		case RAYNEO_INIT_FINALIZE: {
			state = rayneo_init_finalize(ctx);
			break;
		}

		case RAYNEO_INIT_RETRY_WAIT:
			retry_count++;
			if (retry_count > RAYNEO_INIT_MAX_RETRIES) {
				RAYNEO_ERROR(hmd, "Init failed after %d retries", RAYNEO_INIT_MAX_RETRIES);
				state = RAYNEO_INIT_FAILED;
				break;
			}
			RAYNEO_WARN(hmd, "Init retry %d/%d, reopening device...", retry_count, RAYNEO_INIT_MAX_RETRIES);
			rayneo_usb_close(dev);
			m_clock_windowed_skew_tracker_reset(ctx->clock_tracker);
			rayneo_sleep_ms(RAYNEO_RETRY_WAIT_MS);
			ctx->panel_fov = (rayneo_panel_fov_t){0};
			ctx->imu_calib.valid = 0;
			rayneo_compute_imu_calib_transform(&ctx->imu_calib, &ctx->imu_calib_matrix,
			                                   &ctx->imu_calib_accel_bias);
			ctx->device_params = NULL;
			state = RAYNEO_INIT_OPEN_USB;
			break;

		case RAYNEO_INIT_OK: {
			rayneo_signal_init_result(hmd, RAYNEO_INIT_RESULT_OK);
			return RAYNEO_PHASE_RUNNING;
		}

		case RAYNEO_INIT_FAILED:
			rayneo_signal_init_result(hmd, RAYNEO_INIT_RESULT_FAILED);
			return RAYNEO_PHASE_DONE;
		}
	}
}

static enum rayneo_phase
rayneo_phase_running(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	while (true) {
		if (rayneo_thread_should_stop(hmd)) {
			return RAYNEO_PHASE_SHUTDOWN;
		}

		rayneo_packet_t pkt = {0};
		int rc = rayneo_proto_read_next(dev, &pkt, RAYNEO_POLL_TIMEOUT_MS);
		if (rc < 0) {
			RAYNEO_ERROR(hmd, "USB read error in streaming mode");
			xrt_atomic_s32_store(&hmd->device_lost, 1);
			return RAYNEO_PHASE_SHUTDOWN;
		}

		switch (pkt.type) {
		case RAYNEO_PACKET_SENSOR: {
			timepoint_ns receive_local_ns = os_monotonic_get_ns();
			rayneo_update_fusion(ctx, &pkt.sensor, receive_local_ns);
			break;
		}
		case RAYNEO_PACKET_BRIGHTNESS_SET: {
			// Accept device-reported brightness as the new target: the event may come from
			// hardware button presses, not just our own requests, and we can't distinguish them.
			int32_t brightness_level = (int32_t)pkt.brightness_response.brightness_level;
			if (brightness_level > hmd->max_brightness) {
				brightness_level = hmd->max_brightness;
			}
			xrt_atomic_s32_store(&hmd->current_brightness, brightness_level);
			xrt_atomic_s32_store(&hmd->desired_brightness, brightness_level);
			RAYNEO_DEBUG(hmd, "Brightness level updated: %d", brightness_level);
			break;
		}
		case RAYNEO_PACKET_VOLUME_UP:
			RAYNEO_DEBUG(hmd, "Volume up: level=%u", pkt.volume_response.volume_level);
			break;
		case RAYNEO_PACKET_VOLUME_DOWN:
			RAYNEO_DEBUG(hmd, "Volume down: level=%u", pkt.volume_response.volume_level);
			break;
		case RAYNEO_PACKET_LOG_STREAM:
			rayneo_log_fw_log(hmd->log_level, &ctx->fw_log_buf, &pkt.log_stream);
			break;
		default: break;
		}

		// Keep device brightness synced to target level.
		int32_t desired = xrt_atomic_s32_load(&hmd->desired_brightness);
		int32_t current = xrt_atomic_s32_load(&hmd->current_brightness);
		if (desired != current && desired >= 0) {
			if (rayneo_proto_request_brightness_set(dev, (uint8_t)desired) != 0) {
				RAYNEO_WARN(hmd, "Failed to send brightness set command (target %d)", desired);
			} else {
				RAYNEO_DEBUG(hmd, "Brightness set request: %d -> %d", current, desired);
				xrt_atomic_s32_store(&hmd->current_brightness, desired);
			}
		}
	}
}

static enum rayneo_phase
rayneo_phase_shutdown(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	rayneo_device_t *dev = &ctx->dev;
	if (xrt_atomic_s32_load(&hmd->device_lost)) {
		return RAYNEO_PHASE_DONE;
	}

	bool got_close_imu_ack = false;
	bool got_2d_mode_ack = false;
	uint64_t state_enter_ms = 0;

	RAYNEO_DEBUG(hmd, "Shutdown: closing IMU and switching to 2D mode...");
	if (debug_get_bool_option_rayneo_fw_log() && rayneo_proto_request_log_enable(dev, false) != 0) {
		return RAYNEO_PHASE_DONE;
	}
	if (rayneo_proto_request_close_imu(dev) != 0) {
		return RAYNEO_PHASE_DONE;
	}
	if (rayneo_proto_request_set_mode(dev, RAYNEO_MODE_2D) != 0) {
		return RAYNEO_PHASE_DONE;
	}
	state_enter_ms = rayneo_now_ms();

	while (true) {
		rayneo_packet_t pkt = {0};
		int rc = rayneo_proto_read_next(dev, &pkt, RAYNEO_POLL_TIMEOUT_MS);
		if (rc < 0) {
			RAYNEO_WARN(hmd, "USB read error during shutdown, giving up");
			return RAYNEO_PHASE_DONE;
		}

		switch (pkt.type) {
		case RAYNEO_PACKET_LOG_STREAM:
			rayneo_log_fw_log(hmd->log_level, &ctx->fw_log_buf, &pkt.log_stream);
			break;
		case RAYNEO_PACKET_ACK_CLOSE_IMU: got_close_imu_ack = true; break;
		case RAYNEO_PACKET_ACK_2D_MODE: got_2d_mode_ack = true; break;
		default: break;
		}

		if ((got_close_imu_ack && got_2d_mode_ack) ||
		    rayneo_timed_out(state_enter_ms, RAYNEO_SHUTDOWN_TIMEOUT_MS)) {
			return RAYNEO_PHASE_DONE;
		}
	}
}

static void *
rayneo_thread_run(void *ptr)
{
	struct rayneo_hmd *hmd = (struct rayneo_hmd *)ptr;
	U_TRACE_SET_THREAD_NAME("RayNeo");
#ifdef XRT_OS_LINUX
	u_linux_try_to_set_realtime_priority_on_thread(U_LOGGING_INFO, "RayNeo USB thread");
#endif

	struct rayneo_thread_ctx ctx = {
	    .hmd = hmd,
	};
	struct m_imu_xio_ahrs_settings ahrs_settings = M_IMU_XIO_AHRS_SETTINGS_DEFAULT;
	ahrs_settings.magnetic_rejection = 70.0f;
	ahrs_settings.recovery_trigger_period = 5 * 200; // 5 seconds at ~200Hz IMU rate
	m_imu_xio_ahrs_init(&ctx.fusion, &ahrs_settings);
	m_mag_calib_nxp_init(&ctx.mag_calib_online, 1.0f);
	ctx.clock_tracker = m_clock_windowed_skew_tracker_alloc(RAYNEO_CLOCK_TRACKER_WINDOW_SAMPLES);
	if (ctx.clock_tracker == NULL) {
		RAYNEO_ERROR(hmd, "Failed to allocate clock tracker");
		rayneo_signal_init_result(hmd, RAYNEO_INIT_RESULT_FAILED);
		return NULL;
	}

	enum rayneo_phase phase = RAYNEO_PHASE_INIT;
	while (phase != RAYNEO_PHASE_DONE) {
		switch (phase) {
		case RAYNEO_PHASE_INIT: phase = rayneo_phase_init(&ctx); break;
		case RAYNEO_PHASE_RUNNING: phase = rayneo_phase_running(&ctx); break;
		case RAYNEO_PHASE_SHUTDOWN: phase = rayneo_phase_shutdown(&ctx); break;
		case RAYNEO_PHASE_DONE: break;
		}
	}

	RAYNEO_DEBUG(hmd, "Thread exiting, closing device");
	m_mag_calib_nxp_save_stop(&ctx.mag_calib_online);
	m_clock_windowed_skew_tracker_destroy(ctx.clock_tracker);
	rayneo_usb_close(&ctx.dev);
	return NULL;
}

static bool
rayneo_four_f32_nonzero(float a, float b, float c, float d)
{
	const float eps = 0.000001f;
	return fabsf(a) > eps && fabsf(b) > eps && fabsf(c) > eps && fabsf(d) > eps;
}

static void
rayneo_setup_views(struct rayneo_hmd *hmd, const rayneo_device_params_t *params, const rayneo_panel_fov_t *panel_fov)
{
	const uint32_t panel_w = params->panel_w_pixels;
	const uint32_t panel_h = params->panel_h_pixels;
	const uint32_t view_count = hmd->base.hmd->view_count;
	const uint32_t eye_w = panel_w;
	const uint32_t eye_h = panel_h;
	const struct xrt_fov default_fov = {
	    .angle_left = -0.362156f,
	    .angle_right = 0.362156f,
	    .angle_up = 0.210482f,
	    .angle_down = -0.210482f,
	};
	struct xrt_fov selected_fov = default_fov;

	assert(view_count > 0 && view_count <= 2);
	assert(panel_w > 0 && panel_h > 0);

	// Common HMD setup.
	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_ADDITIVE;
	hmd->base.hmd->blend_mode_count = 1;
	if (hmd->base.hmd->distortion.models == 0) {
		hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_NONE;
		hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_NONE;
	}
	hmd->base.hmd->screens[0].w_pixels = panel_w * view_count;
	hmd->base.hmd->screens[0].h_pixels = panel_h;

	// Per-view display and viewport layout, split side-by-side.
	for (uint32_t i = 0; i < view_count; i++) {
		hmd->base.hmd->views[i].display.w_pixels = eye_w;
		hmd->base.hmd->views[i].display.h_pixels = eye_h;
		hmd->base.hmd->views[i].viewport.x_pixels = eye_w * i;
		hmd->base.hmd->views[i].viewport.y_pixels = 0;
		hmd->base.hmd->views[i].viewport.w_pixels = eye_w;
		hmd->base.hmd->views[i].viewport.h_pixels = eye_h;
		hmd->base.hmd->views[i].rot = u_device_rotation_ident;
	}

	if (rayneo_four_f32_nonzero(params->startup_fov.angle_left, params->startup_fov.angle_right,
	                            params->startup_fov.angle_up, params->startup_fov.angle_down)) {
		selected_fov = params->startup_fov;
		RAYNEO_DEBUG(hmd, "Applying per-board startup FOV");
	} else if (rayneo_four_f32_nonzero(panel_fov->left_deg, panel_fov->right_deg, panel_fov->top_deg,
	                                   panel_fov->bottom_deg)) {
		selected_fov = (struct xrt_fov){
		    .angle_left = -DEG_TO_RAD(panel_fov->left_deg),
		    .angle_right = DEG_TO_RAD(panel_fov->right_deg),
		    .angle_up = DEG_TO_RAD(panel_fov->top_deg),
		    .angle_down = -DEG_TO_RAD(panel_fov->bottom_deg),
		};
		RAYNEO_DEBUG(hmd, "No per-board startup FOV, applying panel-reported FOV");
	} else {
		RAYNEO_DEBUG(hmd, "No per-board startup FOV, applying default startup FOV");
	}
	hmd->base.hmd->distortion.fov[0] = selected_fov;
	hmd->base.hmd->distortion.fov[1] = selected_fov;
	rayneo_log_fov(hmd->log_level, &selected_fov, hmd->base.hmd->views[0].display.w_pixels,
	               hmd->base.hmd->views[0].display.h_pixels);
}

static xrt_result_t
rayneo_hmd_get_brightness(struct xrt_device *xdev, float *out_brightness)
{
	struct rayneo_hmd *hmd = rayneo_hmd(xdev);
	int32_t max_brightness = hmd->max_brightness;
	if (max_brightness == 0) {
		*out_brightness = 1.0f;
		return XRT_SUCCESS;
	}

	int32_t current = xrt_atomic_s32_load(&hmd->current_brightness);
	*out_brightness = (float)current / (float)max_brightness;
	return XRT_SUCCESS;
}

static xrt_result_t
rayneo_hmd_set_brightness(struct xrt_device *xdev, float brightness, bool relative)
{
	struct rayneo_hmd *hmd = rayneo_hmd(xdev);
	int32_t max_brightness = hmd->max_brightness;

	int32_t target;
	if (relative) {
		int32_t current = xrt_atomic_s32_load(&hmd->current_brightness);
		int32_t delta = (int32_t)(brightness * (float)max_brightness);
		target = current + delta;
	} else {
		target = (int32_t)(brightness * (float)max_brightness + 0.5f);
	}

	// Clamp to valid range.
	if (target < 0) {
		target = 0;
	}
	if (target > max_brightness) {
		target = max_brightness;
	}

	xrt_atomic_s32_store(&hmd->desired_brightness, target);
	return XRT_SUCCESS;
}

static xrt_result_t
rayneo_get_compositor_info(struct xrt_device *xdev,
                           const struct xrt_device_compositor_mode *mode,
                           struct xrt_device_compositor_info *out_info)
{
	const double scanout_multiplier = 1080.0 / 1125.0;
	const int64_t scanout_time_ns = (int64_t)(mode->frame_interval_ns * scanout_multiplier);

	*out_info = (struct xrt_device_compositor_info){
	    .scanout_direction = XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM,
	    .scanout_time_ns = scanout_time_ns,
	};

	(void)xdev;
	return XRT_SUCCESS;
}

static xrt_result_t
rayneo_hmd_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            int64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	struct rayneo_hmd *hmd = rayneo_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}
	if (xrt_atomic_s32_load(&hmd->device_lost)) {
		*out_relation = hmd->last_relation;
		return XRT_ERROR_POSE_NOT_ACTIVE;
	}

	os_thread_helper_lock(&hmd->thread);
	struct xrt_space_relation last_relation = hmd->last_relation;
	timepoint_ns last_update_ns = hmd->last_update_ns;
	os_thread_helper_unlock(&hmd->thread);

	if (at_timestamp_ns > last_update_ns) {
		double time_diff = time_ns_to_s(at_timestamp_ns - last_update_ns);
		if (time_diff > 0.1) {
			time_diff = 0.1;
		}
		m_predict_relation(&last_relation, time_diff, out_relation);
	} else {
		*out_relation = last_relation;
	}

	return XRT_SUCCESS;
}

static void
rayneo_compute_imu_rotation(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	const struct xrt_vec3 *rot_deg = &ctx->device_params->imu_rot_deg;

	ctx->has_imu_rot = fabsf(rot_deg->x) > 0.001f || fabsf(rot_deg->y) > 0.001f || fabsf(rot_deg->z) > 0.001f;
	if (ctx->has_imu_rot) {
		struct xrt_vec3 angles = {
		    .x = DEG_TO_RAD(rot_deg->x),
		    .y = DEG_TO_RAD(rot_deg->y),
		    .z = DEG_TO_RAD(rot_deg->z),
		};
		math_quat_from_euler_angles(&angles, &ctx->imu_rot);
		RAYNEO_DEBUG(hmd, "IMU rotation: X=%0.2f Y=%0.2f Z=%0.2f deg", rot_deg->x, rot_deg->y, rot_deg->z);
	} else {
		ctx->imu_rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
	}
}

/*
 * Finalize initialization: set up all driver state after USB protocol init.
 * Called from the thread in RAYNEO_INIT_FINALIZE, before signaling create().
 */
static enum rayneo_init_state
rayneo_init_finalize(struct rayneo_thread_ctx *ctx)
{
	struct rayneo_hmd *hmd = ctx->hmd;
	const rayneo_device_params_t *device_params = ctx->device_params;

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "%s", device_params->name);
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
	         ctx->device_info.cpuid[0], ctx->device_info.cpuid[1], ctx->device_info.cpuid[2],
	         ctx->device_info.cpuid[3], ctx->device_info.cpuid[4], ctx->device_info.cpuid[5],
	         ctx->device_info.cpuid[6], ctx->device_info.cpuid[7], ctx->device_info.cpuid[8],
	         ctx->device_info.cpuid[9], ctx->device_info.cpuid[10], ctx->device_info.cpuid[11]);

	// Hooks.
	hmd->base.update_inputs = u_device_noop_update_inputs;
	hmd->base.get_tracked_pose = rayneo_hmd_get_tracked_pose;
	hmd->base.get_view_poses = u_device_get_view_poses;
	hmd->base.get_compositor_info = rayneo_get_compositor_info;
	hmd->base.get_brightness = rayneo_hmd_get_brightness;
	hmd->base.set_brightness = rayneo_hmd_set_brightness;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.compositor_info = true;
	hmd->base.supported.brightness_control = true;

	// IMU rotation from device table.
	rayneo_compute_imu_rotation(ctx);

	// Online mag calibration: load saved state and start background save thread.
	if (!debug_get_bool_option_rayneo_disable_mag()) {
		char mag_state_file[512];
		snprintf(mag_state_file, sizeof(mag_state_file), "mag_cal_rayneo_%s.json", hmd->base.serial);
		m_mag_calib_nxp_load(&ctx->mag_calib_online, mag_state_file);
		m_mag_calib_nxp_save_start(&ctx->mag_calib_online, mag_state_file);
	}

	// Fusion state.
	hmd->last_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	hmd->last_update_ns = os_monotonic_get_ns();
	xrt_atomic_s32_store(&hmd->device_lost, 0);

	// Views and display.
	rayneo_setup_views(hmd, device_params, &ctx->panel_fov);
	hmd->base.hmd->screens[0].nominal_frame_interval_ns =
	    (ctx->device_info.glasses_fps != 0) ? time_s_to_ns(1.0f / (float)ctx->device_info.glasses_fps)
	                                        : time_s_to_ns(1.0f / 60.0f);
	u_distortion_mesh_set_none(&hmd->base);

	// Brightness.
	hmd->max_brightness = ctx->device_info.max_brightness;
	int32_t brightness_level = ctx->device_info.brightness;
	if (brightness_level > hmd->max_brightness) {
		brightness_level = hmd->max_brightness;
	}
	xrt_atomic_s32_store(&hmd->current_brightness, brightness_level);
	xrt_atomic_s32_store(&hmd->desired_brightness, brightness_level);

	// Debug variables.
	u_var_add_root(hmd, hmd->base.str, true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_pose(hmd, &hmd->last_relation.pose, "last_pose");
	u_var_add_ro_vec3_f32(hmd, &hmd->debug.gyro, "gyro");
	u_var_add_ro_vec3_f32(hmd, &hmd->debug.accel, "accel");
	u_var_add_ro_vec3_f32(hmd, &hmd->debug.mag, "mag_cal");
	u_var_add_ro_vec3_f32(hmd, &hmd->debug.mag_raw, "mag_raw");
	u_var_add_ro_f32(hmd, &hmd->debug.temperature_c, "temperature_c");
	u_var_add_ro_i32(hmd, &hmd->debug.mag_calib_level, "mag_calib_level");
	u_var_add_ro_f32(hmd, &hmd->debug.mag_calib_error, "mag_calib_error");
	u_var_add_ro_i32(hmd, &hmd->debug.mag_calib_samples, "mag_calib_samples");
	u_var_add_ro_i64(hmd, &hmd->last_update_ns, "timestamp");
	RAYNEO_DEBUG(hmd, "%s (%s)", hmd->base.str, hmd->base.serial);
	return RAYNEO_INIT_OK;
}

static void
rayneo_hmd_destroy(struct xrt_device *xdev)
{
	struct rayneo_hmd *hmd = rayneo_hmd(xdev);
	RAYNEO_DEBUG(hmd, "Destroy begin");
	u_var_remove_root(hmd);

	// os_thread_helper_destroy signals stop and joins the thread.
	// The thread's state machine handles shutdown (close IMU, set 2D, close device).
	if (hmd->thread.initialized) {
		os_thread_helper_destroy(&hmd->thread);
	}

	u_device_free(&hmd->base);
}

static bool
rayneo_hmd_start_thread_and_wait_init(struct rayneo_hmd *hmd)
{
	if (os_thread_helper_init(&hmd->thread) != 0) {
		RAYNEO_ERROR(hmd, "Failed to initialize thread helper");
		return false;
	}
	if (os_thread_helper_start(&hmd->thread, rayneo_thread_run, hmd) != 0) {
		RAYNEO_ERROR(hmd, "Failed to start driver thread");
		return false;
	}
	os_thread_helper_name(&hmd->thread, "RayNeo thread");

	os_thread_helper_lock(&hmd->thread);
	while (hmd->init_result == RAYNEO_INIT_RESULT_PENDING) {
		os_thread_helper_wait_locked(&hmd->thread);
	}
	enum rayneo_init_result result = hmd->init_result;
	os_thread_helper_unlock(&hmd->thread);

	if (result != RAYNEO_INIT_RESULT_OK) {
		RAYNEO_ERROR(hmd, "Thread initialization failed");
		return false;
	}

	return true;
}

static struct xrt_device *
rayneo_hmd_create(void)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct rayneo_hmd *hmd = U_DEVICE_ALLOCATE(struct rayneo_hmd, flags, 1, 0);
	hmd->log_level = debug_get_log_option_rayneo_log();
	hmd->base.destroy = rayneo_hmd_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;

	if (!rayneo_hmd_start_thread_and_wait_init(hmd)) {
		rayneo_hmd_destroy(&hmd->base);
		return NULL;
	}

	return &hmd->base;
}

int
rayneo_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t device_count,
             size_t index,
             cJSON *attached_data,
             struct xrt_device **out_xdev)
{
	(void)xp;
	(void)device_count;
	(void)attached_data;
	(void)devices;
	(void)index;

	struct xrt_device *device = rayneo_hmd_create();
	if (device == NULL) {
		return 0;
	}

	out_xdev[0] = device;
	return 1;
}
