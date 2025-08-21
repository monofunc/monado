// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Device driver for CAVE environments.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#include "vroom_device.h"
#include "vroom_debug.h"
#include "vroom_config.hpp"

#include "xrt/xrt_device.h"

#include "os/os_time.h"
#include "os/os_threading.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_distortion_mesh.h"

#include "vroom_flystick.h"
#include "util/u_visibility_mask.h"

#include <pthread.h>

#include <cstdio>

#include "joycon.h"
#include "JoyShockLibrary.h"

#define DTRACK_PORT 1234

/*
 *
 * Structs and defines.
 *
 */

/*!
 * A CAVE device.
 *
 * @implements xrt_device
 */

/// Casting helper function
static inline struct vroom_device *
vroom_device(struct xrt_device *xdev)
{
	return (struct vroom_device *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(vroom_log, "VROOM_LOG", U_LOGGING_WARN)

#define CAVE_TRACE(p, ...) U_LOG_XDEV_IFL_T(&vroom->base, vroom->log_level, __VA_ARGS__)
#define CAVE_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&vroom->base, vroom->log_level, __VA_ARGS__)
#define CAVE_WARN(p, ...) U_LOG_XDEV_IFL_W(&vroom->base, vroom->log_level, __VA_ARGS__)
#define CAVE_ERROR(p, ...) U_LOG_XDEV_IFL_E(&vroom->base, vroom->log_level, __VA_ARGS__)


pthread_t debugThread;

static void
vroom_destroy(struct xrt_device *xdev)
{
	struct vroom_device *vroom = vroom_device(xdev);

	// Remove the variable tracking.
	u_var_remove_root(vroom);

	// Destroy the thread
	os_mutex_destroy(&vroom->mutex);

	// Properly close information window
	vroom_close_debug_window();
	pthread_join(debugThread, nullptr);

	// Properly close DTrack thread
	if (vroom->dtrack) {
		vroom_dtrack_stop();
		pthread_join(vroom->dtrack_thread, nullptr);
		delete vroom->dtrack;
	}

	// Properly close VRPN thread
	if (vroom->vrpn) {
		vroom_vrpn_stop();
		pthread_join(vroom->vrpn_thread, nullptr);
		delete vroom->vrpn;
	}

	// Free controllers if they were created
	if (vroom->flystick) {
		xrt_device_destroy((struct xrt_device **)&vroom->flystick);
	}
	if (vroom->joycon_left) {
		xrt_device_destroy((struct xrt_device **)&vroom->joycon_left);
	}
	if (vroom->joycon_right) {
		xrt_device_destroy((struct xrt_device **)&vroom->joycon_right);
	}

	JslDisconnectAndDisposeAll();

	// Free device
	u_device_free(&vroom->base);
}

static xrt_result_t
vroom_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

static xrt_result_t
vroom_get_tracked_pose(struct xrt_device *xdev,
                        enum xrt_input_name name,
                        int64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	struct vroom_device *vroom = vroom_device(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		CAVE_ERROR(vroom, "unknown input name");
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	// DTRACK
	// auto dt = vroom->dtrack;
	if (vroom->update_pose) {
		vroom->update_pose(vroom, 0, &vroom->pose);
	}
	else {
		CAVE_WARN(vroom, "no tracking for head");
	}

	// VRPN
	// sh->pose = sh->vrpn->get_corrected_pose(sh->vrpn->headPose);

	// Estimate pose at timestamp at_timestamp_ns!
	math_quat_normalize(&vroom->pose.orientation);
	out_relation->pose = vroom->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
		XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	return XRT_SUCCESS;
}

static xrt_result_t
vroom_get_view_poses(struct xrt_device *xdev,
                      const struct xrt_vec3 *default_eye_relation,
                      int64_t at_timestamp_ns,
                      uint32_t view_count,
                      struct xrt_space_relation *out_head_relation,
                      struct xrt_fov *out_fovs,
                      struct xrt_pose *out_poses)
{
	struct vroom_device *vroom = vroom_device(xdev);
	xrt_vec3 eye_trans{0};

	// Prepare positions
	for (int i = 0; i < view_count; ++i) {
		out_poses[i].position.x = vroom->pose.position.x;
		out_poses[i].position.y = vroom->pose.position.y;
		out_poses[i].position.z = vroom->pose.position.z;

		out_poses[i].orientation.x = 0;
		out_poses[i].orientation.y = 0;
		out_poses[i].orientation.z = 0;
		out_poses[i].orientation.w = 1;
	}

	// Stereoscopy (pass 1)
	if (vroom->enable_3d) {
		auto ipd = vroom->ipd;
		if (vroom->invert_eyes)
			ipd = -ipd;

		auto half_ipd = ipd * 0.5;

		eye_trans.x = (float)half_ipd;
		eye_trans.y = 0;
		eye_trans.z = 0;
		math_quat_rotate_vec3(&vroom->pose.orientation, &eye_trans, &eye_trans);

		// Adjust eye position to match actual pos
		for (int i = 0; i < view_count; ++i) {

			if (i % 2 == 0) {
				out_poses[i].position.x += eye_trans.x;
				out_poses[i].position.y += eye_trans.y;
				out_poses[i].position.z += eye_trans.z;
			} else {
				out_poses[i].position.x -= eye_trans.x;
				out_poses[i].position.y -= eye_trans.y;
				out_poses[i].position.z -= eye_trans.z;
			}
		}
	} else {
		eye_trans.x = 0;
		eye_trans.y = 0;
		eye_trans.z = 0;
	}

	int views_per_display = vroom->config->stereo ? 2 : 1;

	// Calculate FOVs
	for (int i = 0; i < view_count; ++i) {
		struct xrt_pose *pose = &out_poses[i];
		struct xrt_fov *fov = &out_fovs[i];

		struct xrt_vec2 dimensions = vroom->config->displays[i / views_per_display].dimensions;
		struct xrt_pose transform = vroom->display_transforms[i / views_per_display];

		struct xrt_vec3 transformed{pose->position.x + transform.position.x,
		                            pose->position.y + transform.position.y,
		                            pose->position.z + transform.position.z};

		math_quat_rotate_vec3(&transform.orientation, &transformed, &transformed);

		float left = transformed.x + dimensions.x / 2;
		float right = dimensions.x / 2 - transformed.x;
		float bottom = transformed.y + dimensions.y / 2;
		float top = dimensions.y / 2 - transformed.y;

		//! Distance to the screen.
		float depth = transformed.z;

		math_quat_invert(&transform.orientation, &pose->orientation);

		fov->angle_left = -atan2f(left, depth);
		fov->angle_right = atan2f(right, depth);
		fov->angle_up = atan2f(top, depth);
		fov->angle_down = -atan2f(bottom, depth);
	}

	// Stereoscopy (pass 2)
	for (int p = 0; p < view_count; ++p) {
		if (p % 2 == 0) {
			out_poses[p].position.x = +eye_trans.x;
			out_poses[p].position.y = +eye_trans.y;
			out_poses[p].position.z = +eye_trans.z;
		} else {
			out_poses[p].position.x = -eye_trans.x;
			out_poses[p].position.y = -eye_trans.y;
			out_poses[p].position.z = -eye_trans.z;
		}
	}

	vroom->frame_count++;

	out_head_relation->relation_flags =
		(xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                           XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                           XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                           XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	out_head_relation->pose.position = vroom->pose.position;
	out_head_relation->pose.orientation = vroom->pose.orientation;

	return XRT_SUCCESS;
}

xrt_result_t
vroom_get_visibility_mask(struct xrt_device *xdev,
                           enum xrt_visibility_mask_type type,
                           uint32_t view_index,
                           struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov{};

	auto half_pi = (float)(2 * atan(1)); // 90°

	fov.angle_left = -half_pi;
	fov.angle_down = -half_pi;
	fov.angle_up = half_pi;
	fov.angle_right = half_pi;

	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

struct xrt_device *
vroom_get_controller(struct xrt_device *dev, int controller)
{

	struct vroom_device *vroom = vroom_device(dev);

	switch (controller) {
	case 0: // flystick
		return (struct xrt_device *)vroom->flystick;
	case 1: // JoyCon Left
		return (struct xrt_device *)vroom->joycon_left;
	case 2: // JoyCon Right
		return (struct xrt_device *)vroom->joycon_right;
	default: // ?
		CAVE_WARN(vroom, "unknown controller id");
	}

	return nullptr;
}

void
vroom_setup_joycons(struct vroom_device *vroom)
{
	int connected = JslConnectDevices();
	int *handles = new int[connected];
	JslGetConnectedDeviceHandles(handles, connected);

	// Add a warning if no controllers are connected
	if (connected == 0) {
		CAVE_WARN(vroom, "No Joy-Con controllers found");
		delete[] handles;
		return;
	}

	bool left_set = false;
	bool right_set = false;

	for (int i = 0; i < connected; ++i) {
		if (!left_set && joycon_controller_set_handle(vroom->joycon_left, handles[i])) {
			left_set = true;
			continue;
		}
		if (!right_set && joycon_controller_set_handle(vroom->joycon_right, handles[i])) {
			right_set = true;
			continue;
		}
	}

	// Add warnings for controllers that couldn't be set up
	if (!left_set) {
		CAVE_WARN(vroom, "No left Joy-Con controller found");
	}
	if (!right_set) {
		CAVE_WARN(vroom, "No right Joy-Con controller found");
	}

	delete[] handles;
}


struct xrt_device *
vroom_create(void)
{
	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
		(enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct vroom_device *vroom = U_DEVICE_ALLOCATE(struct vroom_device, flags, 3, 0);

	os_mutex_init(&vroom->mutex);

	// This list should be ordered, most preferred first.
	{
		size_t idx = 0;
		vroom->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
		vroom->base.hmd->blend_mode_count = idx;

		vroom->base.update_inputs = vroom_update_inputs;
		vroom->base.get_tracked_pose = vroom_get_tracked_pose;
		vroom->base.get_view_poses = vroom_get_view_poses;
		vroom->base.destroy = vroom_destroy;
		vroom->base.get_visibility_mask = vroom_get_visibility_mask;
		vroom->base.tracking_origin->type = XRT_TRACKING_TYPE_OTHER;

		vroom->pose = XRT_POSE_IDENTITY;
		vroom->log_level = debug_get_log_option_vroom_log();
	}

	// Print name.
	{
		snprintf(vroom->base.str, XRT_DEVICE_NAME_LEN, "VROOM");
		snprintf(vroom->base.serial, XRT_DEVICE_NAME_LEN, "0123456789abcdef");
	}

	// Setup input.
	vroom->base.name = XRT_DEVICE_GENERIC_HMD;
	vroom->base.device_type = XRT_DEVICE_TYPE_HMD;

	vroom->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	// Load configuration
	vroom->config = vroom_load_config();
	if (vroom->config == nullptr) {
		auto c = vroom_create_config();
		vroom_save_config(&c, "vroom.json");
		vroom->config = vroom_load_config();
		CAVE_WARN(vroom, "No config file found. Created new config at vroom.json");
	}

	bool stereo = vroom->config->stereo;

	vroom->base.hmd->stereo = stereo;

	// Prepare displays
	{
		int views_per_display = stereo ? 2 : 1;

		vroom->base.hmd->screen_count = vroom->config->display_count;
		vroom->base.hmd->view_count = vroom->config->display_count * views_per_display;
		vroom->display_transforms = new xrt_pose[vroom->config->display_count];

		auto screens = vroom->base.hmd->screens;
		auto views = vroom->base.hmd->views;


		for (int d = 0; d < vroom->config->display_count; ++d) {
			auto &display = vroom->config->displays[d];

			// Window size
			screens[d].x_pixels = display.screenPosition.x;
			screens[d].y_pixels = display.screenPosition.y;
			screens[d].w_pixels = display.screenSize.x;
			screens[d].h_pixels = display.screenSize.y;
			screens[d].nominal_frame_interval_ns = time_s_to_ns(1.0f / 60.0f);

			printf("display %d: pos: %d;%d, size %dx%d\n", d, display.screenPosition.x,
			       display.screenPosition.y, display.screenSize.x, display.screenSize.y);

			// Transform
			xrt_vec3 rot_rad{
				(float)(display.rotation.x * M_PI / 180.f),
				(float)(display.rotation.y * M_PI / 180.f),
				(float)(display.rotation.z * M_PI / 180.f),
			};

			vroom->display_transforms[d].position = {-display.position.x, -display.position.y,
			                                        -display.position.z};
			math_quat_from_euler_angles(&rot_rad, &vroom->display_transforms[d].orientation);



			for (int v = 0; v < views_per_display; ++v) {
				int view = (views_per_display * d) + v;
				// Viewport size
				views[view].viewport.x_pixels = 0;
				views[view].viewport.y_pixels = 0;
				views[view].viewport.w_pixels = display.screenSize.x;
				views[view].viewport.h_pixels = display.screenSize.y;

				// Texture size
				int texX = 1, texY = 1;

				if (display.textureSize.x < 0) {
					do {
						texX *= 2;
					} while (texX < display.screenSize.x);
				} else {
					texX = display.textureSize.x;
				}

				if (display.textureSize.y < 0) {
					do {
						texY *= 2;
					} while (texY < display.screenSize.y);
				} else {
					texY = display.textureSize.y;
				}

				views[view].display.w_pixels = texX;
				views[view].display.h_pixels = texY;

				// Rotation
				views[view].rot = u_device_rotation_ident;
				views[view].target_index = d;
				views[view].eyes = v == 0 ? XRT_EYE_LEFT : XRT_EYE_RIGHT;

				printf("view %d: tex: %dx%d\n", view, texX, texY);
			}
		}
	}

	// Prepare tracking
	{
		if (strcmp(vroom->config->tracking.system, "dtrack") == 0) {
			auto &dtConf = vroom->config->tracking.dtrack;

			vroom->dtrack = new VroomDTrack(dtConf.port);
			vroom->dtrack->headBody = dtConf.bodies.head;
			vroom->dtrack->leftBody = dtConf.bodies.left;
			vroom->dtrack->rightBody = dtConf.bodies.right;

			vroom->update_pose = dtrack_update_pose;

			pthread_create(&vroom->dtrack_thread, nullptr, vroom_dtrack_run, vroom->dtrack);
		}
		if (strcmp(vroom->config->tracking.system, "vrpn") == 0) {
			auto &vrpnConf = vroom->config->tracking.vrpn;

			vroom->vrpn = new VroomVRPN();

			// Trackers
			vroom->vrpn->headTrackerName = vrpnConf.trackers.head.tracker;
			vroom->vrpn->headTrackerSensor = vrpnConf.trackers.head.sensor;

			vroom->vrpn->leftHandTrackerName = vrpnConf.trackers.left.tracker;
			vroom->vrpn->leftHandTrackerSensor = vrpnConf.trackers.left.sensor;

			vroom->vrpn->rightHandTrackerName = vrpnConf.trackers.right.tracker;
			vroom->vrpn->rightHandTrackerSensor = vrpnConf.trackers.right.sensor;

			// Space correction
			vroom->vrpn->space_correction.pos = vrpnConf.space_correction.pos;

			vroom->vrpn->space_correction.rot = {
				(float)(vrpnConf.space_correction.rot.x * M_PI / 180.f),
				(float)(vrpnConf.space_correction.rot.y * M_PI / 180.f),
				(float)(vrpnConf.space_correction.rot.z * M_PI / 180.f),
			};

			vroom->vrpn->space_correction.mirrorX = vrpnConf.space_correction.mirror.x;
			vroom->vrpn->space_correction.mirrorY = vrpnConf.space_correction.mirror.y;
			vroom->vrpn->space_correction.mirrorZ = vrpnConf.space_correction.mirror.z;

			// Pose retrieval
			vroom->update_pose = vrpn_update_pose;

			pthread_create(&vroom->vrpn_thread, NULL, vroom_vrpn_run, vroom->vrpn);
		}
	}

	// Setup controllers
	{
		if (strcmp(vroom->config->controller_type, "joycon") == 0) {
			vroom->joycon_left = joycon_controller_create(vroom, JS_TYPE_JOYCON_LEFT);
			vroom->joycon_left->update_pose = vroom->update_pose;

			vroom->joycon_right = joycon_controller_create(vroom, JS_TYPE_JOYCON_RIGHT);
			vroom->joycon_right->update_pose = vroom->update_pose;

			vroom_setup_joycons(vroom);
		}
	}

	vroom->base.supported.orientation_tracking = true;
	vroom->base.supported.position_tracking = true;

	vroom->created_ns = os_monotonic_get_ns();

	vroom->frame_count = 0;
	vroom->enable_3d = false;
	vroom->ipd = 0.062f;

	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&vroom->base);

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(vroom, "VROOM Device", true);
	u_var_add_pose(vroom, &vroom->pose, "pose");
	u_var_add_log_level(vroom, &vroom->log_level, "log_level");

	// Open info window
	pthread_create(&debugThread, nullptr, (void *(*)(void *))vroom_debug_window, vroom);

	// Initialize position
	vroom->pose.position.x = 0.00f;
	vroom->pose.position.y = 1.00f;
	vroom->pose.position.z = 1.00f;

	return &vroom->base;
}