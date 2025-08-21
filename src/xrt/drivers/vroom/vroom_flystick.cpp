/*!
 * @file
 * @brief  DTrack Flystick Controller
 *
 * Handles communication and calibration information for the DTrack Controller for the CAVE
 *
 * Ported from OpenHMD
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */


#include <cstdio>
#include <cassert>
#include <cinttypes>

#include "math/m_api.h"

#include "util/u_device.h"
#include "util/u_var.h"

#include "vroom_device.h"
#include "vroom_flystick.h"

/* STRUCTS AND OTHERS */


// Input bindings for the D_Track controller
static struct xrt_binding_input_pair simple_inputs_dtrack[13] = {
	{XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_DTRACK_CONTROLLER_TRIGGER_CLICK},
	{XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_DTRACK_CONTROLLER_THUMBSTICK_CLICK},

	{XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_DTRACK_CONTROLLER_GRIP_POSE},
	{XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_DTRACK_CONTROLLER_AIM_POSE},


	{XRT_INPUT_TOUCH_X_CLICK, XRT_INPUT_DTRACK_CONTROLLER_1_CLICK},
	{XRT_INPUT_TOUCH_X_TOUCH, XRT_INPUT_DTRACK_CONTROLLER_2_CLICK},
	{XRT_INPUT_TOUCH_Y_CLICK, XRT_INPUT_DTRACK_CONTROLLER_3_CLICK},
	{XRT_INPUT_TOUCH_Y_TOUCH, XRT_INPUT_DTRACK_CONTROLLER_4_CLICK},

	{XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_DTRACK_CONTROLLER_TRIGGER_CLICK},
	{XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_DTRACK_CONTROLLER_THUMBSTICK_CLICK},

	{XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_DTRACK_CONTROLLER_THUMBSTICK},


	{XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_DTRACK_CONTROLLER_GRIP_POSE},
	{XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_DTRACK_CONTROLLER_AIM_POSE},


};

static struct xrt_binding_profile binding_profiles_dtrack[1] = {
	{
		.name = XRT_DEVICE_TOUCH_CONTROLLER,
		.inputs = simple_inputs_dtrack,
		.input_count = ARRAY_SIZE(simple_inputs_dtrack),
		.outputs = nullptr,
		.output_count = 0,
	},
};

/* */

/// Casting helper function
static inline struct dtrack_flystick_controller *
dtrack_flystick_controller(struct xrt_device *xdev)
{
	return (struct dtrack_flystick_controller *)xdev;
}

static void
dtrack_flystick_controller_destroy(struct xrt_device *xdev)
{
	struct dtrack_flystick_controller *fs = dtrack_flystick_controller(xdev);

	// Remove the variable tracking.
	u_var_remove_root(fs);

	// Destroy the thread
	os_mutex_destroy(&fs->mutex);

	u_device_free(&fs->base);
}

static void
dtrack_flystick_controller_update_input_click(struct dtrack_flystick_controller *fs,
                                              int index,
                                              int64_t when_ns,
                                              int val)
{
	fs->base.inputs[index].timestamp = when_ns;
	fs->base.inputs[index].value.boolean = (val != 0);

	/*if (sh->base.inputs[index].value.boolean) {
	        printf("Button %d pressed. \n", index);
	}*/
}

static xrt_result_t
dtrack_flystick_controller_update_inputs(struct xrt_device *xdev)
{
	struct dtrack_flystick_controller *fs = dtrack_flystick_controller(xdev);

	os_mutex_lock(&fs->mutex);

	auto dt = fs->dtrack;
	int64_t last_ns = os_monotonic_get_ns();

	/*if (dt->flystickVisible) {
		fs->base.inputs[DTRACK_TRIGGER].timestamp = last_ns;
		fs->base.inputs[DTRACK_TRIGGER].value.vec1.x = (float)dt->flystickButtons[0];

		dtrack_flystick_controller_update_input_click(fs, DTRACK_1, last_ns, dt->flystickButtons[1]);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_2, last_ns, dt->flystickButtons[2]);

		dtrack_flystick_controller_update_input_click(fs, DTRACK_3, last_ns, dt->flystickButtons[3]);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_4, last_ns, dt->flystickButtons[4]);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_THUMBSTICK_CLICK, last_ns,
		                                              dt->flystickButtons[5]);

		fs->base.inputs[DTRACK_THUMBSTICK].timestamp = last_ns;
		fs->base.inputs[DTRACK_THUMBSTICK].value.vec2.x = (float)dt->flystickAnalog[0];
		fs->base.inputs[DTRACK_THUMBSTICK].value.vec2.y = (float)dt->flystickAnalog[1];
	} else {
		// Reset inputs to neutral when the FlyStick is not visible
		dtrack_flystick_controller_update_input_click(fs, DTRACK_TRIGGER, last_ns, 0);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_1, last_ns, 0);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_2, last_ns, 0);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_3, last_ns, 0);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_4, last_ns, 0);
		dtrack_flystick_controller_update_input_click(fs, DTRACK_THUMBSTICK_CLICK, last_ns, 0);

		fs->base.inputs[DTRACK_THUMBSTICK].value.vec2.x = 0.0f;
		fs->base.inputs[DTRACK_THUMBSTICK].value.vec2.y = 0.0f;
	}*/

	os_mutex_unlock(&fs->mutex);

	return XRT_SUCCESS;
}

static xrt_result_t
dtrack_flystick_controller_get_tracked_pose(struct xrt_device *xdev,
                                            enum xrt_input_name name,
                                            int64_t at_timestamp_ns,
                                            struct xrt_space_relation *out_relation)
{

	struct dtrack_flystick_controller *fs = dtrack_flystick_controller(xdev);
	auto dt = fs->dtrack;

	auto new_pose = fs->pose;

	/*if (/*dt->isTracking && dt->flystickVisible#1# false) {
		new_pose.position.x = (float)dt->flystickPos[0] * .001f;
		new_pose.position.y = (float)dt->flystickPos[1] * .001f;
		new_pose.position.z = (float)dt->flystickPos[2] * .001f;

		new_pose.orientation.w = (float)dt->flystickQuat.w;
		new_pose.orientation.x = (float)dt->flystickQuat.x;
		new_pose.orientation.y = (float)dt->flystickQuat.y;
		new_pose.orientation.z = (float)dt->flystickQuat.z;
	}*/

	fs->pose = new_pose;


	// Estimate pose at timestamp at_timestamp_ns!
	math_quat_normalize(&fs->pose.orientation);
	out_relation->pose = fs->pose;

	// Why?
	out_relation->pose.position.x += 0.2f;
	out_relation->pose.position.y -= 1.3f;
	out_relation->pose.position.z += 0.5f;

	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	return XRT_SUCCESS;
}


struct dtrack_flystick_controller *
dtrack_flystick_controller_create(struct vroom_device *sys, VroomDTrack *dtrack)
{

	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
		(enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct dtrack_flystick_controller *sh = U_DEVICE_ALLOCATE(struct dtrack_flystick_controller, flags, 10, 0);

	// This list should be ordered, most preferred first.
	size_t idx = 0;
	sh->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	sh->base.hmd->blend_mode_count = idx;

	sh->base.update_inputs = dtrack_flystick_controller_update_inputs;
	sh->base.get_tracked_pose = dtrack_flystick_controller_get_tracked_pose;
	sh->base.destroy = dtrack_flystick_controller_destroy;
	// sh->base.

	sh->pose = XRT_POSE_IDENTITY;

	// Link to the VROOM and DTrack SDK to use
	sh->sys = sys;
	sh->dtrack = dtrack;

	// Print name.
	snprintf(sh->base.str, XRT_DEVICE_NAME_LEN, "Flystick Controller");
	snprintf(sh->base.serial, XRT_DEVICE_NAME_LEN, "0123456789abcdef");

	// Setup input.
	sh->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	sh->base.device_type = XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER;

	// sh->base.inputs[0].name = XRT_INPUT_GENERIC_HAND_TRACKING_LEFT;

	sh->base.inputs[DTRACK_1].name = XRT_INPUT_DTRACK_CONTROLLER_1_CLICK;
	sh->base.inputs[DTRACK_2].name = XRT_INPUT_DTRACK_CONTROLLER_2_CLICK;
	sh->base.inputs[DTRACK_3].name = XRT_INPUT_DTRACK_CONTROLLER_3_CLICK;
	sh->base.inputs[DTRACK_4].name = XRT_INPUT_DTRACK_CONTROLLER_4_CLICK;

	sh->base.inputs[DTRACK_TRIGGER].name = XRT_INPUT_DTRACK_CONTROLLER_TRIGGER_CLICK;
	sh->base.inputs[DTRACK_THUMBSTICK_CLICK].name = XRT_INPUT_DTRACK_CONTROLLER_THUMBSTICK_CLICK;
	sh->base.inputs[DTRACK_THUMBSTICK].name = XRT_INPUT_DTRACK_CONTROLLER_THUMBSTICK;

	sh->base.inputs[DTRACK_GRIP_POSE].name = XRT_INPUT_DTRACK_CONTROLLER_GRIP_POSE;
	sh->base.inputs[DTRACK_AIM_POSE].name = XRT_INPUT_DTRACK_CONTROLLER_AIM_POSE;

	sh->base.binding_profiles = binding_profiles_dtrack;
	sh->base.binding_profile_count = ARRAY_SIZE(binding_profiles_dtrack);

	sh->base.supported.orientation_tracking = true;
	sh->base.supported.position_tracking = true;

	sh->created_ns = os_monotonic_get_ns();

	sh->frame_count = 0;

	os_mutex_init(&sh->mutex);

	return sh;
}
