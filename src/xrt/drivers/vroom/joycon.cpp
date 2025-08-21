/*!
 * @file
 * @brief Nintendo Joy-Con
 *
 * Handles communication and calibration information for Joy-Cons in use in a CAVE
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#include "joycon.h"

#include <cstdio>
#include <cassert>

#include "math/m_api.h"

#include "util/u_device.h"

#include "vroom_device.h"

#include "JoyShockLibrary.h"

#define JC_TRACE(p, ...) U_LOG_XDEV_IFL_T(&jc->base, jc->log_level, __VA_ARGS__)
#define JC_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&jc->base, jc->log_level, __VA_ARGS__)
#define JC_WARN(p, ...) U_LOG_XDEV_IFL_W(&jc->base, jc->log_level, __VA_ARGS__)
#define JC_ERROR(p, ...) U_LOG_XDEV_IFL_E(&jc->base, jc->log_level, __VA_ARGS__)

/* STRUCTS & ENUMS */

// Button enums
enum joycon_button_index
{
	SHOULDER = 1,
	TRIGGER = 2,
	NORTH = 3,
	WEST = 4,
	SOUTH = 5,
	EAST = 6,
	SYSTEM = 7,
	SQUEEZE = 8,
	THUMBSTICK_CLICK = 9,
	THUMBSTICK = 10,
	GRIP_POSE = 11,
	AIM_POSE = 12
};

// Input mapping -- Oculus Touch
static struct xrt_binding_input_pair left_joycon_mapping[12] = {
	{XRT_INPUT_TOUCH_Y_TOUCH, XRT_INPUT_JOY_CON_UP_CLICK},
	{XRT_INPUT_TOUCH_Y_CLICK, XRT_INPUT_JOY_CON_RIGHT_CLICK},
	{XRT_INPUT_TOUCH_X_TOUCH, XRT_INPUT_JOY_CON_LEFT_CLICK},
	{XRT_INPUT_TOUCH_X_CLICK, XRT_INPUT_JOY_CON_DOWN_CLICK},
	{XRT_INPUT_TOUCH_MENU_CLICK, XRT_INPUT_JOY_CON_CAPTURE_CLICK},

	{XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_JOY_CON_ZL_CLICK},
	{XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_JOY_CON_L_CLICK},
	{XRT_INPUT_TOUCH_SQUEEZE_VALUE, XRT_INPUT_JOY_CON_SR_CLICK},
	{XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_JOY_CON_THUMBSTICK_CLICK},
	{XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_JOY_CON_THUMBSTICK},
	{XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_JOY_CON_GRIP_POSE},
	{XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_JOY_CON_AIM_POSE},
};
static struct xrt_binding_profile left_joycon_profile[1] = {
	{
		.name = XRT_DEVICE_TOUCH_CONTROLLER,
		.inputs = left_joycon_mapping,
		.input_count = ARRAY_SIZE(left_joycon_mapping),
		.outputs = nullptr,
		.output_count = 0,
	},
};

static struct xrt_binding_input_pair right_joycon_mapping[12] = {
	{XRT_INPUT_TOUCH_A_TOUCH, XRT_INPUT_JOY_CON_A_CLICK},
	{XRT_INPUT_TOUCH_A_CLICK, XRT_INPUT_JOY_CON_B_CLICK},
	{XRT_INPUT_TOUCH_B_TOUCH, XRT_INPUT_JOY_CON_X_CLICK},
	{XRT_INPUT_TOUCH_B_CLICK, XRT_INPUT_JOY_CON_Y_CLICK},
	{XRT_INPUT_TOUCH_SYSTEM_CLICK, XRT_INPUT_JOY_CON_HOME_CLICK},

	{XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_JOY_CON_ZR_CLICK},
	{XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_JOY_CON_R_CLICK},
	{XRT_INPUT_TOUCH_SQUEEZE_VALUE, XRT_INPUT_JOY_CON_SL_CLICK},
	{XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_JOY_CON_THUMBSTICK_CLICK},
	{XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_JOY_CON_THUMBSTICK},
	{XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_JOY_CON_GRIP_POSE},
	{XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_JOY_CON_AIM_POSE},
};
static struct xrt_binding_profile right_joycon_profile[1] = {
	{
		.name = XRT_DEVICE_TOUCH_CONTROLLER,
		.inputs = right_joycon_mapping,
		.input_count = ARRAY_SIZE(right_joycon_mapping),
		.outputs = nullptr,
		.output_count = 0,
	},
};

/// Casting helper function.
static inline struct joycon_controller *
joycon_controller(struct xrt_device *xdev)
{
	return (struct joycon_controller *)xdev;
}

static void
jc_update_input(struct joycon_controller *jc,
                int index,
                int64_t when_ns,
                int val)
{
	jc->base.inputs[index].timestamp = when_ns;
	jc->base.inputs[index].value.boolean = (val != 0);
}

static xrt_result_t
joycon_controller_update_inputs(struct xrt_device *xdev)
{
	struct joycon_controller *jc = joycon_controller(xdev);

	// No handle was set. Can't do anything
	if (!jc->handle_set)
		return XRT_SUCCESS;

	// Controller isn't connected. No need to bother with the rest
	if (!JslStillConnected(jc->handle))
		return XRT_SUCCESS;

	int64_t last_ns = os_monotonic_get_ns();

	struct JOY_SHOCK_STATE inputs = JslGetSimpleState(jc->handle);

	switch (jc->type) {
	case JS_TYPE_JOYCON_LEFT: {
		jc_update_input(jc, TRIGGER, last_ns, inputs.buttons & JSMASK_ZL);
		jc_update_input(jc, NORTH, last_ns, inputs.buttons & JSMASK_UP);
		jc_update_input(jc, WEST, last_ns, inputs.buttons & JSMASK_LEFT);
		jc_update_input(jc, SOUTH, last_ns, inputs.buttons & JSMASK_DOWN);
		jc_update_input(jc, EAST, last_ns, inputs.buttons & JSMASK_RIGHT);
		jc_update_input(jc, SYSTEM, last_ns, inputs.buttons & JSMASK_CAPTURE);
		jc_update_input(jc, SQUEEZE, last_ns, inputs.buttons & JSMASK_L);
		jc_update_input(jc, THUMBSTICK_CLICK, last_ns, inputs.buttons & JSMASK_LCLICK);

		jc->base.inputs[THUMBSTICK].timestamp = last_ns;
		jc->base.inputs[THUMBSTICK].value.vec2.x = inputs.stickLX;
		jc->base.inputs[THUMBSTICK].value.vec2.y = inputs.stickLY;
		break;
	}
	case JS_TYPE_JOYCON_RIGHT: {
		jc_update_input(jc, TRIGGER, last_ns, inputs.buttons & JSMASK_ZR);
		jc_update_input(jc, NORTH, last_ns, inputs.buttons & JSMASK_N);
		jc_update_input(jc, WEST, last_ns, inputs.buttons & JSMASK_W);
		jc_update_input(jc, SOUTH, last_ns, inputs.buttons & JSMASK_S);
		jc_update_input(jc, EAST, last_ns, inputs.buttons & JSMASK_E);
		jc_update_input(jc, SYSTEM, last_ns, inputs.buttons & JSMASK_HOME);
		jc_update_input(jc, SQUEEZE, last_ns, inputs.buttons & JSMASK_R);
		jc_update_input(jc, THUMBSTICK_CLICK, last_ns, inputs.buttons & JSMASK_RCLICK);

		jc->base.inputs[THUMBSTICK].timestamp = last_ns;
		jc->base.inputs[THUMBSTICK].value.vec2.x = inputs.stickRX;
		jc->base.inputs[THUMBSTICK].value.vec2.y = inputs.stickRY;
		break;
	}
	default: break;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
joycon_controller_get_tracked_pose(struct xrt_device *xdev,
                                   enum xrt_input_name name,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation)
{
	struct joycon_controller *jc = joycon_controller(xdev);

	auto new_pose = jc->pose;

	auto isLeft = jc->type == JS_TYPE_JOYCON_LEFT;

	/* DTRACK */
	if (jc->update_pose) {
		jc->update_pose(jc->sys, isLeft ? 1 : 2, &new_pose);
	} else {

	}

	/* VRPN */
	// new_pose = jc->vrpn->get_corrected_pose(isLeft ? jc->vrpn->leftHandPose : jc->vrpn->rightHandPose);

	math_quat_normalize(&new_pose.orientation);
	jc->pose = new_pose;

	out_relation->pose = jc->pose;

	// Why?
	out_relation->pose.position.x += isLeft ? +0.2f : -0.2f;
	out_relation->pose.position.y -= 1.3f;
	out_relation->pose.position.z += 0.5f;

	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	return XRT_SUCCESS;
}

static void
joycon_controller_destroy(struct xrt_device *xdev)
{
	struct joycon_controller *jc = joycon_controller(xdev);

	u_device_free(&jc->base);
}

struct joycon_controller *
joycon_controller_create(struct vroom_device *sys, int jsl_type)
{
	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);

	struct joycon_controller *jc = U_DEVICE_ALLOCATE(struct joycon_controller, flags, 13, 0);

	// Function pointers
	jc->base.update_inputs = joycon_controller_update_inputs;
	jc->base.get_tracked_pose = joycon_controller_get_tracked_pose;
	jc->base.destroy = joycon_controller_destroy;

	// Initialize pose
	jc->pose = XRT_POSE_IDENTITY;

	// Match log to parent system
	jc->log_level = sys->log_level;

	// Link back
	jc->sys = sys;
	jc->type = jsl_type;

	// Identify controller
	jc->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	switch (jsl_type) {
	case JS_TYPE_JOYCON_LEFT: jc->base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;

		snprintf(jc->base.str, XRT_DEVICE_NAME_LEN, "Nintendo Joy-Con (L)");
		snprintf(jc->base.serial, XRT_DEVICE_NAME_LEN, "HAC-015");

		// jc->base.inputs[0].name = XRT_INPUT_GENERIC_HAND_TRACKING_LEFT;
		jc->base.inputs[SHOULDER].name = XRT_INPUT_JOY_CON_L_CLICK;
		jc->base.inputs[TRIGGER].name = XRT_INPUT_JOY_CON_ZL_CLICK;
		jc->base.inputs[NORTH].name = XRT_INPUT_JOY_CON_UP_CLICK;
		jc->base.inputs[WEST].name = XRT_INPUT_JOY_CON_LEFT_CLICK;
		jc->base.inputs[SOUTH].name = XRT_INPUT_JOY_CON_DOWN_CLICK;
		jc->base.inputs[EAST].name = XRT_INPUT_JOY_CON_RIGHT_CLICK;
		jc->base.inputs[SYSTEM].name = XRT_INPUT_JOY_CON_CAPTURE_CLICK;
		jc->base.inputs[SQUEEZE].name = XRT_INPUT_JOY_CON_SR_CLICK;
		jc->base.inputs[THUMBSTICK_CLICK].name = XRT_INPUT_JOY_CON_THUMBSTICK_CLICK;
		jc->base.inputs[THUMBSTICK].name = XRT_INPUT_JOY_CON_THUMBSTICK;
		jc->base.inputs[GRIP_POSE].name = XRT_INPUT_JOY_CON_GRIP_POSE;
		jc->base.inputs[AIM_POSE].name = XRT_INPUT_JOY_CON_AIM_POSE;

		jc->base.binding_profiles = left_joycon_profile;
		jc->base.binding_profile_count = ARRAY_SIZE(left_joycon_profile);
		break;
	case JS_TYPE_JOYCON_RIGHT: jc->base.device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;

		snprintf(jc->base.str, XRT_DEVICE_NAME_LEN, "Nintendo Joy-Con (R)");
		snprintf(jc->base.serial, XRT_DEVICE_NAME_LEN, "HAC-016");

		// jc->base.inputs[0].name = XRT_INPUT_GENERIC_HAND_TRACKING_RIGHT;
		jc->base.inputs[SHOULDER].name = XRT_INPUT_JOY_CON_R_CLICK;
		jc->base.inputs[TRIGGER].name = XRT_INPUT_JOY_CON_ZR_CLICK;
		jc->base.inputs[NORTH].name = XRT_INPUT_JOY_CON_X_CLICK;
		jc->base.inputs[WEST].name = XRT_INPUT_JOY_CON_Y_CLICK;
		jc->base.inputs[SOUTH].name = XRT_INPUT_JOY_CON_B_CLICK;
		jc->base.inputs[EAST].name = XRT_INPUT_JOY_CON_A_CLICK;
		jc->base.inputs[SYSTEM].name = XRT_INPUT_JOY_CON_HOME_CLICK;
		jc->base.inputs[SQUEEZE].name = XRT_INPUT_JOY_CON_SL_CLICK;
		jc->base.inputs[THUMBSTICK_CLICK].name = XRT_INPUT_JOY_CON_THUMBSTICK_CLICK;
		jc->base.inputs[THUMBSTICK].name = XRT_INPUT_JOY_CON_THUMBSTICK;
		jc->base.inputs[GRIP_POSE].name = XRT_INPUT_JOY_CON_GRIP_POSE;
		jc->base.inputs[AIM_POSE].name = XRT_INPUT_JOY_CON_AIM_POSE;

		jc->base.binding_profiles = right_joycon_profile;
		jc->base.binding_profile_count = ARRAY_SIZE(right_joycon_profile);
		break;
	default:
		// It's not a JoyCon.
		 JC_ERROR(jc, "Type %d is not a recognized Joy-Con type",
		 	jsl_type);
		u_device_free(&jc->base);  // Clean up allocated device
		return nullptr;
	}

	jc->base.supported.orientation_tracking = true;
	jc->base.supported.position_tracking = true;

	return jc;
}

bool
joycon_controller_set_handle(struct joycon_controller *jc, int handle)
{
	if (JslGetControllerType(handle) != jc->type)
		return false;

	jc->handle = handle;
	jc->handle_set = true;
	JslSetAutomaticCalibration(jc->handle, true);

	return true;
}

bool
joycon_controller_get_handle(struct joycon_controller *jc, int *out_handle)
{
	if (!jc->handle_set)
		return false;

	*out_handle = jc->handle;
	return true;
}

