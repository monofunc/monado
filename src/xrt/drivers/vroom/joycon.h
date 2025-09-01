/*!
 * @file
 * @brief Nintendo Joy-Con controller interface
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 *
 * @ingroup drv_vroom
 */

#pragma once

#include "xrt/xrt_device.h"
#include "vroom_device.h"

#include <os/os_threading.h>

struct joycon_controller
{
	struct xrt_device base;
	struct xrt_pose pose;

	enum u_logging_level log_level;

	struct vroom_device *sys; ///< System this controller belongs to.

	bool handle_set;
	int handle; ///< JoyShockLibrary handle
	int type;

	void (*update_pose)(const vroom_device *vroom, const int body, xrt_pose *out);
};

struct joycon_controller *
joycon_controller_create(struct vroom_device *sys, int jsl_type);

bool
joycon_controller_set_handle(struct joycon_controller *jc, int handle);

bool
joycon_controller_get_handle(struct joycon_controller *jc, int *out_handle);