/*!
 * @file
 * @brief  DTrack Flystick Controller interface
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#ifndef VROOM_FLYSTICK_H
#define VROOM_FLYSTICK_H

#include "os/os_time.h"
#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "DTrackSDK.hpp"
#include "vroom_device.h"
#include "vroom_interface.h"

#include <os/os_threading.h>

enum vroom_buttons_index
{
	DTRACK_1 = 1,
	DTRACK_2 = 2,
	DTRACK_3 = 3,
	DTRACK_4 = 4,
	DTRACK_TRIGGER = 5,
	DTRACK_THUMBSTICK_CLICK = 6,
	DTRACK_THUMBSTICK = 7,

	DTRACK_GRIP_POSE = 8,
	DTRACK_AIM_POSE = 9,
};

struct dtrack_flystick_controller
{
	struct xrt_device base;

	struct os_mutex mutex;

	struct xrt_pose pose;

	/* The system this controller belongs to / receives reports from */
	struct vroom_device *sys;

	// Array of inputs
	int buttons[DTRACK_FLYSTICK_BUTTONS];

	uint64_t created_ns;
	uint64_t frame_count;

	uint64_t device_id;

	VroomDTrack *dtrack;
};

struct dtrack_flystick_controller *
dtrack_flystick_controller_create(struct vroom_device *sys, VroomDTrack *dtrack);

#endif
