// Copyright 2022-2025, INSA Rouen Normandie
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Interface for CAVE driver.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */


#pragma once

#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "vroom_interface.h"
#include "vroom_config.hpp"
#include "dtrack.hpp"
#include "vrpn.hpp"

#include <os/os_threading.h>

typedef struct
{
	bool tracking;
	bool head_visible;
	bool flystick_visible;
} tracking_status;

struct vroom_device
{
	struct xrt_device base;
	struct xrt_pose pose;

	struct os_mutex mutex;

	// Static configuration
	struct vroom_config *config;
	xrt_pose *display_transforms;

	// Runtime configuration
	int64_t created_ns;
	int64_t frame_count;
	bool enable_3d;
	bool invert_eyes;
	float ipd;

	void (*update_pose)(const vroom_device *vroom, const int body, xrt_pose *out);

	enum u_logging_level log_level;

	/* The Dtrack controller the CAVE has. */
	VroomDTrack *dtrack;
	pthread_t dtrack_thread;

	/* VRPN client */
	VroomVRPN *vrpn;
	pthread_t vrpn_thread;

	/* Controllers */
	struct dtrack_flystick_controller *flystick; ///< ART FlyStick 2 controller - inputs and tracking from DTRACK2
	struct joycon_controller *joycon_left;       ///< Nintendo Joy-Con (L) - inputs from the Joy-Con
	struct joycon_controller *joycon_right;      ///< Nintendo Joy-Con (R) - inputs from the Joy-Con
};

void
vroom_setup_joycons(struct vroom_device *vroom);
