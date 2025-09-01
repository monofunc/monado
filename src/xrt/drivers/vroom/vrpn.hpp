// Copyright 2022-2025, INSA Rouen Normandie
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief VRPN thread interface
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 *
 * @ingroup drv_vroom
 */

#pragma once

#include <vrpn_Tracker.h>
#include "xrt/xrt_defines.h"

struct vroom_device;

class VroomVRPN
{
public:
	vrpn_Tracker_Remote *head;
	vrpn_Tracker_Remote *leftHand;
	vrpn_Tracker_Remote *rightHand;

	// Noms tracker
	const char *headTrackerName;
	int headTrackerSensor = 0;
	const char *leftHandTrackerName;
	int leftHandTrackerSensor = 0;
	const char *rightHandTrackerName;
	int rightHandTrackerSensor = 0;

	// Poses VRPN
	xrt_pose headPose = XRT_POSE_IDENTITY;
	xrt_pose leftHandPose = XRT_POSE_IDENTITY;
	xrt_pose rightHandPose = XRT_POSE_IDENTITY;

	struct
	{
		xrt_vec3 pos;
		xrt_vec3 rot;

		bool mirrorX;
		bool mirrorY;
		bool mirrorZ;
	} space_correction;

	// Constructeur
	VroomVRPN();
	~VroomVRPN();

	bool
	mainLoop();

	xrt_pose
	get_corrected_pose(xrt_pose pose);
};

void *
vroom_vrpn_run(void *ptr);

void
vroom_vrpn_stop();


void
vrpn_update_pose(const vroom_device *vroom, int body, xrt_pose *out);