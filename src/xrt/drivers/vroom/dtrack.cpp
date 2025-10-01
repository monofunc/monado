// Copyright 2022-2025, INSA Rouen Normandie
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  DTrack tracking code.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#include "vroom_device.h"
#include "dtrack.hpp"
#include "os/os_threading.h"
#include <iostream>

VroomDTrack::VroomDTrack(uint16_t port)
{
	dtrack = new DTrackSDK(port);
	dtrack->setDataTimeoutUS(33'000); // 33 milliseconds - 30 Hz
}

bool
VroomDTrack::receive()
{
	auto hasReceived = dtrack->receive();

	return hasReceived;
}

VroomDTrack::~VroomDTrack()
{
	delete dtrack;
}

static bool running = true;

void *
vroom_dtrack_run(void *ptr)
{
	running = true;

	auto *cdt = reinterpret_cast<VroomDTrack *>(ptr);

	while (running) {
		cdt->receive();
	}

	return EXIT_SUCCESS;
}

void
vroom_dtrack_stop()
{
	running = false;
}

void
dtrack_update_pose(const vroom_device *vroom, const int body, xrt_pose *out)
{
	const auto *dt = vroom->dtrack->dtrack;
	const auto &dtConf = vroom->config->tracking.dtrack.bodies;
	int dtBody = -1;

	switch (body) {
	// head
	case 0: dtBody = dtConf.head; break;
	// left hand
	case 1: dtBody = dtConf.left; break;
	// right hand
	case 2: dtBody = dtConf.right; break;
	default: return;
	}

	if (dt == nullptr)
		return;

	const auto b = dt->getBody(dtBody);
	if (b == nullptr)
		return;

	out->position.x = static_cast<float>(b->loc[0] * 0.001f);
	out->position.y = static_cast<float>(b->loc[1] * 0.001f);
	out->position.z = static_cast<float>(b->loc[2] * 0.001f);

	const auto quat = b->getQuaternion();
	out->orientation.x = static_cast<float>(quat.x);
	out->orientation.y = static_cast<float>(quat.y);
	out->orientation.z = static_cast<float>(quat.z);
	out->orientation.w = static_cast<float>(quat.w);
}
