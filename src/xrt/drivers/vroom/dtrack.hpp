/*!
 * @file
 * @brief Interface for DTrack tracking code.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */
#pragma once

#include "xrt/xrt_defines.h"

#include "DTrackSDK.hpp"


#define DEFAULT_DTRACK_PORT 5000
#define DTRACK_FLYSTICK_BUTTONS 6

class VroomDTrack
{
public:
	DTrackSDK *dtrack = nullptr;

	int headBody = 0;
	int leftBody = 0;
	int rightBody = 0;

	VroomDTrack() : VroomDTrack(DEFAULT_DTRACK_PORT)
	{
	}

	explicit
	VroomDTrack(uint16_t port);

	~VroomDTrack();

	bool
	receive();
};

void *
vroom_dtrack_run(void *ptr);
void
vroom_dtrack_stop();

struct vroom_device;

void
dtrack_update_pose(const vroom_device *vroom, int body, xrt_pose *out);