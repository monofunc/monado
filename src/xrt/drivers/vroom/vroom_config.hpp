/*!
 * @file
 * @brief VROOM configuration file interface
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 *
 * @ingroup drv_vroom
 */


#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vroom_display
{
	char *name;

	xrt_vec3 position;
	xrt_vec3 rotation;
	xrt_vec2 dimensions;

	xrt_vec2_i32 screenPosition;
	xrt_vec2_i32 screenSize;

	xrt_vec2_i32 textureSize;
};

struct vrpn_tracker
{
	char *tracker;
	int sensor = 0;
};

struct dtrack_config
{
	uint16_t port;

	struct
	{
		int head;
		int left;
		int right;
	} bodies;
};

struct vrpn_config
{
	struct
	{
		vrpn_tracker head;
		vrpn_tracker left;
		vrpn_tracker right;
	} trackers;

	struct
	{
		xrt_vec3 pos;
		xrt_vec3 rot;
		struct
		{
			bool x;
			bool y;
			bool z;
		} mirror;
	} space_correction;
};

struct vroom_config
{
	// Displays
	bool stereo;

	size_t display_count;
	vroom_display *displays;

	// Controllers
	char* controller_type;

	struct
	{
		char* system;

		dtrack_config dtrack;
		vrpn_config vrpn;
	} tracking;
};


struct vroom_config *
vroom_load_config();

vroom_config
vroom_create_config();

void
vroom_save_config(const vroom_config* config, const char* filepath);


#ifdef __cplusplus
}
#endif