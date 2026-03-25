// Copyright 2026, LunaticCat.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Cosmos related code for Libsurvive.
 * @author LunaticCat <lunareredwood@gmail.com>
 * @ingroup drv_survive
 */

#include "xrt/xrt_prober.h"

#include "survive_cosmos.h"


bool
survive_cosmos_init(struct survive_device *survive, struct survive_system *sys)
{
	struct xrt_prober_device **devices = NULL;
	size_t device_count = 0;
	xrt_result_t result = xrt_prober_lock_list(sys->xp, &devices, &device_count);
	if (result != XRT_SUCCESS) {
		SURVIVE_ERROR(survive, "Failed to lock prober devices, cannot check for Cosmos Elite, got err %d",
		              result);
		return false;
	}

	for (size_t i = 0; i < device_count; i++) {
		struct xrt_prober_device *d = devices[i];
		if (d->vendor_id == COSMOS_VID && d->product_id == COSMOS_PID) {
			SURVIVE_DEBUG(survive, "Found Cosmos Elite device in USB list");
			struct os_hid_device *hid;

			result = xrt_prober_open_hid_interface(sys->xp, d, 0, &hid);
			if (result != XRT_SUCCESS) {
				SURVIVE_ERROR(survive, "Failed to open Cosmos Elite HID interface, got err %d", result);
				survive->hmd.cosmos_hid = NULL;
				continue;
			}

			int ret = cosmos_hid_open(hid, &survive->hmd.cosmos_hid);
			if (ret < 0) {
				SURVIVE_ERROR(survive, "Failed to create Cosmos Elite HID interface, got err %d", ret);
				continue;
			}

			break;
		}
	}

	result = xrt_prober_unlock_list(sys->xp, &devices);
	if (result != XRT_SUCCESS) {
		SURVIVE_ERROR(survive, "Failed to unlock prober devices after checking for Cosmos Elite, got err %d",
		              result);
		return false;
	}

	if (survive->hmd.cosmos_hid == NULL) {
		SURVIVE_WARN(survive,
		             "No Cosmos Elite device found. This is non-fatal, but some features may not work.");
	} else {
		SURVIVE_DEBUG(survive, "Sent display power-on command.");
	}

	return true;
}

void
survive_cosmos_setup_hmd(struct survive_device *survive)
{
	int width, height;
	cosmos_resolution_get_extents(&width, &height);

	survive->base.hmd->screens[0].nominal_frame_interval_ns =
	    (uint64_t)time_s_to_ns(1.0f / cosmos_resolution_get_refresh_rate());

	for (int i = 0; i < 2; i++) {
		survive->base.hmd->views[i].display.w_pixels = width / 2;
		survive->base.hmd->views[i].display.h_pixels = height;

		survive->base.hmd->views[i].viewport.w_pixels = width / 2;
		survive->base.hmd->views[i].viewport.h_pixels = height;
	}

	survive->base.hmd->views[1].viewport.x_pixels = width / 2;

	survive->base.hmd->screens[0].w_pixels = width;
	survive->base.hmd->screens[0].h_pixels = height;

	survive->base.supported.brightness_control = true;
}

void
survive_cosmos_teardown(struct survive_device *survive)
{
	if (survive->hmd.cosmos_hid != NULL) {
		cosmos_hid_destroy(survive->hmd.cosmos_hid);
		survive->hmd.cosmos_hid = NULL;
	}
}

xrt_result_t
survive_cosmos_compute_distortion(
    struct survive_device *survive, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct cosmos_config *config = cosmos_get_config(survive->hmd.cosmos_hid);

	struct xrt_vec2 uv = {.x = u, .y = v};

	// Try COSMOS distortion first, fallback if fails.
	if (htc_config_compute_distortion(&config->base, view, &uv, result)) {
		return XRT_SUCCESS;
	}

	return XRT_ERROR_FEATURE_NOT_SUPPORTED;
}