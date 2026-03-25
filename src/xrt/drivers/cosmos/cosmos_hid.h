// Copyright 2026, LunaticCat.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the Cosmos HID interface.
 * @author LunaticCat <lunareredwood@gmail.com>
 * @ingroup drv_cosmos
 */

#pragma once


#include <assert.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif


#define COSMOS_VID 0x0bb4
#define COSMOS_PID 0x0313

enum cosmos_resolution
{
	COSMOS_RESOLUTION_2880_1700_90 = 0,
};

static inline void
cosmos_resolution_get_extents(int *out_w, int *out_h)
{
	*out_w = 2880;
	*out_h = 1700;
}

static inline double
cosmos_resolution_get_refresh_rate(void)
{
	return 90.0;
}

struct cosmos_hid;

int
cosmos_hid_open(struct os_hid_device *hid_dev, struct cosmos_hid **out_hid);

enum cosmos_resolution
cosmos_get_resolution(struct cosmos_hid *cosmos);

struct cosmos_config *
cosmos_get_config(struct cosmos_hid *cosmos);

void
cosmos_hid_destroy(struct cosmos_hid *cosmos);

bool
cosmos_hid_set_display_power(struct cosmos_hid *cosmos, bool power_on);


#ifdef __cplusplus
} // extern "C"
#endif