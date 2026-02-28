// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to RayNeo driver.
 * @author ChatGPT 5.3
 * @ingroup drv_rayneo
 */

#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USB identifiers used by supported RayNeo glasses. */
#define RAYNEO_VID 0x1bbb
#define RAYNEO_PID 0xaf50

typedef struct cJSON cJSON;
struct xrt_prober;
struct xrt_prober_device;
struct xrt_device;

/*!
 * Probing function for RayNeo AR Glasses.
 *
 * @ingroup drv_rayneo
 * @see xrt_prober_found_func_t
 */
int
rayneo_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t device_count,
             size_t index,
             cJSON *attached_data,
             struct xrt_device **out_xdev);

#ifdef __cplusplus
}
#endif
