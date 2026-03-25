// Copyright 2026, LunaticCat.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Cosmos related code for Libsurvive.
 * @author LunaticCat <lunareredwood@gmail.com>
 * @ingroup drv_survive
 */

#pragma once

#include "survive_internal.h"


bool
survive_cosmos_init(struct survive_device *survive, struct survive_system *sys);

void
survive_cosmos_setup_hmd(struct survive_device *survive);

void
survive_cosmos_teardown(struct survive_device *survive);

xrt_result_t
survive_cosmos_compute_distortion(
    struct survive_device *survive, uint32_t view, float u, float v, struct xrt_uv_triplet *result);

xrt_result_t
survive_cosmos_set_brightness(struct survive_device *survive, float brightness, bool relative);
