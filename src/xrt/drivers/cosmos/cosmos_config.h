// Copyright 2026, LunaticCat.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Code to parse and handle the Cosmos configuration data.
 * @author LunaticCat <lunareredwood@gmail.com>
 * @ingroup drv_cosmos
 */

#pragma once

#include "xrt/xrt_defines.h"

#include "htc/htc_config.h"

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif
struct cosmos_config
{
	struct htc_config base;

	uint32_t direct_mode_edid_pid;
	uint32_t direct_mode_edid_vid;
};

bool
cosmos_config_parse(const char *config_data, size_t config_size, struct cosmos_config *out_config);

#ifdef __cplusplus
}
#endif
