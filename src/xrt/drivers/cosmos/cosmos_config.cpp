// Copyright 2026, LunaticCat.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Code to parse and handle the Cosmos configuration data.
 * @author LunaticCat <lunareredwood@gmail.com>
 * @ingroup drv_cosmos
 */

#include "util/u_json.hpp"
#include "util/u_logging.h"

#include "math/m_api.h"

#include "cosmos_config.h"

#include <string.h>



using xrt::auxiliary::util::json::JSONNode;

bool
cosmos_config_parse(const char *config_data, size_t config_size, cosmos_config *out_config)
{
	memset(out_config, 0, sizeof(*out_config));

	JSONNode root{std::string{config_data, config_size}};
	if (root.isInvalid()) {
		U_LOG_E("Failed to parse JSON config data.");
		return false;
	}

	if (!htc_config_parse(config_data, config_size, &out_config->base)) {
		return false;
	}

	return true;
}