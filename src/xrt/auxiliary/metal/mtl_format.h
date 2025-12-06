// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal format conversion functions.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_metal
 */

#pragma once

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Convert a Metal pixel format to a Vulkan format.
 *
 * @param format Metal pixel format value.
 * @return Vulkan format value, or 0 if unknown.
 *
 * @ingroup aux_metal
 */
int64_t
mtl_format_to_vk(int64_t format);

/*!
 * Convert a Vulkan format to a Metal pixel format.
 *
 * @param format Vulkan format value.
 * @return Metal pixel format value, or 0 if unknown.
 *
 * @ingroup aux_metal
 */
int64_t
mtl_format_from_vk(int64_t format);


#ifdef __cplusplus
}
#endif
