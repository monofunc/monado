// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Glue code to Metal client side code.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup comp_client
 */

#include "xrt/xrt_gfx_metal.h"

#import <Metal/Metal.h>


void
xrt_gfx_metal_get_device(void **out_ptr)
{
	// Get the default Metal device for this system.
	// MTLCreateSystemDefaultDevice returns a retained device.
	// The caller receives ownership of this reference.
	id<MTLDevice> device = MTLCreateSystemDefaultDevice();

	*out_ptr = (void *)device;
}
