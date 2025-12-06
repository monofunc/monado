// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds Metal specific session functions.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup oxr_main
 * @ingroup comp_client
 */

#include <stdlib.h>

#include "util/u_misc.h"

#include "xrt/xrt_instance.h"
#include "xrt/xrt_gfx_metal.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_two_call.h"
#include "oxr_handle.h"


XrResult
oxr_session_populate_metal(struct oxr_logger *log,
                           struct oxr_system *sys,
                           XrGraphicsBindingMetalKHR const *next,
                           struct oxr_session *sess)
{
	struct xrt_compositor_native *xcn = sess->xcn;
	struct xrt_compositor_metal *xcm = xrt_gfx_metal_provider_create( //
	    xcn,                                                          //
	    next->commandQueue);                                          //

	if (xcm == NULL) {
		return oxr_error(log, XR_ERROR_INITIALIZATION_FAILED, "Failed to create a Metal client compositor");
	}

	sess->compositor = &xcm->base;
	sess->create_swapchain = oxr_swapchain_metal_create;

	return XR_SUCCESS;
}
