// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds Metal swapchain related functions.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup oxr_main
 * @ingroup comp_client
 */

#include <stdlib.h>

#include "util/u_debug.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_swapchain_common.h"


static XrResult
metal_enumerate_images(struct oxr_logger *log,
                       struct oxr_swapchain *sc,
                       uint32_t count,
                       XrSwapchainImageBaseHeader *images)
{
	struct xrt_swapchain_metal *xscm = (struct xrt_swapchain_metal *)sc->swapchain;
	XrSwapchainImageMetalKHR *metal_imgs = (XrSwapchainImageMetalKHR *)images;

	for (uint32_t i = 0; i < count; i++) {
		metal_imgs[i].texture = xscm->images[i];
	}

	return oxr_session_success_result(sc->sess);
}

XrResult
oxr_swapchain_metal_create(struct oxr_logger *log,
                           struct oxr_session *sess,
                           const XrSwapchainCreateInfo *createInfo,
                           struct oxr_swapchain **out_swapchain)
{
	struct oxr_swapchain *sc;
	XrResult ret;

	ret = oxr_swapchain_common_create(log, sess, createInfo, &sc);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	// Set our API specific function(s).
	sc->enumerate_images = metal_enumerate_images;

	*out_swapchain = sc;

	return XR_SUCCESS;
}
