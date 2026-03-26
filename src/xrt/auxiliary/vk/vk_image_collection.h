// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Vulkan image collection implementing xrt_allocation_collection.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_vk
 */

#pragma once

#include "xrt/xrt_allocation_collection.h"


#ifdef __cplusplus
extern "C" {
#endif

struct vk_bundle;
struct vk_image_collection;
struct xrt_swapchain_create_info;


/*!
 * @addtogroup aux_vk
 * @{
 */

/*!
 * Allocates a new xrt_allocation_collection backed by Vulkan images.
 * Internally calls vk_ic_allocate to create the images.
 *
 * Supports retrieving images via:
 * - XRT_ALLOCATION_TYPE_VULKAN_IMAGE (VkImage handles)
 * - XRT_ALLOCATION_TYPE_NATIVE_IMAGE (xrt_image_native structs)
 *
 * @param vk          Vulkan bundle.
 * @param xscci       Swapchain creation info describing the images.
 * @param image_count Number of images to allocate.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_vk
 */
xrt_result_t
vk_xac_allocate(struct vk_bundle *vk,
                const struct xrt_swapchain_create_info *xscci,
                uint32_t image_count,
                struct xrt_allocation_collection **out_xac);

/*!
 * Creates a new xrt_allocation_collection by importing native image handles.
 * Internally calls vk_ic_from_natives to import the images.
 *
 * Takes ownership of the native handles in the native_images array. On success,
 * the handles are consumed (closed/unreffed) and should not be used by the
 * caller. On failure, the handles remain valid and owned by the caller. This is
 * an all-or-nothing operation.
 *
 * Supports retrieving images via:
 * - XRT_ALLOCATION_TYPE_VULKAN_IMAGE (VkImage handles)
 * - XRT_ALLOCATION_TYPE_NATIVE_IMAGE (xrt_image_native structs)
 *
 * @param vk            Vulkan bundle.
 * @param xscci         Swapchain creation info describing the images.
 * @param native_images Array of native image handles to import (ownership transferred on success).
 * @param image_count   Number of images to import.
 * @param out_xac       Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_vk
 */
xrt_result_t
vk_xac_from_natives(struct vk_bundle *vk,
                    const struct xrt_swapchain_create_info *xscci,
                    struct xrt_image_native *native_images,
                    uint32_t image_count,
                    struct xrt_allocation_collection **out_xac);

/*!
 * Creates a new xrt_allocation_collection that wraps a vk_image_collection.
 * Takes ownership of the vk_image_collection - the caller should not
 * call vk_ic_destroy on it after this function succeeds.
 *
 * @param vk          Vulkan bundle.
 * @param vkic        The vk_image_collection to wrap (ownership transferred).
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_vk
 */
xrt_result_t
vk_xac_create(struct vk_bundle *vk, struct vk_image_collection *vkic, struct xrt_allocation_collection **out_xac);

/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
