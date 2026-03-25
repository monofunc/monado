// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Auxiliary allocation collection helpers.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compositor.h"


#ifdef __cplusplus
extern "C" {
#endif

struct vk_bundle;


/*!
 * @defgroup aux_allocation Auxiliary allocation utilities
 * @ingroup aux
 *
 * @{
 */

/*!
 * Allocates a new allocation collection for the given swapchain create info.
 * Abstracts platform-specific differences (e.g., Metal on macOS, Vulkan elsewhere).
 *
 * @param vk          Vulkan bundle.
 * @param xscci       Swapchain creation info describing the images.
 * @param image_count Number of images to allocate.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success, or an appropriate error code.
 *
 * @ingroup aux_allocation
 */
xrt_result_t
a_allocator_allocate(struct vk_bundle *vk,
                     const struct xrt_swapchain_create_info *xscci,
                     uint32_t image_count,
                     struct xrt_allocation_collection **out_xac);

/*!
 * Imports native images into a new allocation collection.
 *
 * Takes ownership of the native handles in the native_images array. On success,
 * the handles are consumed and should not be used by the caller. On failure,
 * the handles remain valid and owned by the caller. This is an all-or-nothing
 * operation.
 *
 * @param vk            Vulkan bundle.
 * @param xscci         Swapchain creation info describing the images.
 * @param native_images Array of native image handles to import (ownership transferred on success).
 * @param image_count   Number of images to import.
 * @param out_xac       Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success, or an appropriate error code.
 *
 * @ingroup aux_allocation
 */
xrt_result_t
a_allocator_import_from_natives(struct vk_bundle *vk,
                                const struct xrt_swapchain_create_info *xscci,
                                struct xrt_image_native *native_images,
                                uint32_t image_count,
                                struct xrt_allocation_collection **out_xac);

/*!
 * Ensures the allocation collection supports retrieving VkImages.
 *
 * Takes an existing allocation collection and returns one that supports getting
 * VkImages. The conversion logic is as follows:
 *
 * - If the given collection already supports VkImage and uses the same
 *   @ref vk_bundle, returns it (with incremented reference count).
 * - If the collection supports native images
 *   (@ref XRT_ALLOCATION_TYPE_NATIVE_IMAGE), creates a new collection by
 *   importing the native images using @ref a_allocator_import_from_natives.
 * - If the collection supports Metal textures
 *   (@ref XRT_ALLOCATION_TYPE_METAL_TEXTURE),
 *   wraps them as VkImages using @ref mtl_image_collection_wrap_from_metal
 *   (macOS only).
 * - Otherwise, returns XRT_ERROR_ALLOCATION.
 *
 * @param vk      Vulkan bundle.
 * @param xac     Existing allocation collection to convert.
 * @param out_xac Pointer to receive the allocation collection with VkImage support.
 * @return XRT_SUCCESS on success, or an appropriate error code.
 *
 * @ingroup aux_allocation
 */
xrt_result_t
a_allocator_ensure_vk_images(struct vk_bundle *vk,
                             struct xrt_allocation_collection *xac,
                             struct xrt_allocation_collection **out_xac);

/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
