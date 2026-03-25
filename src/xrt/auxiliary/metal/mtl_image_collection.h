// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal image collection for Vulkan/Metal interop.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_metal
 */

#pragma once

#include "xrt/xrt_allocation_collection.h"


#include <mach/mach_types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct vk_bundle;
struct xrt_swapchain_create_info;


/*!
 * Allocates a new allocation collection that is backed by pure Metal textures.
 * Only supports retrieving MTLTexture from it (no Vulkan images).
 *
 * @param mtl_device  The MTLDevice as a void pointer (id<MTLDevice>).
 * @param xscci       Swapchain creation info describing the images.
 * @param image_count Number of images to allocate.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_metal
 */
xrt_result_t
mtl_image_collection_create(void *mtl_device,
                            const struct xrt_swapchain_create_info *xscci,
                            uint32_t image_count,
                            struct xrt_allocation_collection **out_xac);

/*!
 * Allocates a new allocation collection that is backed by Vulkan images and
 * supports retrieving both VkImage and MTLTexture from it. This requires
 * the VK_EXT_metal_objects extension to be enabled on the vk_bundle.
 *
 * The MTLDevice is obtained from the Vulkan device using
 * VkExportMetalDeviceInfoEXT.
 *
 * @param vk          Vulkan bundle with VK_EXT_metal_objects enabled.
 * @param xscci       Swapchain creation info describing the images.
 * @param image_count Number of images to allocate.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_metal
 */
xrt_result_t
mtl_image_collection_create_from_vk(struct vk_bundle *vk,
                                    const struct xrt_swapchain_create_info *xscci,
                                    uint32_t image_count,
                                    struct xrt_allocation_collection **out_xac);

/*!
 * Creates a new allocation collection by wrapping Metal textures as Vulkan images.
 *
 * Takes an existing allocation collection that doesn't support Vulkan images
 * but supports Metal textures (XRT_ALLOCATION_PROPERTY_METAL_DEVICE and
 * XRT_ALLOCATION_TYPE_METAL_TEXTURE), verifies that its MTLDevice matches the
 * one from the given vk_bundle, and wraps the Metal textures as VkImages.
 *
 * This requires the VK_EXT_metal_objects extension to be enabled on the
 * @ref vk_bundle.
 *
 * @param vk      Vulkan bundle with VK_EXT_metal_objects enabled.
 * @param xac_mtl Allocation collection with Metal textures (must support
 *                XRT_ALLOCATION_PROPERTY_METAL_DEVICE and
 *                XRT_ALLOCATION_TYPE_METAL_TEXTURE).
 * @param out_xac Pointer to receive the new allocation collection with both
 *                VkImage and MTLTexture support.
 * @return XRT_SUCCESS on success, XRT_ERROR_VULKAN if MTLDevice doesn't match
 *         or Metal objects extension is not available, XRT_ERROR_UNSUPPORTED_PROPERTY
 *         if the input collection doesn't support required properties/types.
 *
 * @ingroup aux_metal
 */
xrt_result_t
mtl_image_collection_wrap_from_metal(struct vk_bundle *vk,
                                     struct xrt_allocation_collection *xac_mtl,
                                     struct xrt_allocation_collection **out_xac);

/*!
 * Like @ref mtl_image_collection_create_from_vk but creates shareable textures
 * via [MTLDevice newSharedTextureWithDescriptor:]. Supports get_all(NATIVE_IMAGE)
 * which exports Mach ports from MTLSharedTextureHandle.
 *
 * @param vk          Vulkan bundle with VK_EXT_metal_objects enabled.
 * @param xscci       Swapchain creation info describing the images.
 * @param image_count Number of images to allocate.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_metal
 */
xrt_result_t
mtl_image_collection_create_shared_from_vk(struct vk_bundle *vk,
                                            const struct xrt_swapchain_create_info *xscci,
                                            uint32_t image_count,
                                            struct xrt_allocation_collection **out_xac);

/*!
 * Create an allocation collection by importing textures from Mach ports.
 *
 * Each port is a send right from MTLSharedTextureHandle. On success,
 * ownership of all ports transfers to the collection (send rights are
 * deallocated after import). On failure, caller retains ownership.
 *
 * @param mtl_device  MTLDevice as void*.
 * @param xscci       Swapchain creation info.
 * @param ports       Array of Mach port send rights.
 * @param image_count Number of ports/images.
 * @param out_xac     Pointer to receive the new allocation collection.
 * @return XRT_SUCCESS on success.
 *
 * @ingroup aux_metal
 */
xrt_result_t
mtl_image_collection_import_from_port(void *mtl_device,
                                       const struct xrt_swapchain_create_info *xscci,
                                       const mach_port_t *ports,
                                       uint32_t image_count,
                                       struct xrt_allocation_collection **out_xac);


#ifdef __cplusplus
}
#endif
