// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Vulkan image collection implementing xrt_allocation_collection.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_vk
 */

#include "xrt/xrt_config_vulkan.h"
#include "xrt/xrt_compositor.h"

#include "util/u_misc.h"
#include "util/u_logging.h"

#include "vk/vk_helpers.h"
#include "vk/vk_image_allocator.h"
#include "vk/vk_image_collection.h"


/*
 *
 * Structs.
 *
 */

/*!
 * Implementation of @ref xrt_allocation_collection that wraps a vk_image_collection.
 */
struct vk_xrt_image_collection
{
	//! Base interface.
	struct xrt_allocation_collection base;

	//! Vulkan bundle used to create images.
	struct vk_bundle *vk;

	//! The wrapped vk_image_collection.
	struct vk_image_collection vkic;

	//! Native image handles (populated on demand).
	struct xrt_image_native natives[XRT_MAX_SWAPCHAIN_IMAGES];

	//! Whether native handles have been retrieved.
	bool natives_valid;

	//! Swapchain creation info used to create this collection.
	struct xrt_swapchain_create_info info;
};


/*
 *
 * Helpers.
 *
 */

static inline struct vk_xrt_image_collection *
vk_xrt_image_collection(struct xrt_allocation_collection *xac)
{
	return (struct vk_xrt_image_collection *)xac;
}


/*
 *
 * Internal functions.
 *
 */

/*!
 * Ensure native handles are populated by calling vk_ic_get_handles if needed.
 */
static xrt_result_t
ensure_natives(struct vk_xrt_image_collection *vxic)
{
	if (vxic->natives_valid) {
		return XRT_SUCCESS;
	}

	xrt_graphics_buffer_handle_t handles[XRT_MAX_SWAPCHAIN_IMAGES];

	VkResult ret = vk_ic_get_handles(vxic->vk, &vxic->vkic, XRT_MAX_SWAPCHAIN_IMAGES, handles);
	if (ret != VK_SUCCESS) {
		U_LOG_E("Failed to get native handles for images: %d", ret);
		return XRT_ERROR_VULKAN;
	}

	for (uint32_t i = 0; i < vxic->base.image_count; i++) {
		vxic->natives[i].handle = handles[i];
		vxic->natives[i].size = vxic->vkic.images[i].size;
		vxic->natives[i].use_dedicated_allocation = vxic->vkic.images[i].use_dedicated_allocation;
	}

	vxic->natives_valid = true;

	return XRT_SUCCESS;
}


/*
 *
 * xrt_allocation_collection interface.
 *
 */

static xrt_result_t
vk_xic_get_property(struct xrt_allocation_collection *xac, enum xrt_allocation_property prop, void *ptr)
{
	struct vk_xrt_image_collection *vxic = vk_xrt_image_collection(xac);

	switch (prop) {
	case XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO:
		// See XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO for semantics.
		// Copy the swapchain info into the caller's struct.
		*(struct xrt_swapchain_create_info *)ptr = vxic->info;
		return XRT_SUCCESS;
	case XRT_ALLOCATION_PROPERTY_VK_BUNDLE:
		// See XRT_ALLOCATION_PROPERTY_VK_BUNDLE for reference semantics.
		*(struct vk_bundle **)ptr = vxic->vk;
		return XRT_SUCCESS;
	case XRT_ALLOCATION_PROPERTY_METAL_DEVICE:
		U_LOG_E("MTLDevice not supported for Vulkan image collection");
		return XRT_ERROR_UNSUPPORTED_PROPERTY;
	default: U_LOG_E("Unknown property: %d", (int)prop); return XRT_ERROR_UNSUPPORTED_PROPERTY;
	}
}

static xrt_result_t
vk_xic_get_all(struct xrt_allocation_collection *xac, enum xrt_allocation_type type, size_t element_size, void *ptr)
{
	struct vk_xrt_image_collection *vxic = vk_xrt_image_collection(xac);

	switch (type) {
	case XRT_ALLOCATION_TYPE_VULKAN_IMAGE: {
		if (element_size != sizeof(VkImage)) {
			U_LOG_E("Invalid element size for VkImage: %zu (expected %zu)", element_size, sizeof(VkImage));
			return XRT_ERROR_IPC_FAILURE;
		}
		VkImage *images = (VkImage *)ptr;
		// See XRT_ALLOCATION_TYPE_VULKAN_IMAGE for lifetime semantics.
		for (uint32_t i = 0; i < vxic->base.image_count; i++) {
			images[i] = vxic->vkic.images[i].handle;
		}
		return XRT_SUCCESS;
	}
	case XRT_ALLOCATION_TYPE_NATIVE_IMAGE: {
		if (element_size != sizeof(struct xrt_image_native)) {
			U_LOG_E("Invalid element size for xrt_image_native: %zu (expected %zu)", element_size,
			        sizeof(struct xrt_image_native));
			return XRT_ERROR_IPC_FAILURE;
		}

		xrt_result_t xret = ensure_natives(vxic);
		if (xret != XRT_SUCCESS) {
			return xret;
		}

		struct xrt_image_native *natives = (struct xrt_image_native *)ptr;
		for (uint32_t i = 0; i < vxic->base.image_count; i++) {
			natives[i] = vxic->natives[i];
		}
		return XRT_SUCCESS;
	}
	default: U_LOG_E("Unsupported allocation type: %d", (int)type); return XRT_ERROR_IPC_FAILURE;
	}
}

static void
vk_xic_destroy(struct xrt_allocation_collection *xac)
{
	struct vk_xrt_image_collection *vxic = vk_xrt_image_collection(xac);

	// Note: We don't unref native handles here - vk_ic_get_handles transfers
	// ownership, but the VkDeviceMemory backing them is destroyed by vk_ic_destroy.
	// The handles themselves become invalid when the memory is freed.

	vk_ic_destroy(vxic->vk, &vxic->vkic);
	free(vxic);
}


/*
 *
 * Helper function.
 *
 */

static void
init_base(struct vk_xrt_image_collection *vxic,
          struct vk_bundle *vk,
          const struct xrt_swapchain_create_info *xscci,
          uint32_t image_count)
{
	vxic->base.reference.count = 1;
	vxic->base.image_count = image_count;
	vxic->base.supported_types_count = 2;
	vxic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_VULKAN_IMAGE;
	vxic->base.supported_types[1].type = XRT_ALLOCATION_TYPE_NATIVE_IMAGE;
	vxic->base.supported_properties_count = 2;
	vxic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	vxic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_VK_BUNDLE;
	vxic->base.get_property = vk_xic_get_property;
	vxic->base.get_all = vk_xic_get_all;
	vxic->base.destroy = vk_xic_destroy;

	vxic->vk = vk;
	vxic->natives_valid = false;
	vxic->info = *xscci;
}


/*
 *
 * Exported functions.
 *
 */

xrt_result_t
vk_xac_allocate(struct vk_bundle *vk,
                const struct xrt_swapchain_create_info *xscci,
                uint32_t image_count,
                struct xrt_allocation_collection **out_xac)
{
	assert(vk != NULL);
	assert(xscci != NULL);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	// Allocate the image collection structure.
	struct vk_xrt_image_collection *vxic = U_TYPED_CALLOC(struct vk_xrt_image_collection);
	if (vxic == NULL) {
		U_LOG_E("Failed to allocate vk_xrt_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Allocate Vulkan images.
	VkResult ret = vk_ic_allocate(vk, xscci, image_count, &vxic->vkic);
	if (ret == VK_ERROR_FEATURE_NOT_PRESENT) {
		free(vxic);
		return XRT_ERROR_SWAPCHAIN_FLAG_VALID_BUT_UNSUPPORTED;
	}
	if (ret == VK_ERROR_FORMAT_NOT_SUPPORTED) {
		free(vxic);
		return XRT_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
	}
	if (ret != VK_SUCCESS) {
		free(vxic);
		return XRT_ERROR_VULKAN;
	}

	// Initialize base structure.
	init_base(vxic, vk, xscci, image_count);

	*out_xac = &vxic->base;

	return XRT_SUCCESS;
}

xrt_result_t
vk_xac_from_natives(struct vk_bundle *vk,
                    const struct xrt_swapchain_create_info *xscci,
                    struct xrt_image_native *native_images,
                    uint32_t image_count,
                    struct xrt_allocation_collection **out_xac)
{
	assert(vk != NULL);
	assert(xscci != NULL);
	assert(native_images != NULL);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	// Allocate the image collection structure.
	struct vk_xrt_image_collection *vxic = U_TYPED_CALLOC(struct vk_xrt_image_collection);
	if (vxic == NULL) {
		U_LOG_E("Failed to allocate vk_xrt_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Import native images into Vulkan.
	VkResult ret = vk_ic_from_natives(vk, xscci, native_images, image_count, &vxic->vkic);
	if (ret == VK_ERROR_FEATURE_NOT_PRESENT) {
		free(vxic);
		return XRT_ERROR_SWAPCHAIN_FLAG_VALID_BUT_UNSUPPORTED;
	}
	if (ret == VK_ERROR_FORMAT_NOT_SUPPORTED) {
		free(vxic);
		return XRT_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
	}
	if (ret != VK_SUCCESS) {
		free(vxic);
		return XRT_ERROR_VULKAN;
	}

	// Initialize base structure.
	init_base(vxic, vk, xscci, image_count);

	*out_xac = &vxic->base;

	return XRT_SUCCESS;
}

xrt_result_t
vk_xac_create(struct vk_bundle *vk, struct vk_image_collection *vkic, struct xrt_allocation_collection **out_xac)
{
	assert(vk != NULL);
	assert(vkic != NULL);
	assert(out_xac != NULL);
	assert(vkic->image_count > 0);
	assert(vkic->image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	// Allocate the image collection structure.
	struct vk_xrt_image_collection *vxic = U_TYPED_CALLOC(struct vk_xrt_image_collection);
	if (vxic == NULL) {
		U_LOG_E("Failed to allocate vk_xrt_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure.
	init_base(vxic, vk, &vkic->info, vkic->image_count);

	// Take ownership of the vk_image_collection contents.
	vxic->vkic = *vkic;

	// Clear the source so caller doesn't accidentally double-free.
	U_ZERO(vkic);

	*out_xac = &vxic->base;

	return XRT_SUCCESS;
}
