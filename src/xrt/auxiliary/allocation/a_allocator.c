// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Auxiliary allocation collection helpers.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_allocation
 */

#include "util/u_handles.h"

#include "vk/vk_helpers.h"
#include "vk/vk_image_collection.h"

#if defined(XRT_OS_OSX)
#include "metal/mtl_image_collection.h"
#endif

#include "a_allocator.h"


/*
 *
 * Exported functions.
 *
 */

xrt_result_t
a_allocator_allocate(struct vk_bundle *vk,
                     const struct xrt_swapchain_create_info *xscci,
                     uint32_t image_count,
                     struct xrt_allocation_collection **out_xac)
{
#if defined(XRT_OS_LINUX) || defined(XRT_OS_WINDOWS) || defined(XRT_OS_ANDROID)
	// On Linux, Windows, and Android, use Vulkan allocation collection.
	xrt_result_t xret = vk_xac_allocate(vk, xscci, image_count, out_xac);
	XVK_CHK_ALWAYS_RET(xret, "vk_xac_allocate");
#elif defined(XRT_OS_OSX)
	// On macOS, use Metal-backed allocation collection.
	xrt_result_t xret = mtl_image_collection_create_from_vk(vk, xscci, image_count, out_xac);
	XVK_CHK_ALWAYS_RET(xret, "mtl_image_collection_create_from_vk");
#else
#error "Unsupported platform"
#endif
}

xrt_result_t
a_allocator_import_from_natives(struct vk_bundle *vk,
                                const struct xrt_swapchain_create_info *xscci,
                                struct xrt_image_native *native_images,
                                uint32_t image_count,
                                struct xrt_allocation_collection **out_xac)
{
#if defined(XRT_OS_LINUX) || defined(XRT_OS_WINDOWS) || defined(XRT_OS_ANDROID)
	// Import from native images using Vulkan.
	// This is platform-independent as it imports existing native handles.
	return vk_xac_from_natives(vk, xscci, native_images, image_count, out_xac);
#elif defined(XRT_OS_OSX)
	// Not supported yet.
	U_LOG_E("Importing from native images is not supported on macOS yet");
	return XRT_ERROR_ALLOCATION;
#else
#error "Unsupported platform"
#endif
}

xrt_result_t
a_allocator_ensure_vk_images(struct vk_bundle *vk,
                             struct xrt_allocation_collection *xac,
                             struct xrt_allocation_collection **out_xac)
{
	xrt_result_t xret = XRT_ERROR_ALLOCATION;

	// Check if the collection already supports VkImage.
	bool supports_swapchain_info = false;
	bool supports_vk_bundle = false;
	bool supports_metal_device = false;
	bool supports_vk_image = false;
	bool supports_native_images = false;
	bool supports_metal_texture = false;

	for (uint32_t i = 0; i < xac->supported_properties_count; i++) {
		switch (xac->supported_properties[i].property) {
		case XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO: supports_swapchain_info = true; break;
		case XRT_ALLOCATION_PROPERTY_VK_BUNDLE: supports_vk_bundle = true; break;
		case XRT_ALLOCATION_PROPERTY_METAL_DEVICE: supports_metal_device = true; break;
		}
	}

	for (uint32_t i = 0; i < xac->supported_types_count; i++) {
		switch (xac->supported_types[i].type) {
		case XRT_ALLOCATION_TYPE_VULKAN_IMAGE: supports_vk_image = true; break;
		case XRT_ALLOCATION_TYPE_NATIVE_IMAGE: supports_native_images = true; break;
		case XRT_ALLOCATION_TYPE_METAL_TEXTURE: supports_metal_texture = true; break;
		case XRT_ALLOCATION_TYPE_IOSURFACE: break; // Not used.
		}
	}

	// If it already supports VkImage, check if it uses the same vk_bundle.
	if (supports_vk_image && supports_vk_bundle) {
		struct vk_bundle *xac_vk = NULL;
		xret = xrt_allocation_collection_get_property( //
		    xac,                                       //
		    XRT_ALLOCATION_PROPERTY_VK_BUNDLE,         //
		    &xac_vk);                                  //
		XVK_CHK_AND_RET(xret, "xrt_allocation_collection_get_property(PROPERTY_VK_BUNDLE)");
		if (xret == XRT_SUCCESS && xac_vk == vk) {
			// Same vk_bundle, just return it with a reference.
			xrt_allocation_collection_reference(out_xac, xac);
			return XRT_SUCCESS;
		}
		// Different vk_bundle or couldn't get property, need to convert.
	}

	// Check if the collection supports native images.
	if (supports_native_images && supports_swapchain_info) {
		// Get the swapchain info.
		struct xrt_swapchain_create_info xscci;
		xret = xrt_allocation_collection_get_property( //
		    xac,                                       //
		    XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO,    //
		    &xscci);                                   //
		XVK_CHK_AND_RET(xret, "xrt_allocation_collection_get_property(PROPERTY_SWAPCHAIN_INFO)");

		// Allocate native images array.
		uint32_t image_count = xac->image_count;
		struct xrt_image_native *native_images = U_TYPED_ARRAY_CALLOC(struct xrt_image_native, image_count);
		if (native_images == NULL) {
			U_LOG_E("Failed to allocate native images array");
			return XRT_ERROR_ALLOCATION;
		}

		// Get the native images.
		xret = xrt_allocation_collection_get_all( //
		    xac,                                  //
		    XRT_ALLOCATION_TYPE_NATIVE_IMAGE,     //
		    sizeof(struct xrt_image_native),      //
		    native_images);                       //
		XVK_CHK_ONLY_PRINT(xret, "xrt_allocation_collection_get_all(TYPE_NATIVE_IMAGE)");
		if (xret != XRT_SUCCESS) {
			free(native_images);
			return xret;
		}

		// Import them into a new collection.
		xret = a_allocator_import_from_natives( //
		    vk,                                 //
		    &xscci,                             //
		    native_images,                      //
		    image_count,                        //
		    out_xac);                           //
		XVK_CHK_ONLY_PRINT(xret, "a_allocator_import_from_natives");

		// On failure, unref the native images.
		if (xret != XRT_SUCCESS) {
			for (uint32_t i = 0; i < image_count; i++) {
				u_graphics_buffer_unref(&native_images[i].handle);
			}
		}

		// Free the native images array (ownership transferred to new collection or freed on error).
		free(native_images);
		if (xret == XRT_SUCCESS) {
			return xret;
		}
	}

#if defined(XRT_OS_OSX)
	if (supports_metal_device && supports_metal_texture) {
		// Use the Metal wrapper function.
		xret = mtl_image_collection_wrap_from_metal(vk, xac, out_xac);
		XVK_CHK_ALWAYS_RET(xret, "mtl_image_collection_wrap_from_metal");
	}
#endif

	VK_ERROR(vk,
	         "All methods to ensure VkImage failed!"
	         "\n\tsupports_swapchain_info: %s"
	         "\n\tsupports_vk_bundle: %s"
	         "\n\tsupports_metal_device: %s"
	         "\n\tsupports_vk_image: %s"
	         "\n\tsupports_native_images: %s"
	         "\n\tsupports_metal_texture: %s",
	         supports_swapchain_info ? "true" : "false", //
	         supports_vk_bundle ? "true" : "false",      //
	         supports_metal_device ? "true" : "false",   //
	         supports_vk_image ? "true" : "false",       //
	         supports_native_images ? "true" : "false",  //
	         supports_metal_texture ? "true" : "false"); //

	// No supported conversion path.
	return XRT_ERROR_ALLOCATION;
}
