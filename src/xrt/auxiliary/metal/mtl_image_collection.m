// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal image collection for Vulkan/Metal interop.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_metal
 */

#import <Metal/Metal.h>

#include <mach/mach.h>

#include "xrt/xrt_config_vulkan.h"

#include "xrt/xrt_compositor.h"

#include "util/u_misc.h"
#include "util/u_logging.h"

#include "vk/vk_helpers.h"

#include "mtl_format.h"
#include "mtl_image_collection.h"
#include "mtl_shared_texture.h"


/*
 *
 * Structs.
 *
 */

/*!
 * Implementation of @ref xrt_allocation_collection that supports
 * both Vulkan images and Metal textures.
 */
struct mtl_image_collection
{
	//! Base interface.
	struct xrt_allocation_collection base;

	//! Vulkan bundle used to create images.
	struct vk_bundle *vk;

	//! Metal device (retained).
	id<MTLDevice> device;

	//! Vulkan images wrapping the Metal textures.
	VkImage images[XRT_MAX_SWAPCHAIN_IMAGES];

	//! Metal textures.
	id<MTLTexture> textures[XRT_MAX_SWAPCHAIN_IMAGES];

	//! Textures created via newSharedTextureWithDescriptor (can export Mach ports).
	bool is_shared;

	//! Swapchain creation info used to create this collection.
	struct xrt_swapchain_create_info info;
};


/*
 *
 * Helpers.
 *
 */

static inline struct mtl_image_collection *
mtl_image_collection(struct xrt_allocation_collection *xac)
{
	return (struct mtl_image_collection *)xac;
}

/*!
 * Create a Metal texture descriptor and texture from swapchain create info.
 * This is the pure Metal version without any Vulkan dependencies.
 *
 * @param mtl_device    The Metal device to create the texture on.
 * @param info          Swapchain creation info.
 * @param mtl_format    The Metal pixel format.
 * @param out_texture   Output: the created Metal texture (retained).
 * @return true on success, false on failure.
 */
static bool
create_texture_for_metal(id<MTLDevice> mtl_device,
                         const struct xrt_swapchain_create_info *info,
                         MTLPixelFormat mtl_format,
                         id<MTLTexture> *out_texture)
{
	// Create Metal texture descriptor.
	MTLTextureDescriptor *desc = [[MTLTextureDescriptor alloc] init];
	desc.width = info->width;
	desc.height = info->height;
	desc.depth = 1;
	desc.arrayLength = info->array_size > 0 ? info->array_size : 1;
	desc.mipmapLevelCount = info->mip_count > 0 ? info->mip_count : 1;
	desc.sampleCount = info->sample_count > 0 ? info->sample_count : 1;
	desc.textureType = info->array_size > 1 ? MTLTextureType2DArray : MTLTextureType2D;
	desc.pixelFormat = mtl_format;
	desc.storageMode = MTLStorageModePrivate;

	// Set Metal usage based on XRT swapchain bits.
	MTLTextureUsage mtl_usage = 0;
	if (info->bits & XRT_SWAPCHAIN_USAGE_COLOR) {
		mtl_usage |= MTLTextureUsageRenderTarget;
	}
	if (info->bits & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL) {
		mtl_usage |= MTLTextureUsageRenderTarget;
	}
	if (info->bits & XRT_SWAPCHAIN_USAGE_SAMPLED) {
		mtl_usage |= MTLTextureUsageShaderRead;
	}
	if (info->bits & XRT_SWAPCHAIN_USAGE_UNORDERED_ACCESS) {
		mtl_usage |= MTLTextureUsageShaderWrite;
	}
	// Always allow shader read for flexibility.
	mtl_usage |= MTLTextureUsageShaderRead;
	desc.usage = mtl_usage;

	// Create the Metal texture.
	id<MTLTexture> texture = [mtl_device newTextureWithDescriptor:desc];
	[desc release];

	if (texture == nil) {
		U_LOG_E("Failed to create Metal texture");
		return false;
	}

	*out_texture = texture;

	return true;
}

/*!
 * Create a Metal texture and import it into a Vulkan image.
 */
static VkResult
create_image_for_metal(struct vk_bundle *vk,
                       id<MTLDevice> mtl_device,
                       const struct xrt_swapchain_create_info *info,
                       VkImage *out_image,
                       id<MTLTexture> *out_texture)
{
	VkResult ret;
	VkFormat vk_format = (VkFormat)info->format;

	// Convert Vulkan format to Metal format.
	MTLPixelFormat mtl_format = (MTLPixelFormat)mtl_format_from_vk(vk_format);
	if (mtl_format == MTLPixelFormatInvalid) {
		U_LOG_E("Unsupported Vulkan format for Metal: %d", vk_format);
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}

	VkImageUsageFlags image_usage = vk_csci_get_image_usage_flags(vk, vk_format, info->bits);
	if (image_usage == 0) {
		U_LOG_E("Unsupported swapchain usage flags");
		return VK_ERROR_FEATURE_NOT_PRESENT;
	}

	// Create the Metal texture using the shared helper.
	id<MTLTexture> texture = nil;
	if (!create_texture_for_metal(mtl_device, info, mtl_format, &texture)) {
		U_LOG_E("Failed to create Metal texture");
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	// Build Vulkan image create flags.
	VkImageCreateFlags image_create_flags = 0;
	if ((info->bits & XRT_SWAPCHAIN_USAGE_MUTABLE_FORMAT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
	}
	if ((info->create & XRT_SWAPCHAIN_CREATE_PROTECTED_CONTENT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_PROTECTED_BIT;
	}
	if (info->face_count == 6) {
		image_create_flags |= VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	}

	// Import the Metal texture into the Vulkan image.
	VkImageAspectFlagBits plane = (info->bits & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL)
	                                  ? VK_IMAGE_ASPECT_DEPTH_BIT
	                                  : VK_IMAGE_ASPECT_COLOR_BIT;

	VkImportMetalTextureInfoEXT import_texture_info = {
	    .sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT,
	    .pNext = NULL,
	    .plane = plane,
	    .mtlTexture = (__bridge MTLTexture_id)texture,
	};

	VkImageCreateInfo create_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &import_texture_info,
	    .flags = image_create_flags,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = vk_format,
	    .extent = {.width = info->width, .height = info->height, .depth = 1},
	    .mipLevels = info->mip_count,
	    .arrayLayers = info->array_size * info->face_count,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_OPTIMAL,
	    .usage = image_usage,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};

	VkImage image = VK_NULL_HANDLE;
	ret = vk->vkCreateImage(vk->device, &create_info, NULL, &image);
	VK_CHK_ONLY_PRINT(ret, "vkCreateImage");
	if (ret != VK_SUCCESS) {
		[texture release];
		return ret;
	}

	*out_texture = texture;
	*out_image = image;

	return VK_SUCCESS;
}


/*
 *
 * Internal functions.
 *
 */

/*!
 * Finalize and free an image collection. Cleans up all Vulkan images,
 * Metal textures, and the Metal device.
 *
 * @param mic The image collection to finalize.
 */
static void
mtl_image_collection_fini(struct mtl_image_collection *mic)
{
	struct vk_bundle *vk = mic->vk;

	// Destroy Vulkan images first (they reference the Metal textures).
	for (uint32_t i = 0; i < mic->base.image_count; i++) {
		if (mic->images[i] == VK_NULL_HANDLE) {
			continue;
		}

		// The bundle should always be set for Vulkan images.
		assert(vk != NULL);
		vk->vkDestroyImage(vk->device, mic->images[i], NULL);
		mic->images[i] = VK_NULL_HANDLE;
	}

	// Release Metal textures (memory is managed by Metal).
	for (uint32_t i = 0; i < mic->base.image_count; i++) {
		if (mic->textures[i] == nil) {
			continue;
		}

		[mic->textures[i] release];
		mic->textures[i] = nil;
	}

	// Release Metal device.
	if (mic->device != nil) {
		[mic->device release];
		mic->device = nil;
	}
}


/*
 *
 * xrt_allocation_collection interface.
 *
 */

static xrt_result_t
mtl_xac_get_property(struct xrt_allocation_collection *xac, enum xrt_allocation_property prop, void *ptr)
{
	struct mtl_image_collection *mic = mtl_image_collection(xac);

	switch (prop) {
	case XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO:
		// See XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO for semantics.
		// Copy the swapchain info into the caller's struct.
		*(struct xrt_swapchain_create_info *)ptr = mic->info;
		return XRT_SUCCESS;
	case XRT_ALLOCATION_PROPERTY_VK_BUNDLE:
		// See XRT_ALLOCATION_PROPERTY_VK_BUNDLE for reference semantics.
		if (mic->vk == NULL) {
			U_LOG_E("vk_bundle not available for pure Metal image collection");
			return XRT_ERROR_UNSUPPORTED_PROPERTY;
		}
		*(struct vk_bundle **)ptr = mic->vk;
		return XRT_SUCCESS;
	case XRT_ALLOCATION_PROPERTY_METAL_DEVICE:
		// See XRT_ALLOCATION_PROPERTY_METAL_DEVICE for reference semantics.
		*(void **)ptr = (__bridge void *)mic->device;
		return XRT_SUCCESS;
	default: U_LOG_E("Unknown property: %d", (int)prop); return XRT_ERROR_UNSUPPORTED_PROPERTY;
	}
}

static xrt_result_t
mtl_xac_get_all(struct xrt_allocation_collection *xac, enum xrt_allocation_type type, size_t element_size, void *ptr)
{
	struct mtl_image_collection *mic = mtl_image_collection(xac);

	switch (type) {
	case XRT_ALLOCATION_TYPE_VULKAN_IMAGE: {
		// Pure Metal collections don't have Vulkan images.
		if (mic->vk == NULL) {
			U_LOG_E("VkImage not supported for pure Metal image collection");
			return XRT_ERROR_IPC_FAILURE;
		}
		if (element_size != sizeof(VkImage)) {
			U_LOG_E("Invalid element size for VkImage: %zu (expected %zu)", element_size, sizeof(VkImage));
			return XRT_ERROR_IPC_FAILURE;
		}
		VkImage *images = (VkImage *)ptr;
		// See XRT_ALLOCATION_TYPE_VULKAN_IMAGE for lifetime semantics.
		for (uint32_t i = 0; i < mic->base.image_count; i++) {
			images[i] = mic->images[i];
		}
		return XRT_SUCCESS;
	}
	case XRT_ALLOCATION_TYPE_METAL_TEXTURE: {
		if (element_size != sizeof(void *)) {
			U_LOG_E("Invalid element size for MTLTexture: %zu (expected %zu)", element_size,
			        sizeof(void *));
			return XRT_ERROR_IPC_FAILURE;
		}
		void **textures = (void **)ptr;
		// See XRT_ALLOCATION_TYPE_METAL_TEXTURE for reference semantics.
		for (uint32_t i = 0; i < mic->base.image_count; i++) {
			textures[i] = (__bridge void *)mic->textures[i];
		}
		return XRT_SUCCESS;
	}
	case XRT_ALLOCATION_TYPE_NATIVE_IMAGE: {
		if (!mic->is_shared) {
			U_LOG_E("NATIVE_IMAGE not supported for non-shared collection");
			return XRT_ERROR_IPC_FAILURE;
		}
		if (element_size != sizeof(struct xrt_image_native)) {
			U_LOG_E("Invalid element size for xrt_image_native: %zu (expected %zu)", element_size,
			        sizeof(struct xrt_image_native));
			return XRT_ERROR_IPC_FAILURE;
		}
		struct xrt_image_native *natives = (struct xrt_image_native *)ptr;
		for (uint32_t i = 0; i < mic->base.image_count; i++) {
			mach_port_t port = mtl_shared_texture_to_mach_port((void *)mic->textures[i]);
			if (port == MACH_PORT_NULL) {
				U_LOG_E("Failed to export Mach port for texture %u", i);
				for (uint32_t j = 0; j < i; j++) {
					mach_port_deallocate(mach_task_self(), natives[j].handle);
					natives[j].handle = MACH_PORT_NULL;
				}
				return XRT_ERROR_IPC_FAILURE;
			}
			natives[i].handle = port;
			natives[i].size = 0;
			natives[i].use_dedicated_allocation = false;
			natives[i].is_dxgi_handle = false;
		}
		return XRT_SUCCESS;
	}
	default: U_LOG_E("Unsupported allocation type: %d", (int)type); return XRT_ERROR_IPC_FAILURE;
	}
}

static void
mtl_xac_destroy(struct xrt_allocation_collection *xac)
{
	struct mtl_image_collection *mic = mtl_image_collection(xac);

	mtl_image_collection_fini(mic);
	free(mic);
}


/*
 *
 * Exported functions.
 *
 */

xrt_result_t
mtl_image_collection_create(void *mtl_device,
                            const struct xrt_swapchain_create_info *xscci,
                            uint32_t image_count,
                            struct xrt_allocation_collection **out_xac)
{
	struct mtl_image_collection *mic = NULL;
	uint32_t i;

	assert(mtl_device != NULL);
	assert(xscci != NULL);
	assert(xscci->format != 0);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	// Convert Vulkan format to Metal format.
	VkFormat vk_format = (VkFormat)xscci->format;
	MTLPixelFormat mtl_format = (MTLPixelFormat)mtl_format_from_vk(vk_format);
	if (mtl_format == MTLPixelFormatInvalid) {
		U_LOG_E("Unsupported Vulkan format for Metal: %d", vk_format);
		return XRT_ERROR_VULKAN;
	}

	// Allocate the image collection structure.
	mic = U_TYPED_CALLOC(struct mtl_image_collection);
	if (mic == NULL) {
		U_LOG_E("Failed to allocate mtl_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure for pure Metal (MTLTexture supported).
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 2;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 1;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	// No Vulkan bundle for pure Metal.
	mic->vk = NULL;

	// Retain the Metal device.
	mic->device = [(id<MTLDevice>)mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Create Metal textures.
	for (i = 0; i < image_count; i++) {
		bool bret = create_texture_for_metal( //
		    mic->device,                      //
		    xscci,                            //
		    mtl_format,                       //
		    &mic->textures[i]);               //
		if (!bret) {
			U_LOG_E("Failed to create Metal texture %u", i);
			goto err_textures;
		}
	}

	*out_xac = &mic->base;

	return XRT_SUCCESS;

err_textures:
	mtl_image_collection_fini(mic);
	free(mic);

	return XRT_ERROR_VULKAN;
}

xrt_result_t
mtl_image_collection_create_from_vk(struct vk_bundle *vk,
                                    const struct xrt_swapchain_create_info *xscci,
                                    uint32_t image_count,
                                    struct xrt_allocation_collection **out_xac)
{
	struct mtl_image_collection *mic = NULL;
	VkResult ret;
	uint32_t i;

	assert(vk != NULL);
	assert(xscci != NULL);
	assert(xscci->format != 0);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	if (!vk->has_EXT_metal_objects) {
		U_LOG_E("VK_EXT_metal_objects extension not enabled");
		return XRT_ERROR_VULKAN;
	}

	// Export the Metal device from Vulkan using VK_EXT_metal_objects.
	VkExportMetalDeviceInfoEXT device_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_DEVICE_INFO_EXT,
	    .pNext = NULL,
	    .mtlDevice = NULL,
	};

	VkExportMetalObjectsInfoEXT export_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_OBJECTS_INFO_EXT,
	    .pNext = &device_info,
	};

	vk->vkExportMetalObjectsEXT(vk->device, &export_info);

	if (device_info.mtlDevice == NULL) {
		U_LOG_E("Failed to export MTLDevice from Vulkan");
		return XRT_ERROR_VULKAN;
	}

	id<MTLDevice> mtl_device = (__bridge id<MTLDevice>)device_info.mtlDevice;

	// Allocate the image collection structure.
	mic = U_TYPED_CALLOC(struct mtl_image_collection);
	if (mic == NULL) {
		U_LOG_E("Failed to allocate mtl_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure.
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 3;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_VK_BUNDLE;
	mic->base.supported_properties[2].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 2;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_VULKAN_IMAGE;
	mic->base.supported_types[1].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	mic->vk = vk;

	// Retain the Metal device.
	mic->device = [mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Create Metal textures and import them into Vulkan images.
	for (i = 0; i < image_count; i++) {
		ret = create_image_for_metal( //
		    vk,                       //
		    mic->device,              //
		    xscci,                    //
		    &mic->images[i],          //
		    &mic->textures[i]);       //
		VK_CHK_WITH_GOTO(ret, "create_image_for_metal", err_images);
	}

	*out_xac = &mic->base;

	return XRT_SUCCESS;

err_images:
	mtl_image_collection_fini(mic);
	free(mic);

	return XRT_ERROR_VULKAN;
}

xrt_result_t
mtl_image_collection_wrap_from_metal(struct vk_bundle *vk,
                                     struct xrt_allocation_collection *xac_mtl,
                                     struct xrt_allocation_collection **out_xac)
{
	struct mtl_image_collection *mic = NULL;
	VkResult vk_ret;
	xrt_result_t xrt_ret;
	uint32_t i;

	assert(vk != NULL);
	assert(xac_mtl != NULL);
	assert(out_xac != NULL);

	if (!vk->has_EXT_metal_objects) {
		U_LOG_E("VK_EXT_metal_objects extension not enabled");
		return XRT_ERROR_VULKAN;
	}

	// Verify that the input allocation collection supports the required properties and types.
	bool has_mtl_device_property = false;
	bool has_metal_texture_type = false;
	bool has_swapchain_info_property = false;

	for (uint32_t j = 0; j < xac_mtl->supported_properties_count; j++) {
		if (xac_mtl->supported_properties[j].property == XRT_ALLOCATION_PROPERTY_METAL_DEVICE) {
			has_mtl_device_property = true;
		}
		if (xac_mtl->supported_properties[j].property == XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO) {
			has_swapchain_info_property = true;
		}
	}

	for (uint32_t j = 0; j < xac_mtl->supported_types_count; j++) {
		if (xac_mtl->supported_types[j].type == XRT_ALLOCATION_TYPE_METAL_TEXTURE) {
			has_metal_texture_type = true;
		}
	}

	if (!has_mtl_device_property || !has_metal_texture_type || !has_swapchain_info_property) {
		U_LOG_E("Input allocation collection does not support required properties/types");
		U_LOG_E("Has MTL_DEVICE: %d, Has METAL_TEXTURE: %d, Has SWAPCHAIN_INFO: %d", //
		        has_mtl_device_property, has_metal_texture_type, has_swapchain_info_property);
		return XRT_ERROR_UNSUPPORTED_PROPERTY;
	}

	// Get the MTLDevice from the input allocation collection.
	void *input_mtl_device_ptr = NULL;
	xrt_ret = xrt_allocation_collection_get_property( //
	    xac_mtl,                                      //
	    XRT_ALLOCATION_PROPERTY_METAL_DEVICE,         //
	    &input_mtl_device_ptr);                       //
	if (xrt_ret != XRT_SUCCESS) {
		U_LOG_E("Failed to get MTLDevice from input allocation collection");
		return xrt_ret;
	}

	id<MTLDevice> input_mtl_device = (__bridge id<MTLDevice>)input_mtl_device_ptr;

	// Export the Metal device from Vulkan using VK_EXT_metal_objects.
	VkExportMetalDeviceInfoEXT device_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_DEVICE_INFO_EXT,
	    .pNext = NULL,
	    .mtlDevice = NULL,
	};

	VkExportMetalObjectsInfoEXT export_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_OBJECTS_INFO_EXT,
	    .pNext = &device_info,
	};

	vk->vkExportMetalObjectsEXT(vk->device, &export_info);

	if (device_info.mtlDevice == NULL) {
		U_LOG_E("Failed to export MTLDevice from Vulkan");
		return XRT_ERROR_VULKAN;
	}

	id<MTLDevice> vk_mtl_device = (__bridge id<MTLDevice>)device_info.mtlDevice;

	// Check if the MTLDevices match.
	if (input_mtl_device != vk_mtl_device) {
		U_LOG_E("MTLDevice from input allocation collection does not match Vulkan MTLDevice");
		U_LOG_E("Input device: %p, Vulkan device: %p", input_mtl_device, vk_mtl_device);
		return XRT_ERROR_VULKAN;
	}

	// Get the swapchain info from the input allocation collection.
	struct xrt_swapchain_create_info info;
	xrt_ret = xrt_allocation_collection_get_property( //
	    xac_mtl,                                      //
	    XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO,       //
	    &info);                                       //
	if (xrt_ret != XRT_SUCCESS) {
		U_LOG_E("Failed to get swapchain info from input allocation collection");
		return xrt_ret;
	}

	uint32_t image_count = xac_mtl->image_count;

	// Get the Metal textures from the input allocation collection.
	void *textures_ptr[XRT_MAX_SWAPCHAIN_IMAGES];
	xrt_ret = xrt_allocation_collection_get_all( //
	    xac_mtl,                                 //
	    XRT_ALLOCATION_TYPE_METAL_TEXTURE,       //
	    sizeof(void *),                          //
	    textures_ptr);                           //
	if (xrt_ret != XRT_SUCCESS) {
		U_LOG_E("Failed to get Metal textures from input allocation collection");
		return xrt_ret;
	}

	// Allocate the image collection structure.
	mic = U_TYPED_CALLOC(struct mtl_image_collection);
	if (mic == NULL) {
		U_LOG_E("Failed to allocate mtl_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure.
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 3;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_VK_BUNDLE;
	mic->base.supported_properties[2].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 2;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_VULKAN_IMAGE;
	mic->base.supported_types[1].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	mic->vk = vk;

	// Retain the Metal device.
	mic->device = [vk_mtl_device retain];

	// Store the swapchain creation info.
	mic->info = info;

	// Convert Vulkan format to Metal format for validation.
	VkFormat vk_format = (VkFormat)info.format;
	MTLPixelFormat mtl_format = (MTLPixelFormat)mtl_format_from_vk(vk_format);
	if (mtl_format == MTLPixelFormatInvalid) {
		U_LOG_E("Unsupported Vulkan format for Metal: %d", vk_format);
		xrt_ret = XRT_ERROR_VULKAN;
		goto err_cleanup;
	}

	VkImageUsageFlags image_usage = vk_csci_get_image_usage_flags(vk, vk_format, info.bits);
	if (image_usage == 0) {
		U_LOG_E("Unsupported swapchain usage flags");
		xrt_ret = XRT_ERROR_VULKAN;
		goto err_cleanup;
	}

	// Build Vulkan image create flags.
	VkImageCreateFlags image_create_flags = 0;
	if ((info.bits & XRT_SWAPCHAIN_USAGE_MUTABLE_FORMAT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
	}
	if ((info.create & XRT_SWAPCHAIN_CREATE_PROTECTED_CONTENT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_PROTECTED_BIT;
	}
	if (info.face_count == 6) {
		image_create_flags |= VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	}

	// Wrap each Metal texture as a Vulkan image.
	for (i = 0; i < image_count; i++) {
		id<MTLTexture> texture = (__bridge id<MTLTexture>)textures_ptr[i];

		// Retain the texture since we're taking ownership.
		mic->textures[i] = [texture retain];

		// Import the Metal texture into the Vulkan image.
		VkImageAspectFlagBits plane = (info.bits & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL)
		                                  ? VK_IMAGE_ASPECT_DEPTH_BIT
		                                  : VK_IMAGE_ASPECT_COLOR_BIT;

		VkImportMetalTextureInfoEXT import_texture_info = {
		    .sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT,
		    .pNext = NULL,
		    .plane = plane,
		    .mtlTexture = (__bridge MTLTexture_id)texture,
		};

		VkImageCreateInfo create_info = {
		    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
		    .pNext = &import_texture_info,
		    .flags = image_create_flags,
		    .imageType = VK_IMAGE_TYPE_2D,
		    .format = vk_format,
		    .extent = {.width = info.width, .height = info.height, .depth = 1},
		    .mipLevels = info.mip_count,
		    .arrayLayers = info.array_size * info.face_count,
		    .samples = VK_SAMPLE_COUNT_1_BIT,
		    .tiling = VK_IMAGE_TILING_OPTIMAL,
		    .usage = image_usage,
		    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
		    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
		};

		VkImage image = VK_NULL_HANDLE;
		vk_ret = vk->vkCreateImage(vk->device, &create_info, NULL, &image);
		VK_CHK_ONLY_PRINT(vk_ret, "vkCreateImage");
		if (vk_ret != VK_SUCCESS) {
			U_LOG_E("Failed to wrap Metal texture %d as Vulkan image", i);
			xrt_ret = XRT_ERROR_VULKAN;
			goto err_images;
		}

		mic->images[i] = image;
	}

	*out_xac = &mic->base;

	return XRT_SUCCESS;

err_images:
	// Clean up any successfully created images.
	for (uint32_t j = 0; j < i; j++) {
		if (mic->images[j] != VK_NULL_HANDLE) {
			vk->vkDestroyImage(vk->device, mic->images[j], NULL);
			mic->images[j] = VK_NULL_HANDLE;
		}
	}

err_cleanup:
	// Clean up any retained textures.
	for (uint32_t j = 0; j < image_count; j++) {
		if (mic->textures[j] != nil) {
			[mic->textures[j] release];
			mic->textures[j] = nil;
		}
	}

	if (mic->device != nil) {
		[mic->device release];
		mic->device = nil;
	}

	free(mic);

	return xrt_ret;
}

xrt_result_t
mtl_image_collection_create_shared_from_vk(struct vk_bundle *vk,
                                            const struct xrt_swapchain_create_info *xscci,
                                            uint32_t image_count,
                                            struct xrt_allocation_collection **out_xac)
{
	struct mtl_image_collection *mic = NULL;
	VkResult vk_ret;
	xrt_result_t xrt_ret;
	uint32_t i;

	assert(vk != NULL);
	assert(xscci != NULL);
	assert(xscci->format != 0);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	if (!vk->has_EXT_metal_objects) {
		U_LOG_E("VK_EXT_metal_objects extension not enabled");
		return XRT_ERROR_VULKAN;
	}

	// Export the Metal device from Vulkan using VK_EXT_metal_objects.
	VkExportMetalDeviceInfoEXT device_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_DEVICE_INFO_EXT,
	    .pNext = NULL,
	    .mtlDevice = NULL,
	};

	VkExportMetalObjectsInfoEXT export_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_METAL_OBJECTS_INFO_EXT,
	    .pNext = &device_info,
	};

	vk->vkExportMetalObjectsEXT(vk->device, &export_info);

	if (device_info.mtlDevice == NULL) {
		U_LOG_E("Failed to export MTLDevice from Vulkan");
		return XRT_ERROR_VULKAN;
	}

	id<MTLDevice> mtl_device = (__bridge id<MTLDevice>)device_info.mtlDevice;

	// Convert Vulkan format to Metal format.
	VkFormat vk_format = (VkFormat)xscci->format;
	MTLPixelFormat mtl_format = (MTLPixelFormat)mtl_format_from_vk(vk_format);
	if (mtl_format == MTLPixelFormatInvalid) {
		U_LOG_E("Unsupported Vulkan format for Metal: %d", vk_format);
		return XRT_ERROR_VULKAN;
	}

	VkImageUsageFlags image_usage = vk_csci_get_image_usage_flags(vk, vk_format, xscci->bits);
	if (image_usage == 0) {
		U_LOG_E("Unsupported swapchain usage flags");
		return XRT_ERROR_VULKAN;
	}

	// Compute Metal texture usage from swapchain bits.
	MTLTextureUsage mtl_usage = 0;
	if (xscci->bits & XRT_SWAPCHAIN_USAGE_COLOR) {
		mtl_usage |= MTLTextureUsageRenderTarget;
	}
	if (xscci->bits & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL) {
		mtl_usage |= MTLTextureUsageRenderTarget;
	}
	if (xscci->bits & XRT_SWAPCHAIN_USAGE_SAMPLED) {
		mtl_usage |= MTLTextureUsageShaderRead;
	}
	if (xscci->bits & XRT_SWAPCHAIN_USAGE_UNORDERED_ACCESS) {
		mtl_usage |= MTLTextureUsageShaderWrite;
	}
	// Always allow shader read for flexibility.
	mtl_usage |= MTLTextureUsageShaderRead;

	// Allocate the image collection structure.
	mic = U_TYPED_CALLOC(struct mtl_image_collection);
	if (mic == NULL) {
		U_LOG_E("Failed to allocate mtl_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure.
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 3;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_VK_BUNDLE;
	mic->base.supported_properties[2].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 3;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_VULKAN_IMAGE;
	mic->base.supported_types[1].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
	mic->base.supported_types[2].type = XRT_ALLOCATION_TYPE_NATIVE_IMAGE;
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	mic->vk = vk;
	mic->is_shared = true;

	// Retain the Metal device.
	mic->device = [mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Build Vulkan image create flags.
	VkImageCreateFlags image_create_flags = 0;
	if ((xscci->bits & XRT_SWAPCHAIN_USAGE_MUTABLE_FORMAT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
	}
	if ((xscci->create & XRT_SWAPCHAIN_CREATE_PROTECTED_CONTENT) != 0) {
		image_create_flags |= VK_IMAGE_CREATE_PROTECTED_BIT;
	}
	if (xscci->face_count == 6) {
		image_create_flags |= VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	}

	// Create shared Metal textures and import them into Vulkan images.
	for (i = 0; i < image_count; i++) {
		// Create a shared Metal texture via mtl_shared_texture_create.
		void *texture_ptr = mtl_shared_texture_create( //
		    (__bridge void *)mic->device,              //
		    xscci->width,                              //
		    xscci->height,                             //
		    xscci->array_size > 0 ? xscci->array_size : 1, //
		    xscci->sample_count > 0 ? xscci->sample_count : 1, //
		    xscci->mip_count > 0 ? xscci->mip_count : 1, //
		    (uint64_t)mtl_format,                      //
		    (uint64_t)mtl_usage);                      //
		if (texture_ptr == NULL) {
			U_LOG_E("Failed to create shared Metal texture %u", i);
			xrt_ret = XRT_ERROR_VULKAN;
			goto err_images;
		}

		mic->textures[i] = (id<MTLTexture>)texture_ptr;

		// Import the Metal texture into the Vulkan image.
		VkImageAspectFlagBits plane = (xscci->bits & XRT_SWAPCHAIN_USAGE_DEPTH_STENCIL)
		                                  ? VK_IMAGE_ASPECT_DEPTH_BIT
		                                  : VK_IMAGE_ASPECT_COLOR_BIT;

		VkImportMetalTextureInfoEXT import_texture_info = {
		    .sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT,
		    .pNext = NULL,
		    .plane = plane,
		    .mtlTexture = (__bridge MTLTexture_id)mic->textures[i],
		};

		VkImageCreateInfo create_info = {
		    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
		    .pNext = &import_texture_info,
		    .flags = image_create_flags,
		    .imageType = VK_IMAGE_TYPE_2D,
		    .format = vk_format,
		    .extent = {.width = xscci->width, .height = xscci->height, .depth = 1},
		    .mipLevels = xscci->mip_count,
		    .arrayLayers = xscci->array_size * xscci->face_count,
		    .samples = VK_SAMPLE_COUNT_1_BIT,
		    .tiling = VK_IMAGE_TILING_OPTIMAL,
		    .usage = image_usage,
		    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
		    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
		};

		VkImage image = VK_NULL_HANDLE;
		vk_ret = vk->vkCreateImage(vk->device, &create_info, NULL, &image);
		VK_CHK_ONLY_PRINT(vk_ret, "vkCreateImage");
		if (vk_ret != VK_SUCCESS) {
			U_LOG_E("Failed to import shared Metal texture %u as Vulkan image", i);
			xrt_ret = XRT_ERROR_VULKAN;
			goto err_images;
		}

		mic->images[i] = image;
	}

	*out_xac = &mic->base;

	return XRT_SUCCESS;

err_images:
	mtl_image_collection_fini(mic);
	free(mic);

	return xrt_ret;
}

xrt_result_t
mtl_image_collection_import_from_port(void *mtl_device,
                                       const struct xrt_swapchain_create_info *xscci,
                                       const mach_port_t *ports,
                                       uint32_t image_count,
                                       struct xrt_allocation_collection **out_xac)
{
	struct mtl_image_collection *mic = NULL;
	uint32_t i;

	assert(mtl_device != NULL);
	assert(xscci != NULL);
	assert(xscci->format != 0);
	assert(ports != NULL);
	assert(out_xac != NULL);
	assert(image_count > 0);
	assert(image_count <= XRT_MAX_SWAPCHAIN_IMAGES);

	// Allocate the image collection structure.
	mic = U_TYPED_CALLOC(struct mtl_image_collection);
	if (mic == NULL) {
		U_LOG_E("Failed to allocate mtl_image_collection");
		return XRT_ERROR_ALLOCATION;
	}

	// Initialize base structure for pure Metal (no Vulkan).
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 2;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 1;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	// No Vulkan bundle for pure Metal.
	mic->vk = NULL;
	mic->is_shared = false;

	// Retain the Metal device.
	mic->device = [(id<MTLDevice>)mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Import textures from Mach ports.
	for (i = 0; i < image_count; i++) {
		void *texture_ptr = mtl_shared_texture_from_mach_port(mtl_device, ports[i]);
		if (texture_ptr == NULL) {
			U_LOG_E("Failed to import shared texture from Mach port %u", i);
			goto err_textures;
		}

		mic->textures[i] = (id<MTLTexture>)texture_ptr;
	}

	// All imports succeeded, deallocate send rights.
	for (i = 0; i < image_count; i++) {
		mach_port_deallocate(mach_task_self(), ports[i]);
	}

	*out_xac = &mic->base;

	return XRT_SUCCESS;

err_textures:
	// On failure, do NOT deallocate any ports (caller retains ownership).
	mtl_image_collection_fini(mic);
	free(mic);

	return XRT_ERROR_VULKAN;
}
