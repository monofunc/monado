// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal image collection for Vulkan/Metal interop.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_metal
 */

#import <Metal/Metal.h>
#import <IOSurface/IOSurface.h>

#include "xrt/xrt_config_vulkan.h"

#include "util/u_misc.h"
#include "util/u_logging.h"

#include "vk/vk_helpers.h"

#include "mtl_format.h"
#include "mtl_image_collection.h"


/*
 *
 * Structs.
 *
 */

// Not used for now.
#undef USE_IOSURFACE

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

	//! Metal textures backed by IOSurfaces.
	id<MTLTexture> textures[XRT_MAX_SWAPCHAIN_IMAGES];

	//! IOSurfaces backing the Metal textures.
	IOSurfaceRef surfaces[XRT_MAX_SWAPCHAIN_IMAGES];

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
 * Get the bytes per pixel for a Metal pixel format.
 */
static uint32_t
get_bytes_per_pixel(MTLPixelFormat format)
{
	switch (format) {
	case MTLPixelFormatBGRA8Unorm:
	case MTLPixelFormatBGRA8Unorm_sRGB:
	case MTLPixelFormatRGBA8Unorm:
	case MTLPixelFormatRGBA8Unorm_sRGB:
	case MTLPixelFormatRGB10A2Unorm:
	case MTLPixelFormatDepth32Float: return 4;
	case MTLPixelFormatRGBA16Float:
	case MTLPixelFormatDepth32Float_Stencil8: return 8;
	default:
		// Default to 4 bytes per pixel.
		return 4;
	}
}

/*!
 * Get the IOSurface pixel format for a Metal pixel format.
 * Returns 0 if the format is not supported.
 */
static uint32_t
get_iosurface_pixel_format(MTLPixelFormat format)
{
	switch (format) {
	case MTLPixelFormatBGRA8Unorm:
	case MTLPixelFormatBGRA8Unorm_sRGB: return 'BGRA'; // kCVPixelFormatType_32BGRA
	case MTLPixelFormatRGBA8Unorm:
	case MTLPixelFormatRGBA8Unorm_sRGB: return 'RGBA'; // kCVPixelFormatType_32RGBA
	case MTLPixelFormatRGBA16Float: return 'RGhA';     // kCVPixelFormatType_64RGBAHalf
	case MTLPixelFormatRGB10A2Unorm: return 'l10r';    // kCVPixelFormatType_ARGB2101010LEPacked (closest match)
	default:
		// For formats without a direct IOSurface equivalent, return 0.
		// The caller should handle this appropriately.
		return 0;
	}
}

/*!
 * Create an IOSurface with the given properties.
 */
XRT_MAYBE_UNUSED static IOSurfaceRef
create_iosurface(const struct xrt_swapchain_create_info *info, MTLPixelFormat mtl_format)
{
	uint32_t bytes_per_pixel = get_bytes_per_pixel(mtl_format);
	uint32_t bytes_per_row = info->width * bytes_per_pixel;
	uint32_t pixel_format = get_iosurface_pixel_format(mtl_format);

	// Build IOSurface properties dictionary.
	NSDictionary *properties = @{
		(id)kIOSurfaceWidth : @(info->width),
		(id)kIOSurfaceHeight : @(info->height),
		(id)kIOSurfaceBytesPerElement : @(bytes_per_pixel),
		(id)kIOSurfaceBytesPerRow : @(bytes_per_row),
		(id)kIOSurfaceAllocSize : @(bytes_per_row * info->height),
		// Note: For some formats like depth, there's no matching IOSurface pixel format.
		// We only set it if we have a valid mapping.
		(id)kIOSurfacePixelFormat : pixel_format != 0 ? @(pixel_format) : @(0),
	};

	IOSurfaceRef surface = IOSurfaceCreate((__bridge CFDictionaryRef)properties);

	return surface;
}

/*!
 * Create a Metal texture descriptor and IOSurface-backed texture from swapchain create info.
 * This is the pure Metal version without any Vulkan dependencies.
 *
 * @param mtl_device    The Metal device to create the texture on.
 * @param info          Swapchain creation info.
 * @param mtl_format    The Metal pixel format.
 * @param out_texture   Output: the created Metal texture (retained).
 * @param out_surface   Output: the backing IOSurface (retained).
 * @return true on success, false on failure.
 */
static bool
create_texture_for_metal(id<MTLDevice> mtl_device,
                         const struct xrt_swapchain_create_info *info,
                         MTLPixelFormat mtl_format,
                         id<MTLTexture> *out_texture,
                         IOSurfaceRef *out_surface)
{
#if defined(USE_IOSURFACE)
	// Create the IOSurface that will back the texture.
	IOSurfaceRef surface = create_iosurface(info, mtl_format);
	if (surface == NULL) {
		U_LOG_E("Failed to create IOSurface");
		return false;
	}
#else
	IOSurfaceRef surface = NULL;
#endif

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
	// IOSurface-backed textures must use MTLStorageModeShared on macOS.
	desc.storageMode = MTLStorageModeShared;

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

#if defined(USE_IOSURFACE)
	// Create the Metal texture from the IOSurface.
	id<MTLTexture> texture = [mtl_device newTextureWithDescriptor:desc iosurface:surface plane:0];
#else
	// Create the Metal texture from the IOSurface.
	id<MTLTexture> texture = [mtl_device newTextureWithDescriptor:desc];
#endif
	[desc release];

	if (texture == nil) {
		U_LOG_E("Failed to create Metal texture from IOSurface");
		CFRelease(surface);
		return false;
	}

	*out_texture = texture;
	*out_surface = surface;

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
                       id<MTLTexture> *out_texture,
                       IOSurfaceRef *out_surface)
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

	// Create the IOSurface-backed Metal texture using the shared helper.
	id<MTLTexture> texture = nil;
	IOSurfaceRef surface = NULL;
	if (!create_texture_for_metal(mtl_device, info, mtl_format, &texture, &surface)) {
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
	// Per the Vulkan spec, the pNext chain must include one VkImportMetalTextureInfoEXT
	// structure for each plane in the VkImage. For non-planar images, use VK_IMAGE_ASPECT_COLOR_BIT.
	VkImportMetalTextureInfoEXT import_texture_info = {
	    .sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT,
	    .pNext = NULL,
	    .plane = VK_IMAGE_ASPECT_COLOR_BIT,
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
		CFRelease(surface);
		return ret;
	}

	*out_texture = texture;
	*out_image = image;
	*out_surface = surface;

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

	// Release IOSurfaces backing the Metal textures.
	for (uint32_t i = 0; i < mic->base.image_count; i++) {
		if (mic->surfaces[i] == NULL) {
			continue;
		}

		CFRelease(mic->surfaces[i]);
		mic->surfaces[i] = NULL;
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
	case XRT_ALLOCATION_TYPE_IOSURFACE: {
		if (element_size != sizeof(IOSurfaceRef)) {
			U_LOG_E("Invalid element size for IOSurfaceRef: %zu (expected %zu)", element_size,
			        sizeof(IOSurfaceRef));
			return XRT_ERROR_IPC_FAILURE;
		}
		IOSurfaceRef *surfaces = (IOSurfaceRef *)ptr;
		// See XRT_ALLOCATION_TYPE_IOSURFACE for reference semantics.
		for (uint32_t i = 0; i < mic->base.image_count; i++) {
			surfaces[i] = mic->surfaces[i];
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

	// Initialize base structure for pure Metal (MTLTexture and IOSurface supported).
	mic->base.reference.count = 1;
	mic->base.image_count = image_count;
	mic->base.supported_properties_count = 2;
	mic->base.supported_properties[0].property = XRT_ALLOCATION_PROPERTY_SWAPCHAIN_INFO;
	mic->base.supported_properties[1].property = XRT_ALLOCATION_PROPERTY_METAL_DEVICE;
	mic->base.supported_types_count = 1;
	mic->base.supported_types[0].type = XRT_ALLOCATION_TYPE_METAL_TEXTURE;
#if defined(USE_IOSURFACE)
	mic->base.supported_types[mic->base.supported_types_count++].type = XRT_ALLOCATION_TYPE_IOSURFACE;
#endif
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	// No Vulkan bundle for pure Metal.
	mic->vk = NULL;

	// Retain the Metal device.
	mic->device = [(id<MTLDevice>)mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Create IOSurface-backed Metal textures.
	for (i = 0; i < image_count; i++) {
		bool bret = create_texture_for_metal( //
		    mic->device,                      //
		    xscci,                            //
		    mtl_format,                       //
		    &mic->textures[i],                //
		    &mic->surfaces[i]);               //
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
#if defined(USE_IOSURFACE)
	mic->base.supported_types[mic->base.supported_types_count++].type = XRT_ALLOCATION_TYPE_IOSURFACE;
#endif
	mic->base.get_property = mtl_xac_get_property;
	mic->base.get_all = mtl_xac_get_all;
	mic->base.destroy = mtl_xac_destroy;

	mic->vk = vk;

	// Retain the Metal device.
	mic->device = [mtl_device retain];

	// Store the swapchain creation info.
	mic->info = *xscci;

	// Create IOSurface-backed Metal textures and import them into Vulkan images.
	for (i = 0; i < image_count; i++) {
		ret = create_image_for_metal( //
		    vk,                       //
		    mic->device,              //
		    xscci,                    //
		    &mic->images[i],          //
		    &mic->textures[i],        //
		    &mic->surfaces[i]);       //
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
#if defined(USE_IOSURFACE)
	mic->base.supported_types[mic->base.supported_types_count++].type = XRT_ALLOCATION_TYPE_IOSURFACE;
#endif
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
		VkImportMetalTextureInfoEXT import_texture_info = {
		    .sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT,
		    .pNext = NULL,
		    .plane = VK_IMAGE_ASPECT_COLOR_BIT,
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

		// Note: We don't retain IOSurfaces here because we don't have direct access to them
		// from the input allocation collection. The IOSurfaces are owned by the input
		// allocation collection and will be kept alive by it. This is fine because the
		// Metal textures we've retained keep a reference to their backing IOSurfaces.
		mic->surfaces[i] = NULL;
	}

	*out_xac = &mic->base;

	U_LOG_E("YES");
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
