// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  MTLSharedTextureHandle helpers for cross-process texture sharing.
 * @author Mono
 * @ingroup aux_metal
 *
 */

#import <Metal/Metal.h>
#import <Foundation/Foundation.h>

#include "util/u_logging.h"
#include "mtl_shared_texture.h"


/*
 * Private SPI for MTLSharedTextureHandle Mach port transport.
 */

@interface MTLSharedTextureHandle ()
- (mach_port_t)createMachPort;
- (MTLSharedTextureHandle *)initWithMachPort:(mach_port_t)port;
@end


void *
mtl_shared_texture_create(void *mtl_device,
                          uint32_t width,
                          uint32_t height,
                          uint32_t array_length,
                          uint32_t sample_count,
                          uint32_t mip_count,
                          uint64_t mtl_pixel_format,
                          uint64_t mtl_texture_usage)
{
	if (mtl_device == NULL || width == 0 || height == 0 || array_length == 0) {
		U_LOG_E("mtl_shared_texture_create: invalid parameters");
		return NULL;
	}

	if (mtl_pixel_format == 0) {
		U_LOG_E("mtl_shared_texture_create: MTLPixelFormatInvalid");
		return NULL;
	}

	id<MTLDevice> device = (id<MTLDevice>)mtl_device;

	MTLTextureDescriptor *desc = [[MTLTextureDescriptor alloc] init];
	desc.width = width;
	desc.height = height;
	desc.pixelFormat = (MTLPixelFormat)mtl_pixel_format;
	desc.storageMode = MTLStorageModePrivate;
	desc.sampleCount = sample_count > 0 ? sample_count : 1;
	desc.mipmapLevelCount = mip_count > 0 ? mip_count : 1;
	desc.usage = (MTLTextureUsage)mtl_texture_usage;

	if (array_length > 1) {
		desc.textureType = MTLTextureType2DArray;
		desc.arrayLength = array_length;
	} else {
		desc.textureType = MTLTextureType2D;
		desc.arrayLength = 1;
	}

	id<MTLTexture> texture = [device newSharedTextureWithDescriptor:desc];
	[desc release];

	if (texture == nil) {
		U_LOG_E("mtl_shared_texture_create: newSharedTextureWithDescriptor failed "
		        "(w=%u h=%u array=%u fmt=%llu)",
		        width, height, array_length, (unsigned long long)mtl_pixel_format);
		return NULL;
	}

	return (void *)texture;
}

mach_port_t
mtl_shared_texture_to_mach_port(void *mtl_texture)
{
	if (mtl_texture == NULL) {
		return MACH_PORT_NULL;
	}

	id<MTLTexture> texture = (id<MTLTexture>)mtl_texture;

	MTLSharedTextureHandle *handle = [texture newSharedTextureHandle];
	if (handle == nil) {
		U_LOG_E("mtl_shared_texture_to_mach_port: newSharedTextureHandle failed");
		return MACH_PORT_NULL;
	}

	mach_port_t port = [handle createMachPort];
	[handle release];

	if (port == MACH_PORT_NULL) {
		U_LOG_E("mtl_shared_texture_to_mach_port: createMachPort failed");
		return MACH_PORT_NULL;
	}

	return port;
}

void *
mtl_shared_texture_from_mach_port(void *mtl_device, mach_port_t port)
{
	if (mtl_device == NULL || port == MACH_PORT_NULL) {
		U_LOG_E("mtl_shared_texture_from_mach_port: invalid parameters");
		return NULL;
	}

	id<MTLDevice> device = (id<MTLDevice>)mtl_device;

	MTLSharedTextureHandle *handle = [[MTLSharedTextureHandle alloc] initWithMachPort:port];
	if (handle == nil) {
		U_LOG_E("mtl_shared_texture_from_mach_port: initWithMachPort failed");
		return NULL;
	}

	id<MTLTexture> texture = [device newSharedTextureWithHandle:handle];
	[handle release];

	if (texture == nil) {
		U_LOG_E("mtl_shared_texture_from_mach_port: newSharedTextureWithHandle failed");
		return NULL;
	}

	return (void *)texture;
}

void
mtl_shared_texture_release(void *mtl_texture)
{
	if (mtl_texture != NULL) {
		[(id)mtl_texture release];
	}
}
