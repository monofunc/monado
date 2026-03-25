// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  MTLSharedTextureHandle helpers for cross-process texture sharing.
 * @author Mono
 * @ingroup aux_metal
 */

#pragma once

#include "xrt/xrt_config_os.h"

#ifndef XRT_OS_OSX
#error "This header is only for macOS"
#endif

#include <mach/mach_types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Create a shareable MTLTexture for cross-process sharing.
 *
 * Uses [MTLDevice newSharedTextureWithDescriptor:] with MTLStorageModePrivate.
 *
 * @param mtl_device       MTLDevice as void*.
 * @param width            Texture width in pixels.
 * @param height           Texture height in pixels.
 * @param array_length     Number of array layers (1 for 2D, >1 for 2DArray).
 * @param sample_count     Number of samples (1 for non-MSAA).
 * @param mip_count        Number of mipmap levels.
 * @param mtl_pixel_format MTLPixelFormat as uint64_t.
 * @param mtl_texture_usage MTLTextureUsage as uint64_t.
 * @return Retained MTLTexture as void*, or NULL on failure. Caller owns.
 */
void *
mtl_shared_texture_create(void *mtl_device,
                          uint32_t width,
                          uint32_t height,
                          uint32_t array_length,
                          uint32_t sample_count,
                          uint32_t mip_count,
                          uint64_t mtl_pixel_format,
                          uint64_t mtl_texture_usage);

/*!
 * Export a shareable MTLTexture to a Mach port send right.
 *
 * @param mtl_texture  Shareable MTLTexture as void*.
 * @return Mach send right, or MACH_PORT_NULL on failure. Caller owns.
 */
mach_port_t
mtl_shared_texture_to_mach_port(void *mtl_texture);

/*!
 * Import a shared texture from a Mach port.
 *
 * @param mtl_device  MTLDevice as void*.
 * @param port        Mach send right for the shared texture handle.
 * @return Retained MTLTexture as void*, or NULL on failure. Caller owns.
 */
void *
mtl_shared_texture_from_mach_port(void *mtl_device, mach_port_t port);

/*!
 * Release a retained MTLTexture.
 */
void
mtl_shared_texture_release(void *mtl_texture);

#ifdef __cplusplus
}
#endif
