// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining a Metal graphics interface.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_config_os.h"


#ifdef __cplusplus
extern "C" {
#endif

#if defined(XRT_OS_OSX) || defined(XRT_DOXYGEN)

// Forward declarations.
struct xrt_compositor_metal;
struct xrt_compositor_native;

/*!
 * Get the default Metal device for this system.
 *
 * Reference counting note:
 *
 * MTLCreateSystemDefaultDevice returns a retained device, and this function
 * transfers ownership of this reference to the caller.
 *
 * Callers:
 * - MUST call [device release] when done with the device.
 * - MAY call [device retain] to extend the lifetime.
 *
 * @param[out] out_ptr  Pointer to receive the Metal device (id<MTLDevice>).
 *
 * @ingroup xrt_iface
 */
void
xrt_gfx_metal_get_device(void **out_ptr);

/*!
 * Create a Metal client compositor.
 *
 * @param xcn            Native compositor to wrap, not owned by the provider.
 * @param command_queue  Metal command queue from the application (id<MTLCommandQueue>).
 *
 * @ingroup xrt_iface
 * @public @memberof xrt_compositor_native
 */
struct xrt_compositor_metal *
xrt_gfx_metal_provider_create(struct xrt_compositor_native *xcn, void *command_queue);

#endif // XRT_OS_OSX || XRT_DOXYGEN

#ifdef __cplusplus
}
#endif
