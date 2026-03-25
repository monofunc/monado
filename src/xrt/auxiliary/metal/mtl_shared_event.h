// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  MTLSharedEvent Mach port conversion for cross-process GPU sync.
 * @author Mono
 * @ingroup aux_metal
 */

#pragma once

#include "xrt/xrt_config_os.h"

#ifndef XRT_OS_OSX
#error "This header is only for macOS"
#endif

#include <mach/mach_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Export an MTLSharedEvent to a Mach port send right.
 *
 * @param mtl_shared_event  MTLSharedEvent as void*.
 * @return Mach send right, or MACH_PORT_NULL on failure. Caller owns.
 */
mach_port_t
mtl_shared_event_to_mach_port(void *mtl_shared_event);

/*!
 * Import an MTLSharedEvent from a Mach port.
 *
 * @param mtl_device  MTLDevice as void*.
 * @param port        Mach send right for the shared event.
 * @return Retained MTLSharedEvent as void*, or NULL on failure. Caller owns.
 */
void *
mtl_shared_event_from_mach_port(void *mtl_device, mach_port_t port);

/*!
 * Release a retained MTLSharedEvent.
 */
void
mtl_shared_event_release(void *mtl_shared_event);

#ifdef __cplusplus
}
#endif
