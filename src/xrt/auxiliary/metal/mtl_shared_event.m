// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  MTLSharedEvent Mach port conversion for cross-process GPU sync.
 * @author Mono
 * @ingroup aux_metal
 *
 */

#import <Metal/Metal.h>
#import <mach/mach.h>

#include "util/u_logging.h"
#include "mtl_shared_event.h"


/*
 * Private SPI for MTLSharedEvent Mach port transport.
 */

@protocol MTLDeviceSPI <MTLDevice>
- (id<MTLSharedEvent>)newSharedEventWithMachPort:(mach_port_t)machPort;
@end

@interface MTLSharedEventHandle ()
- (mach_port_t)eventPort;
@end


mach_port_t
mtl_shared_event_to_mach_port(void *mtl_shared_event)
{
	if (mtl_shared_event == NULL) {
		return MACH_PORT_NULL;
	}

	id<MTLSharedEvent> event = (id<MTLSharedEvent>)mtl_shared_event;
	MTLSharedEventHandle *handle = [event newSharedEventHandle];
	if (handle == nil) {
		U_LOG_E("mtl_shared_event_to_mach_port: newSharedEventHandle failed");
		return MACH_PORT_NULL;
	}

	// eventPort returns a send right owned by the handle.
	// Add our own reference before releasing the handle.
	mach_port_t port = [handle eventPort];
	if (port == MACH_PORT_NULL) {
		U_LOG_E("mtl_shared_event_to_mach_port: eventPort failed");
		[handle release];
		return MACH_PORT_NULL;
	}

	kern_return_t kr = mach_port_mod_refs(mach_task_self(), port, MACH_PORT_RIGHT_SEND, 1);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mtl_shared_event_to_mach_port: mach_port_mod_refs failed: %d", kr);
		[handle release];
		return MACH_PORT_NULL;
	}

	[handle release];
	return port;
}

void *
mtl_shared_event_from_mach_port(void *mtl_device, mach_port_t port)
{
	if (mtl_device == NULL || port == MACH_PORT_NULL) {
		U_LOG_E("mtl_shared_event_from_mach_port: invalid parameters");
		return NULL;
	}

	id<MTLDeviceSPI> device = (id<MTLDeviceSPI>)mtl_device;
	id<MTLSharedEvent> event = [device newSharedEventWithMachPort:port];
	if (event == nil) {
		U_LOG_E("mtl_shared_event_from_mach_port: newSharedEventWithMachPort failed");
		return NULL;
	}

	return (void *)event;
}

void
mtl_shared_event_release(void *mtl_shared_event)
{
	if (mtl_shared_event != NULL) {
		[(id)mtl_shared_event release];
	}
}
