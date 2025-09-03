// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Source of an observable event
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_list.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_listener;

typedef void (*xrt_listener_notify_fn)(struct xrt_listener *listener, void *data);

struct xrt_listener
{
	//! Part of xrt_signal::listeners
	struct xrt_list link;

	//! Callback function
	xrt_listener_notify_fn notify;
};

struct xrt_signal
{
	//! Contains xrt_listener
	struct xrt_list listeners;
};

static inline void
xrt_signal_init(struct xrt_signal *signal)
{
	xrt_list_init(&signal->listeners);
}

static inline void
xrt_signal_register(struct xrt_signal *signal, struct xrt_listener *listener)
{
	xrt_list_insert(signal->listeners.prev, &listener->link);
}

static inline void
xrt_signal_emit(const struct xrt_signal *signal, void *data)
{
	struct xrt_listener *listener, *tmp;
	XRT_LIST_FOR_EACH_SAFE(listener, tmp, &signal->listeners, link)
	{
		listener->notify(listener, data);
	}
}

#ifdef __cplusplus
} // extern "C"
#endif
