// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Circular doubly linked list
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_compiler.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * TODO
 *
 * See https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/include/linux/list.h
 *
 * @ingroup xrt_iface
 */
struct xrt_list
{
	//! Previous element of the list
	struct xrt_list *prev;
	//! Next element of the list
	struct xrt_list *next;
};

static inline void
xrt_list_init(struct xrt_list *list)
{
	list->prev = list;
	list->next = list;
}

static inline void
xrt_list_insert(struct xrt_list *list, struct xrt_list *element)
{
	element->prev = list;
	element->next = list->next;
	list->next = element;
	element->next->prev = element;
}

static inline void
xrt_list_remove(struct xrt_list *element)
{
	if (element->next == NULL && element->prev == NULL) {
		return;
	}

	element->prev->next = element->next;
	element->next->prev = element->prev;
	element->next = NULL;
	element->prev = NULL;
}

static inline size_t
xrt_list_length(const struct xrt_list *list)
{
	size_t count = 0;

	struct xrt_list *e = list->next;
	while (e != list) {
		e = e->next;
		count++;
	}

	return count;
}

static inline bool
xrt_list_empty(const struct xrt_list *list)
{
	return list->next == list;
}

#define XRT_LIST_FOR_EACH(pos, head, member)                                                                           \
	for (pos = container_of((head)->next, XRT_TYPEOF(*(pos)), member); &pos->member != (head);                                \
	     pos = container_of(pos->member.next, XRT_TYPEOF(*(pos), member))

#define XRT_LIST_FOR_EACH_SAFE(pos, tmp, head, member)                                                                 \
	for (pos = container_of((head)->next, XRT_TYPEOF(*(pos)), member),                                             \
	    tmp = container_of((pos)->member.next, XRT_TYPEOF(*(tmp)), member);                                        \
	     &pos->member != (head); pos = tmp, tmp = container_of(pos->member.next, XRT_TYPEOF(*(tmp)), member))

#define XRT_LIST_FOR_EACH_REVERSE(pos, head, member)                                                                   \
	for (pos = container_of((head)->prev, XRT_TYPEOF(*(pos)), member); &pos->member != (head);                     \
	     pos = container_of(pos->member.prev, XRT_TYPEOF(*(pos)), member))

#define XRT_LIST_FOR_EACH_REVERSE_SAFE(pos, tmp, head, member)                                                         \
	for (pos = container_of((head)->prev, XRT_TYPEOF(*(pos)), member),                                             \
	    tmp = container_of((pos)->member.prev, XRT_TYPEOF(*(tmp)), member);                                        \
	     &pos->member != (head); pos = tmp, tmp = container_of(pos->member.prev, XRT_TYPEOF(*(tmp)), member))

#ifdef __cplusplus
} // extern "C"
#endif
