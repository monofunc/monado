// Copyright 2023-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Preview header for XR_MNDX_bypass_action_set_priorities extension.
 * @author galister <galister-dev@pm.me>
 * @ingroup external_openxr
 */
#ifndef XR_MNDX_BYPASS_ACTION_SET_PRIORITIES_H
#define XR_MNDX_BYPASS_ACTION_SET_PRIORITIES_H 1

#include "openxr_extension_helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

// Extension number 445 (444 prefix)
#define XR_MNDX_bypass_action_set_priorities 1
#define XR_MNDX_bypass_action_set_priorities_SPEC_VERSION 1
#define XR_MNDX_BYPASS_ACTION_SET_PRIORITIES_EXTENSION_NAME "XR_MNDX_bypass_action_set_priorities"

XR_STRUCT_ENUM(XR_TYPE_BYPASS_ACTION_SET_PRIORITIES_MNDX, 1000444006);

// XrBypassActionSetPrioritiesMNDX extends XrActionsSyncInfo
typedef struct XrBypassActionSetPrioritiesMNDX {
    XrStructureType                        type;
    const void* XR_MAY_ALIAS               next;
    uint32_t                               bypassPriorityActionSetCount;
    const XrActionSet*                     bypassPriorityActionSets;
} XrBypassActionSetPrioritiesMNDX;

#ifdef __cplusplus
}
#endif

#endif
