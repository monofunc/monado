// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining NV action context API functions (XR_NVX1_action_context).
 * @ingroup oxr_api
 */

#pragma once

#include "oxr_extension_support.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef OXR_HAVE_NVX1_action_context
//! OpenXR API function @ep{xrCreateInstanceActionContextNV}
XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrCreateInstanceActionContextNV(XrInstance instance,
                                    const XrInstanceActionContextCreateInfoNV *createInfo,
                                    XrInstanceActionContextNV *instanceActionContext);

//! OpenXR API function @ep{xrDestroyInstanceActionContextNV}
XRAPI_ATTR void XRAPI_CALL
oxr_xrDestroyInstanceActionContextNV(XrInstanceActionContextNV instanceActionContext);

//! OpenXR API function @ep{xrCreateSessionActionContextNV}
XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrCreateSessionActionContextNV(XrSession session,
                                   const XrSessionActionContextCreateInfoNV *createInfo,
                                   XrSessionActionContextNV *sessionActionContext);

//! OpenXR API function @ep{xrDestroySessionActionContextNV}
XRAPI_ATTR void XRAPI_CALL
oxr_xrDestroySessionActionContextNV(XrSessionActionContextNV sessionActionContext);

//! OpenXR API function @ep{xrSyncActions2NV}
XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrSyncActions2NV(XrSession session, const XrActionsSyncInfo2NV *syncInfo, XrActionsSyncState2NV *syncState);

//! OpenXR API function @ep{xrGetCurrentInteractionProfile2NV}
XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrGetCurrentInteractionProfile2NV(XrSession session,
                                      const XrInteractionProfileGetInfo2NV *getInfo,
                                      XrInteractionProfileState *state);
#endif


#ifdef __cplusplus
}
#endif
