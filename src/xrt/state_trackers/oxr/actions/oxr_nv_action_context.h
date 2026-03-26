// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief NV action context (XR_NVX1_action_context) internal types and helpers.
 * @ingroup oxr_main
 */

#pragma once

#include "oxr_extension_support.h"
#include "oxr_forward_declarations.h"
#include "oxr_instance_action_context.h"
#include "oxr_session_action_context.h"

#include "oxr_defines.h"
#include "oxr_objects.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef OXR_HAVE_NVX1_action_context

/*
 * Struct definitions.
 */

/*!
 * Instance-level action context handle. Action sets created with this context
 * are parented to this handle. Becomes immutable when a session action context
 * is created from it.
 *
 * @obj{XrInstanceActionContextNV}
 * @extends oxr_handle_base
 */
struct oxr_instance_action_context_nv
{
	struct oxr_handle_base handle;
	struct oxr_instance *inst;
	struct oxr_instance_action_context *context;
	bool immutable;
};

/*!
 * Session-level action context handle. Created with an instance action context;
 * at creation all action sets from that instance context are attached.
 *
 * @obj{XrSessionActionContextNV}
 * @extends oxr_handle_base
 */
struct oxr_session_action_context_nv
{
	struct oxr_handle_base handle;
	struct oxr_session *sess;
	struct oxr_session_action_context context;
	/*! Instance action context (reference-counted); ref taken at create, unref in destroy. */
	struct oxr_instance_action_context *inst_context;
};

/*
 * Helpers.
 */
static inline XrInstanceActionContextNV
oxr_instance_action_context_nv_to_openxr(struct oxr_instance_action_context_nv *ctx)
{
	return XRT_CAST_PTR_TO_OXR_HANDLE(XrInstanceActionContextNV, ctx);
}

static inline XrSessionActionContextNV
oxr_session_action_context_nv_to_openxr(struct oxr_session_action_context_nv *ctx)
{
	return XRT_CAST_PTR_TO_OXR_HANDLE(XrSessionActionContextNV, ctx);
}

/*
 * Internal create/destroy and attach-all.
 */
XrResult
oxr_instance_action_context_nv_create(struct oxr_logger *log,
                                      struct oxr_instance *inst,
                                      const XrInstanceActionContextCreateInfoNV *createInfo,
                                      struct oxr_instance_action_context_nv **out_ctx);

XrResult
oxr_instance_action_context_nv_destroy(struct oxr_logger *log, struct oxr_instance_action_context_nv *ctx);

XrResult
oxr_session_action_context_nv_create(struct oxr_logger *log,
                                     struct oxr_session *sess,
                                     const XrSessionActionContextCreateInfoNV *createInfo,
                                     struct oxr_instance_action_context_nv *inst_ctx_nv,
                                     struct oxr_session_action_context_nv **out_ctx);

XrResult
oxr_session_action_context_nv_destroy(struct oxr_logger *log, struct oxr_session_action_context_nv *ctx);

/*!
 * Resolve instance action context from chain (e.g. createInfo->next or
 * suggestedBindings->next). If chain contains XrInstanceActionContextInfoNV with
 * non-null instanceActionContext, verify and return that context; otherwise
 * return the default inst->action_context (and *out_nv_ctx = NULL).
 * When returning the default context, *out_inst_context is set to &inst->action_context.
 */
XrResult
oxr_resolve_instance_action_context_from_chain(struct oxr_logger *log,
                                               struct oxr_instance *inst,
                                               const void *chain_next,
                                               struct oxr_instance_action_context **out_inst_context,
                                               struct oxr_instance_action_context_nv **out_nv_ctx);

/*!
 * Resolve session action context from chain (e.g. XrSessionActionSetsAttachInfo::next).
 * If chain contains XrSessionActionContextInfoNV with non-null sessionActionContext,
 * verify and return that context; otherwise return the default sess->action_context.
 */
XrResult
oxr_resolve_session_action_context_from_chain(struct oxr_logger *log,
                                              struct oxr_session *sess,
                                              const void *chain_next,
                                              struct oxr_instance_action_context **out_inst_context,
                                              struct oxr_session_action_context **out_sess_context,
                                              struct oxr_session_action_context_nv **out_sess_ctx_nv);

#endif // OXR_HAVE_NVX1_action_context

#ifdef __cplusplus
}
#endif
