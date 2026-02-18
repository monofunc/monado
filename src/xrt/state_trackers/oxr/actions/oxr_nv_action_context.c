// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief NV action context (XR_NVX1_action_context) implementation.
 * @ingroup oxr_main
 */

#include "util/u_misc.h"

#include "oxr_nv_action_context.h"
#include "oxr_input.h"

#include "../oxr_api_verify.h"
#include "../oxr_chain.h"
#include "../oxr_handle.h"
#include "../oxr_logger.h"
#include "../oxr_objects.h"

#include <stdlib.h>

#ifdef OXR_HAVE_NVX1_action_context

static XrResult
instance_action_context_nv_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_instance_action_context_nv *ctx = (struct oxr_instance_action_context_nv *)hb;

	oxr_refcounted_unref(&ctx->context->base);
	ctx->context = NULL;

	return XR_SUCCESS;
}

XrResult
oxr_instance_action_context_nv_create(struct oxr_logger *log,
                                      struct oxr_instance *inst,
                                      const XrInstanceActionContextCreateInfoNV *createInfo,
                                      struct oxr_instance_action_context_nv **out_ctx)
{
	(void)createInfo;
	struct oxr_instance_action_context_nv *ctx = NULL;
	OXR_ALLOCATE_HANDLE_OR_RETURN(log, ctx, OXR_XR_DEBUG_INSTANCE_ACTION_CONTEXT_NV,
	                              instance_action_context_nv_destroy_cb, &inst->handle);
	ctx->inst = inst;
	ctx->immutable = false;
	XrResult ret = oxr_instance_action_context_create(log, &ctx->context);
	if (ret != XR_SUCCESS) {
		oxr_handle_destroy(log, &ctx->handle);
		return ret;
	}
	*out_ctx = ctx;
	return XR_SUCCESS;
}

XrResult
oxr_instance_action_context_nv_destroy(struct oxr_logger *log, struct oxr_instance_action_context_nv *ctx)
{
	return oxr_handle_destroy(log, &ctx->handle);
}

static XrResult
session_action_context_nv_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_session_action_context_nv *ctx = (struct oxr_session_action_context_nv *)hb;
	if (ctx->inst_context != NULL) {
		oxr_refcounted_unref(&ctx->inst_context->base);
		ctx->inst_context = NULL;
	}
	oxr_session_action_context_fini(&ctx->context);
	return XR_SUCCESS;
}

XrResult
oxr_session_action_context_nv_create(struct oxr_logger *log,
                                     struct oxr_session *sess,
                                     const XrSessionActionContextCreateInfoNV *createInfo,
                                     struct oxr_instance_action_context_nv *inst_ctx_nv,
                                     struct oxr_session_action_context_nv **out_ctx)
{
	(void)createInfo;
	struct oxr_instance_action_context *inst_context = inst_ctx_nv->context;
	struct oxr_session_action_context_nv *ctx = NULL;
	OXR_ALLOCATE_HANDLE_OR_RETURN(log, ctx, OXR_XR_DEBUG_SESSION_ACTION_CONTEXT_NV,
	                              session_action_context_nv_destroy_cb, &sess->handle);
	ctx->sess = sess;
	oxr_refcounted_ref(&inst_context->base);
	ctx->inst_context = inst_context;

	XrResult ret = oxr_session_action_context_init(&ctx->context);
	if (ret != XR_SUCCESS) {
		oxr_handle_destroy(log, &ctx->handle);
		return oxr_error(log, ret, "Failed to init internal action context!");
	}

	inst_ctx_nv->immutable = true;
	*out_ctx = ctx;

	// The oxr_session_success_result call handled by caller.
	return XR_SUCCESS;
}

XrResult
oxr_session_action_context_nv_destroy(struct oxr_logger *log, struct oxr_session_action_context_nv *ctx)
{
	return oxr_handle_destroy(log, &ctx->handle);
}

XrResult
oxr_resolve_instance_action_context_from_chain(struct oxr_logger *log,
                                               struct oxr_instance *inst,
                                               const void *chain_next,
                                               struct oxr_instance_action_context **out_inst_context,
                                               struct oxr_instance_action_context_nv **out_nv_ctx)
{
	const XrInstanceActionContextInfoNV *info = OXR_GET_INPUT_FROM_CHAIN(
	    chain_next, XR_TYPE_INSTANCE_ACTION_CONTEXT_INFO_NV, XrInstanceActionContextInfoNV);
	if (info == NULL || info->instanceActionContext == XR_NULL_HANDLE) {
		*out_inst_context = inst->action_context;
		*out_nv_ctx = NULL;
		return XR_SUCCESS;
	}

	struct oxr_instance_action_context_nv *nv_ctx = NULL;
	OXR_VERIFY_INSTANCE_ACTION_CONTEXT_NV_AND_INIT_LOG(log, info->instanceActionContext, nv_ctx,
	                                                   "instanceActionContext");
	if (nv_ctx->inst != inst) {
		return oxr_error(log, XR_ERROR_HANDLE_INVALID, "instanceActionContext belongs to a different instance");
	}

	*out_inst_context = nv_ctx->context;
	*out_nv_ctx = nv_ctx;

	return XR_SUCCESS;
}

XrResult
oxr_resolve_session_action_context_from_chain(struct oxr_logger *log,
                                              struct oxr_session *sess,
                                              const void *chain_next,
                                              struct oxr_instance_action_context **out_inst_context,
                                              struct oxr_session_action_context **out_sess_context,
                                              struct oxr_session_action_context_nv **out_sess_ctx_nv)
{
	const XrSessionActionContextInfoNV *info =
	    OXR_GET_INPUT_FROM_CHAIN(chain_next, XR_TYPE_SESSION_ACTION_CONTEXT_INFO_NV, XrSessionActionContextInfoNV);
	if (info == NULL || info->sessionActionContext == XR_NULL_HANDLE) {
		*out_sess_context = &sess->action_context;
		*out_inst_context = sess->sys->inst->action_context;
		*out_sess_ctx_nv = NULL;
		return XR_SUCCESS;
	}

	struct oxr_session_action_context_nv *sess_ctx_nv = NULL;
	OXR_VERIFY_SESSION_ACTION_CONTEXT_NV_AND_INIT_LOG(log, info->sessionActionContext, sess_ctx_nv,
	                                                  "sessionActionContext");
	if (sess_ctx_nv->sess != sess) {
		return oxr_error(log, XR_ERROR_HANDLE_INVALID, "sessionActionContext belongs to a different session");
	}

	*out_sess_context = &sess_ctx_nv->context;
	*out_inst_context = sess_ctx_nv->inst_context;
	*out_sess_ctx_nv = sess_ctx_nv;

	return XR_SUCCESS;
}

#endif // OXR_HAVE_NVX1_action_context
