// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  NV action context API functions (XR_NVX1_action_context).
 * @ingroup oxr_api
 */

#include "xrt/xrt_compiler.h"

#include "util/u_trace_marker.h"

#include "oxr_extension_support.h"
#include "oxr_api_nv_action_context.h"
#include "oxr_nv_action_context.h"
#include "oxr_api_funcs.h"
#include "oxr_api_verify.h"
#include "oxr_binding.h"
#include "oxr_chain.h"
#include "oxr_input.h"
#include "oxr_logger.h"
#include "oxr_objects.h"
#include "oxr_session_action_context.h"


#ifdef OXR_HAVE_NVX1_action_context

XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrCreateInstanceActionContextNV(XrInstance instance,
                                    const XrInstanceActionContextCreateInfoNV *createInfo,
                                    XrInstanceActionContextNV *instanceActionContext)
{
	struct oxr_instance *inst = NULL;
	struct oxr_logger log;
	struct oxr_instance_action_context_nv *ctx = NULL;
	XrResult ret;

	OXR_VERIFY_INSTANCE_AND_INIT_LOG(&log, instance, inst, "xrCreateInstanceActionContextNV");
	OXR_VERIFY_EXTENSION(&log, inst, NVX1_action_context);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, createInfo, XR_TYPE_INSTANCE_ACTION_CONTEXT_CREATE_INFO_NV);
	OXR_VERIFY_ARG_NOT_NULL(&log, instanceActionContext);

	ret = oxr_instance_action_context_nv_create(&log, inst, createInfo, &ctx);
	if (ret != XR_SUCCESS) {
		return ret;
	}
	*instanceActionContext = oxr_instance_action_context_nv_to_openxr(ctx);
	return XR_SUCCESS;
}

XRAPI_ATTR void XRAPI_CALL
oxr_xrDestroyInstanceActionContextNV(XrInstanceActionContextNV instanceActionContext)
{
	struct oxr_instance_action_context_nv *ctx = NULL;
	struct oxr_logger log;
	oxr_log_init(&log, "xrDestroyInstanceActionContextNV");
	if (instanceActionContext == XR_NULL_HANDLE) {
		return;
	}
	ctx = (struct oxr_instance_action_context_nv *)(uintptr_t)instanceActionContext;
	if (ctx->handle.debug != OXR_XR_DEBUG_INSTANCE_ACTION_CONTEXT_NV) {
		return;
	}
	if (ctx->handle.state != OXR_HANDLE_STATE_LIVE) {
		return;
	}
	oxr_handle_destroy(&log, &ctx->handle);
}

XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrCreateSessionActionContextNV(XrSession session,
                                   const XrSessionActionContextCreateInfoNV *createInfo,
                                   XrSessionActionContextNV *sessionActionContext)
{
	struct oxr_session *sess = NULL;
	struct oxr_logger log;
	struct oxr_session_action_context_nv *ctx = NULL;
	XrResult ret;

	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess, "xrCreateSessionActionContextNV");
	OXR_VERIFY_EXTENSION(&log, sess->sys->inst, NVX1_action_context);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, createInfo, XR_TYPE_SESSION_ACTION_CONTEXT_CREATE_INFO_NV);
	OXR_VERIFY_ARG_NOT_NULL(&log, sessionActionContext);

	struct oxr_instance_action_context_nv *inst_ctx_nv = NULL;
	OXR_VERIFY_INSTANCE_ACTION_CONTEXT_NV_AND_INIT_LOG(&log, createInfo->instanceActionContext, inst_ctx_nv,
	                                                   "xrCreateSessionActionContextNV");
	if (inst_ctx_nv->inst != sess->sys->inst) {
		return oxr_error(&log, XR_ERROR_HANDLE_INVALID,
		                 "instanceActionContext belongs to a different instance");
	}
	if (inst_ctx_nv->immutable) {
		return oxr_error(
		    &log, XR_ERROR_VALIDATION_FAILURE,
		    "instanceActionContext is already immutable (a session action context was created from it)");
	}

	ret = oxr_session_action_context_nv_create(&log, sess, createInfo, inst_ctx_nv, &ctx);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	*sessionActionContext = oxr_session_action_context_nv_to_openxr(ctx);

	return oxr_session_success_result(sess);
}

XRAPI_ATTR void XRAPI_CALL
oxr_xrDestroySessionActionContextNV(XrSessionActionContextNV sessionActionContext)
{
	struct oxr_session_action_context_nv *ctx = NULL;
	struct oxr_logger log;
	oxr_log_init(&log, "xrDestroySessionActionContextNV");
	if (sessionActionContext == XR_NULL_HANDLE) {
		return;
	}
	ctx = (struct oxr_session_action_context_nv *)(uintptr_t)sessionActionContext;
	if (ctx->handle.debug != OXR_XR_DEBUG_SESSION_ACTION_CONTEXT_NV) {
		return;
	}
	if (ctx->handle.state != OXR_HANDLE_STATE_LIVE) {
		return;
	}
	oxr_handle_destroy(&log, &ctx->handle);
}

XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrSyncActions2NV(XrSession session, const XrActionsSyncInfo2NV *syncInfo, XrActionsSyncState2NV *syncState)
{
	OXR_TRACE_MARKER();

	struct oxr_session *sess = NULL;
	struct oxr_logger log;
	struct oxr_session_action_context_nv *sess_ctx_nv = NULL;

	XrResult ret;

	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess, "xrSyncActions2NV");
	OXR_VERIFY_SESSION_NOT_LOST(&log, sess);
	OXR_VERIFY_EXTENSION(&log, sess->sys->inst, NVX1_action_context);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, syncInfo, XR_TYPE_ACTIONS_SYNC_INFO_2_NV);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, syncState, XR_TYPE_ACTIONS_SYNC_STATE_2_NV);


	struct oxr_session_action_context *sess_context = NULL;
	struct oxr_instance_action_context *inst_context = NULL;

	if (syncInfo->sessionActionContext == XR_NULL_HANDLE) {
		sess_context = &sess->action_context;
		inst_context = sess->sys->inst->action_context;
	} else {
		OXR_VERIFY_SESSION_ACTION_CONTEXT_NV_AND_INIT_LOG(&log, syncInfo->sessionActionContext, sess_ctx_nv,
		                                                  "xrSyncActions2NV");
		if (sess_ctx_nv->sess != sess) {
			return oxr_error(&log, XR_ERROR_HANDLE_INVALID,
			                 "sessionActionContext belongs to a different session");
		}
		sess_context = &sess_ctx_nv->context;
		inst_context = sess_ctx_nv->inst_context;
	}

	const XrActiveActionSetPrioritiesEXT *active_priorities = NULL;
#ifdef OXR_HAVE_EXT_active_action_set_priority
	active_priorities = OXR_GET_OUTPUT_FROM_CHAIN(syncInfo->next, XR_TYPE_ACTIVE_ACTION_SET_PRIORITIES_EXT,
	                                              const XrActiveActionSetPrioritiesEXT);
	if (active_priorities) {
		OXR_VERIFY_EXTENSION(&log, sess->sys->inst, EXT_active_action_set_priority);
	}
#endif

	ret = oxr_verify_active_action_sets_sync( //
	    &log,                                 //
	    sess->sys->inst,                      //
	    syncInfo->countActiveActionSets,      //
	    syncInfo->activeActionSets,           //
	    inst_context,                         //
	    "syncInfo->activeActionSets");        //
	if (ret != XR_SUCCESS) {
		return ret;
	}

	bool interaction_profile_changed = false;
	ret = oxr_action_sync_data_with_context( //
	    &log,                                //
	    sess,                                //
	    sess_context,                        //
	    syncInfo->countActiveActionSets,     //
	    syncInfo->activeActionSets,          //
	    active_priorities,                   //
	    &interaction_profile_changed);       //
	if (ret != XR_SUCCESS) {
		return ret;
	}

	syncState->interactionProfileChanged = interaction_profile_changed;

	return oxr_session_success_result(sess);
}

XRAPI_ATTR XrResult XRAPI_CALL
oxr_xrGetCurrentInteractionProfile2NV(XrSession session,
                                      const XrInteractionProfileGetInfo2NV *getInfo,
                                      XrInteractionProfileState *state)
{
	OXR_TRACE_MARKER();

	struct oxr_session *sess = NULL;
	struct oxr_logger log;
	struct oxr_session_action_context_nv *sess_ctx_nv = NULL;
	XrResult ret;

	OXR_VERIFY_SESSION_AND_INIT_LOG(&log, session, sess, "xrGetCurrentInteractionProfile2NV");
	OXR_VERIFY_SESSION_NOT_LOST(&log, sess);
	OXR_VERIFY_EXTENSION(&log, sess->sys->inst, NVX1_action_context);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, getInfo, XR_TYPE_INTERACTION_PROFILE_GET_INFO_2_NV);
	OXR_VERIFY_ARG_TYPE_AND_NOT_NULL(&log, state, XR_TYPE_INTERACTION_PROFILE_STATE);

	struct oxr_instance *inst = sess->sys->inst;

	const struct oxr_session_action_context *sess_context = NULL;

	if (getInfo->sessionActionContext == XR_NULL_HANDLE) {
		OXR_SESSION_CHECK_ATTACHED_AND_RET(sess, &log);
		sess_context = &sess->action_context;
	} else {
		OXR_VERIFY_SESSION_ACTION_CONTEXT_NV_AND_INIT_LOG(&log, getInfo->sessionActionContext, sess_ctx_nv,
		                                                  "xrGetCurrentInteractionProfile2NV");
		if (sess_ctx_nv->sess != sess) {
			return oxr_error(&log, XR_ERROR_HANDLE_INVALID,
			                 "sessionActionContext belongs to a different session");
		}
		sess_context = &sess_ctx_nv->context;

		if (!oxr_session_action_context_has_attached_act_sets(sess_context)) {
			return oxr_error(&log, XR_ERROR_ACTIONSET_NOT_ATTACHED,
			                 "(getInfo->sessionActionContext) not attached action sets");
		}
	}

	if (getInfo->topLevelUserPath == XR_NULL_PATH) {
		return oxr_error(&log, XR_ERROR_PATH_INVALID,
		                 "(topLevelUserPath == XR_NULL_PATH) The null path is not a valid argument");
	}

	if (!oxr_path_is_valid(&log, inst, getInfo->topLevelUserPath)) {
		return oxr_error(&log, XR_ERROR_PATH_INVALID, "(topLevelUserPath == %" PRId64 ") Is not a valid path",
		                 getInfo->topLevelUserPath);
	}

	bool fail = true;
#define COMPUTE_FAIL(X)                                                                                                \
	if (getInfo->topLevelUserPath == inst->path_cache.X) {                                                         \
		fail = false;                                                                                          \
	}
	OXR_FOR_EACH_SUBACTION_PATH(COMPUTE_FAIL)
#undef COMPUTE_FAIL
	if (fail) {
		const char *str = NULL;
		size_t len = 0;
		oxr_path_get_string(&log, inst, getInfo->topLevelUserPath, &str, &len);
		return oxr_error(&log, XR_ERROR_PATH_UNSUPPORTED,
		                 "(topLevelUserPath == %s) Is not a valid top level user path", str);
	}

	ret = oxr_action_get_current_interaction_profile( //
	    &log,                                         //
	    &inst->path_cache,                            //
	    sess_context,                                 //
	    getInfo->topLevelUserPath,                    //
	    state);                                       //
	if (ret != XR_SUCCESS) {
		return ret;
	}
	return oxr_session_success_result(sess);
}

#endif // OXR_HAVE_NVX1_action_context
