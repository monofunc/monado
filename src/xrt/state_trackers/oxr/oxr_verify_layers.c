// Copyright 2018-2022, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  File for verifying app input into api functions.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_main
 * @ingroup oxr_api
 */

#include "xrt/xrt_compiler.h"

#include "math/m_mathinclude.h"
#include "math/m_api.h"

#include "util/u_debug.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_api_verify.h"
#include "oxr_chain.h"

#include "actions/oxr_subaction.h"
#include "actions/oxr_input.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>


/*
 *
 * Layer functions.
 *
 */

static XrResult
is_rect_neg(const XrRect2Di *imageRect)
{
	if (imageRect->offset.x < 0 || imageRect->offset.y < 0) {
		return true;
	}

	return false;
}

static XrResult
is_rect_out_of_bounds(const XrRect2Di *imageRect, struct oxr_swapchain *sc)
{
	uint32_t total_width = imageRect->offset.x + imageRect->extent.width;
	if (total_width > sc->width) {
		return true;
	}
	uint32_t total_height = imageRect->offset.y + imageRect->extent.height;
	if (total_height > sc->height) {
		return true;
	}

	return false;
}

static XrResult
verify_blend_factors(struct oxr_logger *log,
                     struct oxr_session *sess,
                     uint32_t layer_index,
                     const XrCompositionLayerBaseHeader *layer)
{
#ifdef OXR_HAVE_FB_composition_layer_alpha_blend
	if (!sess->sys->inst->extensions.FB_composition_layer_alpha_blend) {
		return XR_SUCCESS;
	}

	const XrCompositionLayerAlphaBlendFB *alphaBlend = OXR_GET_INPUT_FROM_CHAIN(
	    layer, (XrStructureType)XR_TYPE_COMPOSITION_LAYER_ALPHA_BLEND_FB, XrCompositionLayerAlphaBlendFB);

	if (alphaBlend != NULL) {
		if (!u_verify_blend_factor_valid(alphaBlend->srcFactorColor)) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->pNext->srcFactorColor == 0x%08x) unknown blend factor",
			    layer_index, alphaBlend->srcFactorColor);
		}
		if (!u_verify_blend_factor_valid(alphaBlend->dstFactorColor)) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->dstFactorColor == 0x%08x) unknown blend factor",
			                 layer_index, alphaBlend->dstFactorColor);
		}
		if (!u_verify_blend_factor_valid(alphaBlend->srcFactorAlpha)) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->srcFactorAlpha == 0x%08x) unknown blend factor",
			                 layer_index, alphaBlend->srcFactorAlpha);
		}
		if (!u_verify_blend_factor_valid(alphaBlend->dstFactorAlpha)) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->dstFactorAlpha == 0x%08x) unknown blend factor",
			                 layer_index, alphaBlend->dstFactorAlpha);
		}
	}
#else
	// Extension isn't enabled, always pass.
	return XR_SUCCESS;
#endif
}

static XrResult
verify_space(struct oxr_logger *log, uint32_t layer_index, XrSpace space)
{
	if (space == XR_NULL_HANDLE) {
		return oxr_error(
		    log, XR_ERROR_VALIDATION_FAILURE,
		    "(frameEndInfo->layers[%u]->space == XR_NULL_HANDLE) XrSpace must not be XR_NULL_HANDLE",
		    layer_index);
	}

	return XR_SUCCESS;
}

XrResult
oxr_verify_quad_layer(struct oxr_session *sess,
                      struct oxr_logger *log,
                      uint32_t layer_index,
                      XrCompositionLayerQuad *quad,
                      bool verify_swapchian)
{
	XrResult ret = verify_space(log, layer_index, quad->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)quad);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (!math_quat_validate_within_1_percent((struct xrt_quat *)&quad->pose.orientation)) {
		XrQuaternionf *q = &quad->pose.orientation;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.orientation == {%f %f %f %f}) is not a valid quat",
		                 layer_index, q->x, q->y, q->z, q->w);
	}

	if (!math_vec3_validate((struct xrt_vec3 *)&quad->pose.position)) {
		XrVector3f *p = &quad->pose.position;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.position == {%f %f %f}) is not valid", layer_index,
		                 p->x, p->y, p->z);
	}

	if (verify_swapchian) {
		struct oxr_swapchain *sc = XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, quad->subImage.swapchain);

		if (sc == NULL) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain is NULL!",
			                 layer_index);
		}

		if (sc->array_layer_count <= quad->subImage.imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.imageArrayIndex == %u) Invalid swapchain array "
			    "index for quad layer (%u).",
			    layer_index, quad->subImage.imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 1, got %u)",
			                 layer_index, sc->face_count);
		}

		if (!sc->released.yes) {
			return oxr_error(
			    log, XR_ERROR_LAYER_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain has not been released!",
			    layer_index);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) internal image index out of bounds",
			    layer_index);
		}


		if (is_rect_out_of_bounds(&quad->subImage.imageRect, sc)) {
			return oxr_error(
			    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.imageRect == {{%i, %i}, {%u, %u}}) imageRect out "
			    "of image bounds (%u, %u)",
			    layer_index, quad->subImage.imageRect.offset.x, quad->subImage.imageRect.offset.y,
			    quad->subImage.imageRect.extent.width, quad->subImage.imageRect.extent.height, sc->width,
			    sc->height);
		}
	}

	if (is_rect_neg(&quad->subImage.imageRect)) {
		return oxr_error(log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
		                 "(frameEndInfo->layers[%u]->subImage.imageRect.offset == {%i, %i}) has "
		                 "negative component(s)",
		                 layer_index, quad->subImage.imageRect.offset.x, quad->subImage.imageRect.offset.y);
	}

	return XR_SUCCESS;
}

#ifdef OXR_HAVE_KHR_composition_layer_depth
static XrResult
oxr_verify_depth_layer(struct oxr_logger *log,
                       uint32_t layer_index,
                       uint32_t i,
                       const XrCompositionLayerDepthInfoKHR *depth,
                       bool verify_swapchain)
{
	if (verify_swapchain) {
		if (depth->subImage.swapchain == XR_NULL_HANDLE) {
			return oxr_error(
			    log, XR_ERROR_HANDLE_INVALID,
			    "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
			    "swapchain) is XR_NULL_HANDLE",
			    layer_index, i);
		}

		struct oxr_swapchain *sc =
		    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, depth->subImage.swapchain);

		if (!sc->released.yes) {
			return oxr_error(
			    log, XR_ERROR_LAYER_INVALID,
			    "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
			    "swapchain) swapchain has not been released",
			    layer_index, i);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
			    "swapchain) internal image index out of bounds",
			    layer_index, i);
		}

		if (sc->array_layer_count <= depth->subImage.imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
			    "imageArrayIndex == %u) Invalid swapchain array index for projection layer (%u).",
			    layer_index, i, depth->subImage.imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 1, got %u)",
			                 layer_index, sc->face_count);
		}

		if (is_rect_out_of_bounds(&depth->subImage.imageRect, sc)) {
			return oxr_error(
			    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			    "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
			    "imageRect == {{%i, %i}, {%u, %u}}) imageRect out of image bounds (%u, %u)",
			    layer_index, i, depth->subImage.imageRect.offset.x, depth->subImage.imageRect.offset.y,
			    depth->subImage.imageRect.extent.width, depth->subImage.imageRect.extent.height, sc->width,
			    sc->height);
		}
	}

	if (is_rect_neg(&depth->subImage.imageRect)) {
		return oxr_error(log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
		                 "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.subImage."
		                 "imageRect.offset == {%i, %i}) has negative component(s)",
		                 layer_index, i, depth->subImage.imageRect.offset.x,
		                 depth->subImage.imageRect.offset.y);
	}

	if (depth->minDepth < 0.0f || depth->minDepth > 1.0f) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.minDepth) "
		                 "%f must be in [0.0,1.0]",
		                 layer_index, i, depth->minDepth);
	}

	if (depth->maxDepth < 0.0f || depth->maxDepth > 1.0f) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.maxDepth) "
		                 "%f must be in [0.0,1.0]",
		                 layer_index, i, depth->maxDepth);
	}

	if (depth->minDepth > depth->maxDepth) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.minDepth) "
		                 "%f must be <= maxDepth %f ",
		                 layer_index, i, depth->minDepth, depth->maxDepth);
	}

	if (depth->nearZ == depth->farZ) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->views[%i]->next<XrCompositionLayerDepthInfoKHR>.nearZ) %f "
		                 "must be != farZ %f ",
		                 layer_index, i, depth->nearZ, depth->farZ);
	}


	return XR_SUCCESS;
}
#endif // OXR_HAVE_KHR_composition_layer_depth

XrResult
oxr_verify_projection_layer(struct oxr_session *sess,
                            struct oxr_logger *log,
                            uint32_t layer_index,
                            XrCompositionLayerProjection *proj,
                            bool verify_swapchain)
{
	XrResult ret = verify_space(log, layer_index, proj->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)proj);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	switch (sess->current_view_config_type) {
	case XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO:
		if (proj->viewCount != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->viewCount == %u) must be 1 for "
			                 "XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO",
			                 layer_index, proj->viewCount);
		}
		break;
	case XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO:
		if (proj->viewCount != 2) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->viewCount == %u) must be 2 for "
			                 "XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO",
			                 layer_index, proj->viewCount);
		}
		break;
	// This also includes XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO_WITH_FOVEATED_INSET as both values are the same
	case XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO:
		if (proj->viewCount != 4) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->viewCount == %u) must be 4 for "
			                 "XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO",
			                 layer_index, proj->viewCount);
		}
		break;
	case XR_VIEW_CONFIGURATION_TYPE_SECONDARY_MONO_FIRST_PERSON_OBSERVER_MSFT:
		if (proj->viewCount != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->viewCount == %u) must be 1 for "
			                 "XR_VIEW_CONFIGURATION_TYPE_SECONDARY_MONO_FIRST_PERSON_OBSERVER_MSFT",
			                 layer_index, proj->viewCount);
		}
		break;
	default:
		assert(false && "view type validation unimplemented");
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "view type %d not supported",
		                 sess->current_view_config_type);
		break;
	}

#ifdef OXR_HAVE_KHR_composition_layer_depth
	// number of depth layers must be 0 or proj->viewCount
	uint32_t depth_layer_count = 0;
#endif

	// Check for valid swapchain states.
	for (uint32_t i = 0; i < proj->viewCount; i++) {
		const XrCompositionLayerProjectionView *view = &proj->views[i];

		//! @todo More validation?
		if (!math_quat_validate_within_1_percent((struct xrt_quat *)&view->pose.orientation)) {
			const XrQuaternionf *q = &view->pose.orientation;
			return oxr_error(log, XR_ERROR_POSE_INVALID,
			                 "(frameEndInfo->layers[%u]->views[%i]->pose."
			                 "orientation == {%f %f %f %f}) is not a valid quat",
			                 layer_index, i, q->x, q->y, q->z, q->w);
		}

		if (!math_vec3_validate((struct xrt_vec3 *)&view->pose.position)) {
			const XrVector3f *p = &view->pose.position;
			return oxr_error(log, XR_ERROR_POSE_INVALID,
			                 "(frameEndInfo->layers[%u]->views[%i]->pose."
			                 "position == {%f %f %f}) is not valid",
			                 layer_index, i, p->x, p->y, p->z);
		}

		if (verify_swapchain) {
			if (view->subImage.swapchain == XR_NULL_HANDLE) {
				return oxr_error(log, XR_ERROR_HANDLE_INVALID,
				                 "(frameEndInfo->layers[%u]->views[%i]->subImage."
				                 "swapchain is XR_NULL_HANDLE",
				                 layer_index, i);
			}

			struct oxr_swapchain *sc =
			    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, view->subImage.swapchain);

			if (!sc->released.yes) {
				return oxr_error(log, XR_ERROR_LAYER_INVALID,
				                 "(frameEndInfo->layers[%u]->views[%i].subImage."
				                 "swapchain) swapchain has not been released",
				                 layer_index, i);
			}

			if (sc->released.index >= (int)sc->swapchain->image_count) {
				return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
				                 "(frameEndInfo->layers[%u]->views[%i].subImage."
				                 "swapchain) internal image index out of bounds",
				                 layer_index, i);
			}

			if (sc->array_layer_count <= view->subImage.imageArrayIndex) {
				return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
				                 "(frameEndInfo->layers[%u]->views[%i]->subImage."
				                 "imageArrayIndex == %u) Invalid swapchain array "
				                 "index for projection layer (%u).",
				                 layer_index, i, view->subImage.imageArrayIndex, sc->array_layer_count);
			}

			if (sc->face_count != 1) {
				return oxr_error(
				    log, XR_ERROR_VALIDATION_FAILURE,
				    "(frameEndInfo->layers[%u]->views[%i]->subImage.swapchain) Invalid swapchain "
				    "face count (expected 1, got %u)",
				    layer_index, i, sc->face_count);
			}

			if (is_rect_out_of_bounds(&view->subImage.imageRect, sc)) {
				return oxr_error(log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
				                 "(frameEndInfo->layers[%u]->views[%i]->subImage."
				                 "imageRect == {{%i, %i}, {%u, %u}}) imageRect out "
				                 "of image bounds (%u, %u)",
				                 layer_index, i, view->subImage.imageRect.offset.x,
				                 view->subImage.imageRect.offset.y,
				                 view->subImage.imageRect.extent.width,
				                 view->subImage.imageRect.extent.height, sc->width, sc->height);
			}
		}

		if (is_rect_neg(&view->subImage.imageRect)) {
			return oxr_error(log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			                 "(frameEndInfo->layers[%u]->views[%i]-"
			                 ">subImage.imageRect.offset == {%i, "
			                 "%i}) has negative component(s)",
			                 layer_index, i, view->subImage.imageRect.offset.x,
			                 view->subImage.imageRect.offset.y);
		}

#ifdef OXR_HAVE_KHR_composition_layer_depth
		const XrCompositionLayerDepthInfoKHR *depth_info = OXR_GET_INPUT_FROM_CHAIN(
		    view, XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR, XrCompositionLayerDepthInfoKHR);

		if (depth_info) {
			ret = oxr_verify_depth_layer(log, layer_index, i, depth_info, verify_swapchain);
			if (ret != XR_SUCCESS) {
				return ret;
			}
			depth_layer_count++;
		}
#endif // OXR_HAVE_KHR_composition_layer_depth
	}

#ifdef OXR_HAVE_KHR_composition_layer_depth
	if (depth_layer_count > 0 && depth_layer_count != proj->viewCount) {
		return oxr_error(
		    log, XR_ERROR_VALIDATION_FAILURE,
		    "(frameEndInfo->layers[%u] projection layer must have %u depth layers or none, but has: %u)",
		    layer_index, proj->viewCount, depth_layer_count);
	}
#endif // OXR_HAVE_KHR_composition_layer_depth

	return XR_SUCCESS;
}

XrResult
oxr_verify_cube_layer(struct oxr_session *sess,
                      struct oxr_logger *log,
                      uint32_t layer_index,
                      const XrCompositionLayerCubeKHR *cube,
                      bool verify_swapchain)
{
#ifndef OXR_HAVE_KHR_composition_layer_cube
	return oxr_error(log, XR_ERROR_LAYER_INVALID,
	                 "(frameEndInfo->layers[%u]->type) layer type "
	                 "XrCompositionLayerCubeKHR not supported",
	                 layer_index);
#else
	XrResult ret = verify_space(log, layer_index, cube->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)cube);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (!math_quat_validate_within_1_percent((struct xrt_quat *)&cube->orientation)) {
		const XrQuaternionf *q = &cube->orientation;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.orientation == {%f %f %f %f}) is not a valid quat",
		                 layer_index, q->x, q->y, q->z, q->w);
	}

	if (verify_swapchain) {
		struct oxr_swapchain *sc = XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, cube->swapchain);

		if (sc == NULL) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain is NULL!",
			                 layer_index);
		}

		if (sc->array_layer_count <= cube->imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->imageArrayIndex == %u) Invalid swapchain array index for "
			    "cube layer (%u).",
			    layer_index, cube->imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 6) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 6, got %u)",
			                 layer_index, sc->face_count);
		}

		if (!sc->released.yes) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->swapchain) swapchain has not been released!",
			                 layer_index);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) internal image index out of bounds",
			    layer_index);
		}
	}

	return XR_SUCCESS;
#endif // OXR_HAVE_KHR_composition_layer_cube
}

XrResult
oxr_verify_cylinder_layer(struct oxr_session *sess,
                          struct oxr_logger *log,
                          uint32_t layer_index,
                          const XrCompositionLayerCylinderKHR *cylinder,
                          bool verify_swapchain)
{
#ifndef OXR_HAVE_KHR_composition_layer_cylinder
	return oxr_error(log, XR_ERROR_LAYER_INVALID,
	                 "(frameEndInfo->layers[%u]->type) layer type "
	                 "XrCompositionLayerCylinderKHR not supported",
	                 layer_index);
#else
	XrResult ret = verify_space(log, layer_index, cylinder->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)cylinder);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (!math_quat_validate_within_1_percent((struct xrt_quat *)&cylinder->pose.orientation)) {
		const XrQuaternionf *q = &cylinder->pose.orientation;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.orientation == {%f %f %f %f}) is not a valid quat",
		                 layer_index, q->x, q->y, q->z, q->w);
	}

	if (!math_vec3_validate((struct xrt_vec3 *)&cylinder->pose.position)) {
		const XrVector3f *p = &cylinder->pose.position;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.position == {%f %f %f}) is not valid", layer_index,
		                 p->x, p->y, p->z);
	}

	if (verify_swapchain) {
		struct oxr_swapchain *sc =
		    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, cylinder->subImage.swapchain);

		if (sc == NULL) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain is NULL!",
			                 layer_index);
		}


		if (sc->array_layer_count <= cylinder->subImage.imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.imageArrayIndex == %u) Invalid swapchain array "
			    "index for cylinder layer (%u).",
			    layer_index, cylinder->subImage.imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 1, got %u)",
			                 layer_index, sc->face_count);
		}

		if (!sc->released.yes) {
			return oxr_error(
			    log, XR_ERROR_LAYER_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain has not been released!",
			    layer_index);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) internal image index out of bounds",
			    layer_index);
		}

		if (is_rect_out_of_bounds(&cylinder->subImage.imageRect, sc)) {
			return oxr_error(
			    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.imageRect == {{%i, %i}, {%u, %u}}) imageRect out "
			    "of image bounds (%u, %u)",
			    layer_index, cylinder->subImage.imageRect.offset.x, cylinder->subImage.imageRect.offset.y,
			    cylinder->subImage.imageRect.extent.width, cylinder->subImage.imageRect.extent.height,
			    sc->width, sc->height);
		}
	}

	if (is_rect_neg(&cylinder->subImage.imageRect)) {
		return oxr_error(
		    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
		    "(frameEndInfo->layers[%u]->subImage.imageRect.offset == {%i, %i}) has negative component(s)",
		    layer_index, cylinder->subImage.imageRect.offset.x, cylinder->subImage.imageRect.offset.y);
	}

	if (cylinder->radius < 0.f) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(frameEndInfo->layers[%u]->radius == %f) radius cannot be negative", layer_index,
		                 cylinder->radius);
	}

	if (cylinder->centralAngle < 0.f || cylinder->centralAngle > (M_PI * 2)) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(frameEndInfo->layers[%u]->centralAngle == %f) centralAngle out of bounds",
		                 layer_index, cylinder->centralAngle);
	}

	if (cylinder->aspectRatio <= 0.f) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(frameEndInfo->layers[%u]->aspectRatio == %f) aspectRatio out of bounds", layer_index,
		                 cylinder->aspectRatio);
	}

	return XR_SUCCESS;
#endif // OXR_HAVE_KHR_composition_layer_cylinder
}

XrResult
oxr_verify_equirect1_layer(struct oxr_session *sess,
                           struct oxr_logger *log,
                           uint32_t layer_index,
                           const XrCompositionLayerEquirectKHR *equirect,
                           bool verify_swapchain)
{
#ifndef OXR_HAVE_KHR_composition_layer_equirect
	return oxr_error(log, XR_ERROR_LAYER_INVALID,
	                 "(frameEndInfo->layers[%u]->type) layer type "
	                 "XrCompositionLayerEquirectKHR not supported",
	                 layer_index);
#else
	if (verify_swapchain) {
		struct oxr_swapchain *sc =
		    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, equirect->subImage.swapchain);

		if (sc == NULL) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain is NULL!",
			                 layer_index);
		}

		if (sc->array_layer_count <= equirect->subImage.imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.imageArrayIndex == %u) Invalid swapchain array "
			    "index for equirect layer (%u).",
			    layer_index, equirect->subImage.imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 1, got %u)",
			                 layer_index, sc->face_count);
		}

		if (!sc->released.yes) {
			return oxr_error(
			    log, XR_ERROR_LAYER_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain has not been released!",
			    layer_index);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) internal image index out of bounds",
			    layer_index);
		}

		if (is_rect_out_of_bounds(&equirect->subImage.imageRect, sc)) {
			return oxr_error(
			    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.imageRect == {{%i, %i}, {%u, %u}}) imageRect out "
			    "of image bounds (%u, %u)",
			    layer_index, equirect->subImage.imageRect.offset.x, equirect->subImage.imageRect.offset.y,
			    equirect->subImage.imageRect.extent.width, equirect->subImage.imageRect.extent.height,
			    sc->width, sc->height);
		}
	}

	XrResult ret = verify_space(log, layer_index, equirect->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)equirect);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (!math_quat_validate_within_1_percent((struct xrt_quat *)&equirect->pose.orientation)) {
		const XrQuaternionf *q = &equirect->pose.orientation;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.orientation == {%f %f %f %f}) is not a valid quat",
		                 layer_index, q->x, q->y, q->z, q->w);
	}

	if (!math_vec3_validate((struct xrt_vec3 *)&equirect->pose.position)) {
		const XrVector3f *p = &equirect->pose.position;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.position == {%f %f %f}) is not valid", layer_index,
		                 p->x, p->y, p->z);
	}

	if (is_rect_neg(&equirect->subImage.imageRect)) {
		return oxr_error(
		    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
		    "(frameEndInfo->layers[%u]->subImage.imageRect.offset == {%i, %i}) has negative component(s)",
		    layer_index, equirect->subImage.imageRect.offset.x, equirect->subImage.imageRect.offset.y);
	}

	if (equirect->radius < .0f) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(frameEndInfo->layers[%u]->radius == %f) radius out of bounds", layer_index,
		                 equirect->radius);
	}

	return XR_SUCCESS;
#endif // OXR_HAVE_KHR_composition_layer_equirect
}

XrResult
oxr_verify_equirect2_layer(struct oxr_session *sess,
                           struct oxr_logger *log,
                           uint32_t layer_index,
                           const XrCompositionLayerEquirect2KHR *equirect,
                           bool verify_swapchain)
{
#ifndef OXR_HAVE_KHR_composition_layer_equirect2
	return oxr_error(log, XR_ERROR_LAYER_INVALID,
	                 "(frameEndInfo->layers[%u]->type) layer type XrCompositionLayerEquirect2KHR not supported",
	                 layer_index);
#else
	if (verify_swapchain) {
		struct oxr_swapchain *sc =
		    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_swapchain *, equirect->subImage.swapchain);

		if (sc == NULL) {
			return oxr_error(log, XR_ERROR_LAYER_INVALID,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain is NULL!",
			                 layer_index);
		}

		if (sc->array_layer_count <= equirect->subImage.imageArrayIndex) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.imageArrayIndex == %u) Invalid swapchain array "
			    "index for equirect layer (%u).",
			    layer_index, equirect->subImage.imageArrayIndex, sc->array_layer_count);
		}

		if (sc->face_count != 1) {
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
			                 "(frameEndInfo->layers[%u]->subImage.swapchain) Invalid swapchain face count "
			                 "(expected 1, got %u)",
			                 layer_index, sc->face_count);
		}

		if (!sc->released.yes) {
			return oxr_error(
			    log, XR_ERROR_LAYER_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) swapchain has not been released!",
			    layer_index);
		}

		if (sc->released.index >= (int)sc->swapchain->image_count) {
			return oxr_error(
			    log, XR_ERROR_RUNTIME_FAILURE,
			    "(frameEndInfo->layers[%u]->subImage.swapchain) internal image index out of bounds",
			    layer_index);
		}

		if (is_rect_out_of_bounds(&equirect->subImage.imageRect, sc)) {
			return oxr_error(
			    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
			    "(frameEndInfo->layers[%u]->subImage.imageRect == {{%i, %i}, {%u, %u}}) imageRect out "
			    "of image bounds (%u, %u)",
			    layer_index, equirect->subImage.imageRect.offset.x, equirect->subImage.imageRect.offset.y,
			    equirect->subImage.imageRect.extent.width, equirect->subImage.imageRect.extent.height,
			    sc->width, sc->height);
		}
	}

	XrResult ret = verify_space(log, layer_index, equirect->space);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	ret = verify_blend_factors(log, sess, layer_index, (XrCompositionLayerBaseHeader *)equirect);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (!math_quat_validate_within_1_percent((struct xrt_quat *)&equirect->pose.orientation)) {
		const XrQuaternionf *q = &equirect->pose.orientation;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.orientation == {%f %f %f %f}) is not a valid quat",
		                 layer_index, q->x, q->y, q->z, q->w);
	}

	if (!math_vec3_validate((struct xrt_vec3 *)&equirect->pose.position)) {
		const XrVector3f *p = &equirect->pose.position;
		return oxr_error(log, XR_ERROR_POSE_INVALID,
		                 "(frameEndInfo->layers[%u]->pose.position == {%f %f %f}) is not valid", layer_index,
		                 p->x, p->y, p->z);
	}

	if (is_rect_neg(&equirect->subImage.imageRect)) {
		return oxr_error(
		    log, XR_ERROR_SWAPCHAIN_RECT_INVALID,
		    "(frameEndInfo->layers[%u]->subImage.imageRect.offset == {%i, %i}) has negative component(s)",
		    layer_index, equirect->subImage.imageRect.offset.x, equirect->subImage.imageRect.offset.y);
	}

	if (equirect->centralHorizontalAngle < .0f) {
		return oxr_error(
		    log, XR_ERROR_VALIDATION_FAILURE,
		    "(frameEndInfo->layers[%u]->centralHorizontalAngle == %f) centralHorizontalAngle out of bounds",
		    layer_index, equirect->centralHorizontalAngle);
	}

	/*
	 * Accept all angle ranges here, since we are dealing with π
	 * and we don't want floating point errors to prevent the client
	 * to display the full sphere.
	 */

	return XR_SUCCESS;
#endif // OXR_HAVE_KHR_composition_layer_equirect2
}

XrResult
oxr_verify_passthrough_layer(struct oxr_logger *log,
                             uint32_t layer_index,
                             const XrCompositionLayerPassthroughFB *passthrough)
{
#ifndef OXR_HAVE_FB_passthrough
	return oxr_error(log, XR_ERROR_LAYER_INVALID,
	                 "(frameEndInfo->layers[%u]->type) layer type XrCompositionLayerPassthroughFB not supported",
	                 layer_index);
#else
	if (passthrough->flags == 0 || (passthrough->flags & (XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT |
	                                                      XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
	                                                      XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT)) == 0) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->flags) layer flags is not a valid combination of "
		                 "XrCompositionLayerFlagBits values",
		                 layer_index);
	}

	if (passthrough->space) {
		XrResult ret = verify_space(log, layer_index, passthrough->space);
		if (ret != XR_SUCCESS) {
			return ret;
		}
	}

	struct oxr_passthrough_layer *pl =
	    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_passthrough_layer *, passthrough->layerHandle);
	if (pl == NULL) {
		return oxr_error(log, XR_ERROR_LAYER_INVALID,
		                 "(frameEndInfo->layers[%u]->layerHandle) layerHandle is NULL!", layer_index);
	}

	return XR_SUCCESS;
#endif
}
