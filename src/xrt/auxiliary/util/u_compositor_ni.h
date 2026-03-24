// Copyright 2019-2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Not implemented function helpers for compositors.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_compositor.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Not implemented function helpers.
 *
 */

/*!
 * Not implemented function for @ref xrt_compositor::get_reference_bounds_rect.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_compositor_ni_get_reference_bounds_rect(struct xrt_compositor *xc,
                                          enum xrt_reference_space_type reference_space_type,
                                          struct xrt_vec2 *bounds);

/*!
 * Not implemented function for @ref xrt_compositor::get_view_resolution.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_compositor_ni_get_view_resolution(struct xrt_compositor *xc,
                                    enum xrt_view_type view_type,
                                    uint32_t view,
                                    float *out_scale,
                                    struct xrt_size *out_resolution);


#ifdef __cplusplus
}
#endif
