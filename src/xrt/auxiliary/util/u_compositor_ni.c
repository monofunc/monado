// Copyright 2019-2025, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Not implemented function helpers for compositors.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#include "util/u_compositor_ni.h"
#include "util/u_logging.h"


/*
 *
 * Not implemented function helpers.
 *
 */

#define E(FN) U_LOG_E("Function " #FN " is not implemented")

xrt_result_t
u_compositor_ni_get_reference_bounds_rect(struct xrt_compositor *xc,
                                          enum xrt_reference_space_type reference_space_type,
                                          struct xrt_vec2 *bounds)
{
	E(get_reference_bounds_rect);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_compositor_ni_get_view_resolution(struct xrt_compositor *xc,
                                    enum xrt_view_type view_type,
                                    uint32_t view,
                                    float *out_scale,
                                    struct xrt_size *out_resolution)
{
	E(get_view_resolution);
	return XRT_ERROR_NOT_IMPLEMENTED;
}
