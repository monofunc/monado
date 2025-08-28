// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Very small misc utils.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "util/u_misc.h"


xrt_result_t
u_system_ni_get_viewport_scale(struct xrt_system *xsys, double *out_scale)
{
	*out_scale = 1.0;
	return XRT_SUCCESS;
}
