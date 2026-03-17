// Copyright 2019-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Helper for implementing pacing using C++
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include "u_pacing.h"
#include "u_interface_helper.hpp"
#include "xrt/xrt_deleters.hpp"

#include <memory>

namespace xrt::auxiliary::util {

/**
 * Type alias for a unique_ptr holding an implementation of @ref u_pacing_compositor
 */
using unique_pacing_compositor =
    std::unique_ptr<u_pacing_compositor, xrt::deleters::ptr_ptr_deleter<u_pacing_compositor, u_pc_destroy>>;

/**
 * @brief Helper for implementing @ref u_pacing_compositor with C++
 *
 * @tparam T Your implementation object type.
 *
 * Implementation object must have the following methods, with the same arguments as the C types except
 * without the leading `upc` self-argument.
 * - predict - corresponds to #u_pc_predict
 * - markPoint - corresponds to #u_pc_mark_point
 * - info - corresponds to #u_pc_info
 * - updatePresentOffset - corresponds to #u_pc_update_present_offset
 *
 * Implementation type must also have `using interface_type = u_pacing_compositor;` as a public type alias.
 *
 * See the cxx_wrappers tests for a sample uage of this.
 */
template <typename T>
unique_impl_wrapper<T>
makeUniquePacingCompositorImpl(std::unique_ptr<T> &&implementation)
{
	static_assert(std::is_same<typename T::interface_type, u_pacing_compositor>::value,
	              "T must have interface_type alias for u_pacing_compositor.");
	using wrapper = InterfaceImplWrapper<T>;
	auto ret = makeUniqueImplWrapper(std::move(implementation));
	auto &base = ret->base;
	base.destroy = wrapper::destroy;

	base.predict = [](struct u_pacing_compositor *upc, int64_t now_ns, int64_t *out_frame_id,
	                  int64_t *out_wake_up_time_ns, int64_t *out_desired_present_time_ns,
	                  int64_t *out_present_slop_ns, int64_t *out_predicted_display_time_ns,
	                  int64_t *out_predicted_display_period_ns, int64_t *out_min_display_period_ns) {
		wrapper::getObj(upc).predict(now_ns, out_frame_id, out_wake_up_time_ns, out_desired_present_time_ns,
		                             out_present_slop_ns, out_predicted_display_time_ns,
		                             out_predicted_display_period_ns, out_min_display_period_ns);
	};
	base.mark_point = [](struct u_pacing_compositor *upc, enum u_timing_point point, int64_t frame_id,
	                     int64_t when_ns) { wrapper::getObj(upc).markPoint(point, frame_id, when_ns); };

	base.info = [](struct u_pacing_compositor *upc, int64_t frame_id, int64_t desired_present_time_ns,
	               int64_t actual_present_time_ns, int64_t earliest_present_time_ns, int64_t present_margin_ns,
	               int64_t when_ns) {
		wrapper::getObj(upc).info(frame_id, desired_present_time_ns, actual_present_time_ns,
		                          earliest_present_time_ns, present_margin_ns, when_ns);
	};

	base.update_present_offset = [](struct u_pacing_compositor *upc, int64_t frame_id, int64_t present_offset_ns) {
		wrapper::getObj(upc).updatePresentOffset(frame_id, present_offset_ns);
	};

	return ret;
}

} // namespace xrt::auxiliary::util
