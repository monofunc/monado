// Copyright 2021-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Miscellanous C++ wrapper tests.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 */

#include <iostream>
#include <xrt/xrt_device.hpp>
#include <util/u_pacing.hpp>
#include <os/os_time.h>

#include "catch_amalgamated.hpp"


struct silly_device
{
	xrt_device base{};
	bool *destroyed;


	silly_device(bool &destroyed_) : destroyed(&destroyed_)
	{
		base.destroy = [](xrt_device *xdev) { delete reinterpret_cast<silly_device *>(xdev); };
	}
	~silly_device()
	{
		*destroyed = true;
	}
};

TEST_CASE("unique_xrt_device")
{

	bool destroyed = false;
	{
		// make the device
		auto specific = std::make_unique<silly_device>(destroyed);
		CHECK_FALSE(destroyed);

		// use the generic unique_ptr
		xrt::unique_xrt_device generic(&(specific.release()->base));
		CHECK_FALSE(destroyed);
	}
	// make sure it went away
	CHECK(destroyed);
}

class Pacing
{
public:
	using interface_type = u_pacing_compositor;
	struct Status
	{
		bool predictCalled = false;
		bool markPointCalled = false;
		bool infoCalled = false;
		bool updatePresentOffsetCalled = false;
		bool destroyed = false;
	};
	explicit Pacing(Status &status) : mStatus(&status) {}
	~Pacing()
	{
		mStatus->destroyed = true;
	}


	void
	predict(int64_t now_ns,
	        int64_t *out_frame_id,
	        int64_t *out_wake_up_time_ns,
	        int64_t *out_desired_present_time_ns,
	        int64_t *out_present_slop_ns,
	        int64_t *out_predicted_display_time_ns,
	        int64_t *out_predicted_display_period_ns,
	        int64_t *out_min_display_period_ns)
	{
		mStatus->predictCalled = true;
	}


	void
	markPoint(u_timing_point point, int64_t frame_id, int64_t when_ns)
	{
		mStatus->markPointCalled = true;
	}


	void
	info(int64_t frame_id,
	     int64_t desired_present_time_ns,
	     int64_t actual_present_time_ns,
	     int64_t earliest_present_time_ns,
	     int64_t present_margin_ns,
	     int64_t when_ns)
	{
		mStatus->infoCalled = true;
	}

	void
	updatePresentOffset(int64_t frame_id, int64_t present_offset_ns)
	{
		mStatus->updatePresentOffsetCalled = true;
	}

private:
	Status *mStatus;
};

using namespace xrt::auxiliary::util;

static inline void
pacingShared(u_pacing_compositor *upc, Pacing::Status &status)
{
	CHECK_FALSE(status.predictCalled);
	CHECK_FALSE(status.markPointCalled);
	CHECK_FALSE(status.updatePresentOffsetCalled);
	CHECK_FALSE(status.infoCalled);
	CHECK_FALSE(status.destroyed);

	u_pc_update_present_offset(upc, 0, 0);
	CHECK_FALSE(status.predictCalled);
	CHECK_FALSE(status.markPointCalled);
	CHECK(status.updatePresentOffsetCalled);
	CHECK_FALSE(status.infoCalled);
	CHECK_FALSE(status.destroyed);

	int64_t out_frame_id;
	int64_t out_wake_up_time_ns, out_desired_present_time_ns, out_present_slop_ns, out_predicted_display_time_ns,
	    out_predicted_display_period_ns, out_min_display_period_ns;
	u_pc_predict(upc, os_monotonic_get_ns(), &out_frame_id, &out_wake_up_time_ns, &out_desired_present_time_ns,
	             &out_present_slop_ns, &out_predicted_display_time_ns, &out_predicted_display_period_ns,
	             &out_min_display_period_ns);
	CHECK(status.predictCalled);
	CHECK_FALSE(status.markPointCalled);
	CHECK_FALSE(status.infoCalled);
	CHECK_FALSE(status.destroyed);
}

TEST_CASE("pacing_impl")
{

	Pacing::Status status;
	{
		auto unique_wrapper = makeUniquePacingCompositorImpl(std::make_unique<Pacing>(status));
		SECTION("using unique_impl_wrapper")
		{
			pacingShared(unique_wrapper->getInterface(), status);
			// this is just a unique_ptr around the interface helper
			// so reset will free the object
			unique_wrapper.reset();
		}
		SECTION("using interface pointer")
		{
			pacingShared(unique_wrapper->getInterface(), status);
			auto *iface = releaseWrapperAsInterface(std::move(unique_wrapper));
			CHECK(unique_wrapper == nullptr);
			CHECK(iface);
			u_pc_destroy(&iface);
		}
		SECTION("using interface pointer in unique_pacing_compositor")
		{
			pacingShared(unique_wrapper->getInterface(), status);
			unique_pacing_compositor iface {
				releaseWrapperAsInterface(std::move(unique_wrapper))
			};
			CHECK(unique_wrapper == nullptr);
			CHECK(iface);
			// when iface goes out of scope, the whole thing should be destroyed
		}
	}
	// make sure it went away
	CHECK(status.destroyed);
}
