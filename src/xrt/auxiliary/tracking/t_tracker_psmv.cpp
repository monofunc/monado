// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include <stddef.h>
#include <assert.h>

#include "math/m_api.h"
#include "util/u_var.h"
#include "util/u_trace_marker.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"

#include "tracking/t_tracking.h"
#include "tracking/t_calibration_opencv.hpp"
#include "tracking/t_tracker_psmv_fusion.hpp"
#include "tracking/t_helper_debug_sink.hpp"
#include "tracking/t_conefitting.hpp"

#include <iostream>
#include <memory>
#include <numeric>
#include <type_traits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>


using namespace xrt::auxiliary::tracking;

//! Namespace for PS Move tracking implementation
namespace xrt::auxiliary::tracking::psmv {

static const xrt_vec3 sphere_lever_arm = {0, 0.9, 0};
using Contour = std::vector<cv::Point>;

constexpr float SphereRadius = 0.045f / 2.f;
constexpr size_t MinPoints = 6;

/*!
 * Single camera.
 *
 * @see TrackerPSMV
 */
struct View
{
public:
	// Image rectification / undistortion
	cv::Mat undistort_rectify_map_x;
	cv::Mat undistort_rectify_map_y;
	cv::Mat frame_undist_rectified;

	// Camera calibration
	cv::Matx33d intrinsics;
	cv::Mat distortion; // size may vary
	enum t_camera_distortion_model distortion_model;
	xrt_pose calibration_transform;

	// Cone-fitting helpers (cone-fitting tracking only)
	std::shared_ptr<NormalizedCoordsCache> norm_coords;
	std::shared_ptr<ConeFitter> cone_fitter = std::make_shared<ConeFitter>();

	// Keypoints (disparity-based tracking only)
	std::vector<cv::KeyPoint> keypoints;

	// Output position information
	cv::Vec3f position;
	xrt_vec3 transformed_pos;
	bool position_valid;
	bool transform_valid;

	void
	populate_from_calib(t_camera_calibration &calib, const RemapPair &rectification)
	{
		CameraCalibrationWrapper wrap(calib);
		intrinsics = wrap.intrinsics_mat;
		distortion = wrap.distortion_mat.clone();
		distortion_model = wrap.distortion_model;

		norm_coords.reset(
		    new NormalizedCoordsCache(wrap.image_size_pixels_cv, distortion_model, intrinsics, distortion));

		undistort_rectify_map_x = rectification.remap_x;
		undistort_rectify_map_y = rectification.remap_y;
	}
};

// Has to be standard layout because is embedded in TrackerPSMV.
static_assert(std::is_standard_layout<View>::value);

/*!
 * The core object of the PS Move tracking setup.
 *
 * @implements xrt_tracked_psmv
 * @implements xrt_frame_sink
 * @implements xrt_frame_node
 */
struct TrackerPSMV
{
public:
	struct xrt_tracked_psmv base = {};
	struct xrt_frame_sink sink = {};
	struct xrt_frame_node node = {};

	//! Frame waiting to be processed.
	struct xrt_frame *frame;

	//! Thread and lock helper.
	struct os_thread_helper oth;

	bool tracked = false;

	HelperDebugSink debug = {HelperDebugSink::AllAvailable};

	//! Have we received a new IMU sample.
	bool has_imu = false;

	struct
	{
		struct xrt_vec3 pos = {};
		struct xrt_quat rot = {};
	} fusion;

	View view[2];

	bool calibrated;
	bool mono;
	bool use_conefitting = true;

	cv::Mat disparity_to_depth;
	cv::Vec3d r_cam_translation;
	cv::Matx33d r_cam_rotation;

	cv::Ptr<cv::SimpleBlobDetector> sbd;

	std::shared_ptr<PSMVFusionInterface> filter;

	xrt_vec3 tracked_object_position;

	struct
	{
		bool view[2];
	} gui;
};

// Has to be standard layout because of first element casts we do.
static_assert(std::is_standard_layout<TrackerPSMV>::value);

/*!
 * @brief Perform per-view (two in a stereo camera image) processing on an
 * image, before tracking math is performed.
 *
 * Right now, this is mainly finding blobs/keypoints.
 */
static void
do_view(TrackerPSMV &t, View &view, cv::Mat &grey, cv::Mat &rgb)
{
	XRT_TRACE_MARKER();

	{
		XRT_TRACE_IDENT(remap);

		// Undistort and rectify the whole image.
		cv::remap(grey,                         // src
		          view.frame_undist_rectified,  // dst
		          view.undistort_rectify_map_x, // map1
		          view.undistort_rectify_map_y, // map2
		          cv::INTER_NEAREST,            // interpolation
		          cv::BORDER_CONSTANT,          // borderMode
		          cv::Scalar(0, 0, 0));         // borderValue
	}

	{
		XRT_TRACE_IDENT(threshold);

		cv::threshold(view.frame_undist_rectified, // src
		              view.frame_undist_rectified, // dst
		              32.0,                        // thresh
		              255.0,                       // maxval
		              0);                          // type
	}

	{
		XRT_TRACE_IDENT(detect);

		// Do blob detection with our masks.
		//! @todo Re-enable masks.
		t.sbd->detect(view.frame_undist_rectified, // image
		              view.keypoints,              // keypoints
		              cv::noArray());              // mask
	}


	// Debug is wanted, draw the keypoints.
	if (rgb.cols > 0) {
		cv::drawKeypoints(view.frame_undist_rectified,                // image
		                  view.keypoints,                             // keypoints
		                  rgb,                                        // outImage
		                  cv::Scalar(255, 0, 0),                      // color
		                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // flags
	}
}

// Cone Fitting Helper Functions
static Contour
combineContours(std::vector<Contour> &contours)
{
	Contour allPixels;
	if (contours.size() == 1) {
		allPixels = std::move(contours[0]);
		contours[0].clear();
	} else if (contours.size() > 1) {
		auto totalPixels =
		    std::accumulate(contours.begin(), contours.end(), 0,
		                    [](size_t prevSize, Contour const &c) { return prevSize + c.size(); });
		allPixels.reserve(totalPixels);
		for (const auto &contour : contours) {
			allPixels.insert(allPixels.end(), contour.begin(), contour.end());
		}
		std::cout << "We found " << contours.size() << " contours with a total of " << totalPixels
		          << " pixels.\n";
	}

	return allPixels;
}

static inline std::vector<Contour>
getContours(cv::Mat const &frame)
{
	// Clean up noise with an erode then dilate.
	cv::Mat temp;
	cv::erode(frame, temp, cv::Mat(), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
	cv::dilate(temp, frame, cv::Mat(), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

	std::vector<Contour> output;
	//! @todo we can probably do better than this generic algorithm.
	//! @todo we also need to return the associated "outside" pixel so we
	//! can get the "between" vector
	cv::findContours(frame, output, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	return output;
}

static void
plot_points_by_index(cv::Mat &frame, Contour const &allPixels, std::vector<size_t> const &indices, cv::Vec3b color)
{
	for (auto index : indices) {
		auto px = allPixels[index];
		frame.at<cv::Vec3b>(px.y, px.x) = color;
	}
}

// Plot a crosshair into `frame` at `center`, projected into the view's frame of reference.
static void
plot_projected_crosshair(View &view, cv::Mat &frame, cv::Vec3f const &center, float diameter, cv::Scalar color)
{
	// Get rvec and tvec from calibration pose.
	xrt_vec3 cam_pos = view.calibration_transform.position;
	xrt_vec3 cam_angles = {};
	math_quat_to_euler_angles(&view.calibration_transform.orientation, &cam_angles);
	cv::Vec3f rvec = {cam_angles.x, cam_angles.y, cam_angles.z};
	cv::Vec3f tvec = {cam_pos.x, cam_pos.y, cam_pos.z};

	// Each side is half the diameter.
	float radius = diameter / 2;
	std::vector<cv::Point2f> cross_tips = {{-radius, 0}, {radius, 0}, {0, radius}, {0, -radius}};

	std::vector<cv::Point2f> projected(1);
	cv::projectPoints(center, rvec, tvec, view.intrinsics, view.distortion, projected);

	for (int i = 0; i < 4; i++) {
		cross_tips[i] += projected[0];
	}

	cv::line(frame, cross_tips[0], cross_tips[1], color);
	cv::line(frame, cross_tips[2], cross_tips[3], color);
}

static bool
do_view_cone(TrackerPSMV &t, View &view, cv::Mat &grey, cv::Mat &rgb)
{
	auto contours = getContours(grey);
	auto allPixels = combineContours(contours);
	if (allPixels.empty()) {
		return false;
	}

	cv::drawContours(rgb, contours, -1, cv::Vec3b(0, 255, 0));

	std::vector<cv::Vec3f> directions;
	directions.reserve(allPixels.size());

	for (const auto &point : allPixels) {
		cv::Vec3f normalizedVec = view.norm_coords->getNormalizedVector(point);
		directions.emplace_back(normalizedVec);
	}

	cv::Vec3f position;
	std::vector<size_t> indices;

	if (view.cone_fitter->fit_cone_and_get_inlier_indices(directions, MinPoints, SphereRadius, position, indices)) {
		// Debug is wanted, draw the keypoints.
		if (rgb.cols > 0) {
			plot_points_by_index(rgb, allPixels, indices, cv::Vec3b(255, 0, 0));

			// Invert x and y axes (a sort-of hack to get the crosshair to show up at the right spot).
			cv::Vec3f pos = {-position[0], -position[1], position[2]};
			plot_projected_crosshair(view, rgb, pos, 10, cv::Vec3b(255, 0, 0));
		}
		// y axis in world is inverted from camera-land
		position[1] = -position[1];
		view.position = position;
		return true;
	}

	return false;
}

static void
incorporate_cone_fit_tracking(TrackerPSMV &t, View &view, timepoint_ns timestamp)
{
	xrt_vec3 pos = {view.position[0], view.position[1], view.position[2]};
	if (view.transform_valid) {
		math_pose_transform_point(&view.calibration_transform, &pos, &pos);
	}

	// for debug view
	view.transformed_pos = pos;

	t.tracked_object_position = pos;
	// std::cout << "Transformed position is [" << pos.x << ", " << pos.y
	//           << ", " << pos.z << "]\n";
	//! @todo these are crude guesses.
	const xrt_vec3 variance = {0.0001, 0.0001, 0.001};
	// xrt_pose_sample p = {.timestamp_ns = timestamp, .pose = {.orientation = XRT_QUAT_IDENTITY, .position=pos}};
	// t.filter->process_pose(&p, &variance, NULL, 15);
	t.filter->process_3d_vision_data(timestamp, &pos, &variance, &sphere_lever_arm,
	                                 //! @todo tune cutoff for residual
	                                 //! arbitrarily "too large"
	                                 15);
}

/*!
 * @brief Helper struct that keeps the value that produces the lowest "score" as
 * computed by your functor.
 *
 * Having this as a struct with a method, instead of a single "algorithm"-style
 * function, lets you keep your complicated filtering logic in your own
 * loop, calling in when you have a new candidate for "best".
 *
 * @note Create by calling make_lowest_score_finder() with your
 * function/lambda that takes an element and returns the score, to deduce the
 * un-spellable typename of the lambda.
 *
 * @tparam ValueType The type of a single element value - whatever you want to
 * assign a score to.
 * @tparam FunctionType The type of your functor/lambda that turns a ValueType
 * into a float "score". Usually deduced.
 */
template <typename ValueType, typename FunctionType> struct FindLowestScore
{
	const FunctionType score_functor;
	bool got_one{false};
	ValueType best{};
	float best_score{0};

	void
	handle_candidate(ValueType val)
	{
		float score = score_functor(val);
		if (!got_one || score < best_score) {
			best = val;
			best_score = score;
			got_one = true;
		}
	}
};


//! Factory function for FindLowestScore to deduce the functor type.
template <typename ValueType, typename FunctionType>
static FindLowestScore<ValueType, FunctionType>
make_lowest_score_finder(FunctionType scoreFunctor)
{
	return FindLowestScore<ValueType, FunctionType>{scoreFunctor};
}

//! Convert our 2d point + disparities into 3d points.
static cv::Point3f
world_point_from_blobs(const cv::Point2f &left, const cv::Point2f &right, const cv::Matx44d &disparity_to_depth)
{
	float disp = left.x - right.x;
	cv::Vec4d xydw(left.x, left.y, disp, 1.0f);

	// Transform
	cv::Vec4d h_world = disparity_to_depth * xydw;

	// Divide by scale to get 3D vector from homogeneous coordinate.
	cv::Point3f world_point(      //
	    h_world[0] / h_world[3],  //
	    h_world[1] / h_world[3],  //
	    h_world[2] / h_world[3]); //

	/*
	 * OpenCV camera space is right handed, -Y up, +Z forwards but
	 * Monados camera space is right handed, +Y up, -Z forwards so we need
	 * to invert y and z.
	 */
	world_point.y = -world_point.y;
	world_point.z = -world_point.z;

	return world_point;
}

/*!
 * @brief Perform tracking computations on a frame of video data.
 */
static void
process(TrackerPSMV &t, struct xrt_frame *xf)
{
	XRT_TRACE_MARKER();

	// Only IMU data: nothing to do
	if (xf == NULL) {
		return;
	}

	// Wrong type of frame: unreference and return?
	if (xf->format != XRT_FORMAT_L8) {
		xrt_frame_reference(&xf, NULL);
		return;
	}

	if (!t.calibrated) {
		return;
	}

	int view_count = t.mono ? 1 : 2;
	int cols = xf->width / view_count;
	int rows = xf->height;
	int stride = xf->stride;
	timepoint_ns timestamp = xf->timestamp;

	// Create the debug frame if needed.
	t.debug.refresh(xf);

	// TODO: Allow switching between conefitting and disparity-based tracking
	if (t.use_conefitting) {
		for (int view = 0; view < view_count; view++) {
			cv::Mat grey(rows, cols, CV_8UC1, xf->data + view * cols, stride);

			// TODO: Should we use frame_undist_rectified in the debug view?
			if (t.debug.frame != NULL) {
				// We don't use drawKeypoints, so we have to put something in
				// the buffers ourselves first.
				cv::cvtColor(grey, t.debug.rgb[view], cv::COLOR_GRAY2BGR);
			}

			t.view[view].position_valid = do_view_cone(t, t.view[view], grey, t.debug.rgb[view]);

			if (t.view[view].position_valid) {
				incorporate_cone_fit_tracking(t, t.view[view], timestamp);
			}
		}
	} else {
		cv::Point3f last_point(t.tracked_object_position.x, t.tracked_object_position.y,
		                       t.tracked_object_position.z);
		auto nearest_world = make_lowest_score_finder<cv::Point3f>([&](const cv::Point3f &world_point) {
			// Basically L2 norm, I think.
			auto dist = world_point - last_point;
			return dist.dot(dist);
		});

		t.view[0].keypoints.clear();
		t.view[1].keypoints.clear();

		cv::Mat l_grey(rows, cols, CV_8UC1, xf->data, stride);
		cv::Mat r_grey(rows, cols, CV_8UC1, xf->data + cols, stride);

		do_view(t, t.view[0], l_grey, t.debug.rgb[0]);
		do_view(t, t.view[1], r_grey, t.debug.rgb[1]);

		const cv::Matx44d disparity_to_depth = static_cast<cv::Matx44d>(t.disparity_to_depth);
		// do some basic matching to come up with likely disparity-pairs.

		for (const cv::KeyPoint &l_keypoint : t.view[0].keypoints) {
			cv::Point2f l_blob = l_keypoint.pt;

			auto nearest_blob = make_lowest_score_finder<cv::Point2f>(
			    [&](const cv::Point2f &r_blob) { return l_blob.x - r_blob.x; });

			for (const cv::KeyPoint &r_keypoint : t.view[1].keypoints) {
				cv::Point2f r_blob = r_keypoint.pt;
				// find closest point on same-ish scanline
				if ((l_blob.y < r_blob.y + 3) && (l_blob.y > r_blob.y - 3)) {
					nearest_blob.handle_candidate(r_blob);
				}
			}
			//! @todo do we need to avoid claiming the same counterpart
			//! several times?
			if (nearest_blob.got_one) {
				cv::Point3f pt = world_point_from_blobs(l_blob, nearest_blob.best, disparity_to_depth);
				nearest_world.handle_candidate(pt);
			}
		}

		if (nearest_world.got_one) {
			cv::Point3f world_point = nearest_world.best;
			// update internal state
			t.tracked_object_position = {world_point.x, world_point.y, world_point.z};

#if 0
			//! @todo something less arbitrary for the lever arm?
			//! This puts the origin approximately under the PS
			//! button.
			xrt_vec3 lever_arm{0.f, 0.09f, 0.f};
			//! @todo this should depend on distance
			// Weirdly, this is where *not* applying the
			// disparity-to-distance/rectification/etc would
			// simplify things, since the measurement variance is
			// related to the image sensor. 1.e-4 means 1cm std dev.
			// Not sure how to estimate the depth variance without
			// some research.
			xrt_vec3 variance{1.e-4f, 1.e-4f, 4.e-4f};
#endif
			t.filter->process_3d_vision_data(0, &t.tracked_object_position, NULL, NULL,
			                                 //! @todo tune cutoff for residual arbitrarily "too large"
			                                 15);
		} else {
			t.filter->clear_position_tracked_flag();
		}
	}

	// We are done with the debug frame.
	t.debug.submit();

	// We are done with the frame.
	xrt_frame_reference(&xf, NULL);
}

/*!
 * @brief Tracker processing thread function
 */
static void
run(TrackerPSMV &t)
{
	U_TRACE_SET_THREAD_NAME("PSMV");

	struct xrt_frame *frame = NULL;

	os_thread_helper_lock(&t.oth);

	while (os_thread_helper_is_running_locked(&t.oth)) {

		// No data
		if (!t.has_imu && t.frame == NULL) {
			os_thread_helper_wait_locked(&t.oth);

			/*
			 * Loop back to the top to check if we should stop,
			 * also handles spurious wakeups by re-checking the
			 * condition in the if case. Essentially two loops.
			 */
			continue;
		}

		// Take a reference on the current frame, this keeps it alive
		// if it is replaced during the consumer processing it, but
		// we no longer need to hold onto the frame on the queue we
		// just move the pointer.
		frame = t.frame;
		t.frame = NULL;

		// Unlock the mutex when we do the work.
		os_thread_helper_unlock(&t.oth);

		process(t, frame);

		// Have to lock it again.
		os_thread_helper_lock(&t.oth);
	}

	os_thread_helper_unlock(&t.oth);
}

/*!
 * @brief Retrieves a pose from the filter.
 */
static void
get_pose(TrackerPSMV &t, enum xrt_input_name name, timepoint_ns when_ns, struct xrt_space_relation *out_relation)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	if (name == XRT_INPUT_PSMV_BALL_CENTER_POSE) {
		out_relation->pose.position = t.tracked_object_position;
		out_relation->pose.orientation.x = 0.0f;
		out_relation->pose.orientation.y = 0.0f;
		out_relation->pose.orientation.z = 0.0f;
		out_relation->pose.orientation.w = 1.0f;

		out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                                                               XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

		os_thread_helper_unlock(&t.oth);
		return;
	}

	t.filter->get_prediction(when_ns, out_relation);

	os_thread_helper_unlock(&t.oth);
}

static void
imu_data(TrackerPSMV &t, timepoint_ns timestamp_ns, struct xrt_tracking_sample *sample)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}
	t.filter->process_imu_data(timestamp_ns, sample, NULL);

	os_thread_helper_unlock(&t.oth);
}

static void
frame(TrackerPSMV &t, struct xrt_frame *xf)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	xrt_frame_reference(&t.frame, xf);
	// Wake up the thread.
	os_thread_helper_signal_locked(&t.oth);

	os_thread_helper_unlock(&t.oth);
}

static void
break_apart(TrackerPSMV &t)
{
	os_thread_helper_stop_and_wait(&t.oth);
}

} // namespace xrt::auxiliary::tracking::psmv

using xrt::auxiliary::tracking::psmv::TrackerPSMV;

/*
 *
 * C wrapper functions.
 *
 */

extern "C" void
t_psmv_push_imu(struct xrt_tracked_psmv *xtmv, timepoint_ns timestamp_ns, struct xrt_tracking_sample *sample)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	imu_data(t, timestamp_ns, sample);
}

extern "C" void
t_psmv_get_tracked_pose(struct xrt_tracked_psmv *xtmv,
                        enum xrt_input_name name,
                        timepoint_ns when_ns,
                        struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	get_pose(t, name, when_ns, out_relation);
}

extern "C" void
t_psmv_fake_destroy(struct xrt_tracked_psmv *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	(void)t;
	// Not the real destroy function
}

extern "C" void
t_psmv_sink_push_frame(struct xrt_frame_sink *xsink, struct xrt_frame *xf)
{
	auto &t = *container_of(xsink, TrackerPSMV, sink);
	frame(t, xf);
}

extern "C" void
t_psmv_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerPSMV, node);
	break_apart(t);
}

extern "C" void
t_psmv_node_destroy(struct xrt_frame_node *node)
{
	auto *t_ptr = container_of(node, TrackerPSMV, node);
	os_thread_helper_destroy(&t_ptr->oth);

	// Tidy variable setup.
	u_var_remove_root(t_ptr);

	delete t_ptr;
}

extern "C" void *
t_psmv_run(void *ptr)
{
	auto &t = *(TrackerPSMV *)ptr;
	run(t);
	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

extern "C" int
t_psmv_start(struct xrt_tracked_psmv *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	return os_thread_helper_start(&t.oth, t_psmv_run, &t);
}

extern "C" int
t_psmv_create(struct xrt_frame_context *xfctx,
              struct xrt_colour_rgb_f32 *rgb,
              struct t_stereo_camera_calibration *data,
              struct xrt_tracked_psmv **out_xtmv,
              struct xrt_frame_sink **out_sink)
{
	XRT_TRACE_MARKER();

	U_LOG_D("Creating PSMV tracker.");

	auto &t = *(new TrackerPSMV());
	int ret;

	t.mono = data->mono;
	t.base.get_tracked_pose = t_psmv_get_tracked_pose;
	t.base.push_imu = t_psmv_push_imu;
	t.base.destroy = t_psmv_fake_destroy;
	t.base.colour = *rgb;
	t.sink.push_frame = t_psmv_sink_push_frame;
	t.node.break_apart = t_psmv_node_break_apart;
	t.node.destroy = t_psmv_node_destroy;
	t.fusion.rot.x = 0.0f;
	t.fusion.rot.y = 0.0f;
	t.fusion.rot.z = 0.0f;
	t.fusion.rot.w = 1.0f;
	t.filter = PSMVFusionInterface::create();

	ret = os_thread_helper_init(&t.oth);
	if (ret != 0) {
		delete (&t);
		return ret;
	}

	static int hack = 0;
	switch (hack++) {
	case 0:
		t.fusion.pos.x = -0.3f;
		t.fusion.pos.y = 1.3f;
		t.fusion.pos.z = -0.5f;
		break;
	case 1:
		t.fusion.pos.x = 0.3f;
		t.fusion.pos.y = 1.3f;
		t.fusion.pos.z = -0.5f;
		break;
	default:
		t.fusion.pos.x = 0.0f;
		t.fusion.pos.y = 0.8f + hack * 0.1f;
		t.fusion.pos.z = -0.5f;
		break;
	}

	StereoRectificationMaps rectify(data);
	StereoCameraCalibrationWrapper wrapped(data);

	t.view[0].populate_from_calib(data->view[0], rectify.view[0].rectify);
	t.view[1].populate_from_calib(data->view[1], rectify.view[1].rectify);
	t.disparity_to_depth = rectify.disparity_to_depth_mat;
	t.r_cam_rotation = wrapped.camera_rotation_mat;
	t.r_cam_translation = wrapped.camera_translation_mat;

	std::cout << t.disparity_to_depth << std::endl;

	{
		xrt_pose &transform = t.view[0].calibration_transform;
		t.view[0].transform_valid = true;
		transform.orientation.w = 1;
		transform.position.x = t.r_cam_translation[0];
		transform.position.y = t.r_cam_translation[1];
		transform.position.z = t.r_cam_translation[2];

		xrt_matrix_3x3 rot_mat;
		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				rot_mat.v[i * 3 + j] = t.r_cam_rotation(i, j);
			}
		}
		math_quat_from_matrix_3x3(&rot_mat, &transform.orientation);
	}

	t.calibrated = true;

	// clang-format off
	cv::SimpleBlobDetector::Params blob_params;
	blob_params.filterByArea = false;
	blob_params.filterByConvexity = true;
	blob_params.minConvexity = 0.8;
	blob_params.filterByInertia = false;
	blob_params.filterByColor = true;
	blob_params.blobColor = 255; // 0 or 255 - color comes from binarized image?
	blob_params.minArea = 1;
	blob_params.maxArea = 1000;
	blob_params.maxThreshold = 51; // using a wide threshold span slows things down bigtime
	blob_params.minThreshold = 50;
	blob_params.thresholdStep = 1;
	blob_params.minDistBetweenBlobs = 5;
	blob_params.minRepeatability = 1; // need this to avoid error?
	// clang-format on

	t.sbd = cv::SimpleBlobDetector::create(blob_params);
	xrt_frame_context_add(xfctx, &t.node);

	// Everything is safe, now setup the variable tracking.
	u_var_add_root(&t, "PSMV Tracker", true);
	u_var_add_vec3_f32(&t, &t.tracked_object_position, "last.ball.pos");
	u_var_add_sink_debug(&t, &t.debug.usd, "Debug");
	u_var_add_gui_header(&t, &t.gui.view[0], "View 0");
	u_var_add_ro_vec3_f32(&t, (xrt_vec3 *)&(t.view[0].position.val), "Raw position");
	u_var_add_ro_vec3_f32(&t, &(t.view[0].transformed_pos), "Transformed position");
	u_var_add_gui_header(&t, &t.gui.view[1], "View 1");
	u_var_add_ro_vec3_f32(&t, (xrt_vec3 *)&(t.view[1].position.val), "Raw position");
	u_var_add_ro_vec3_f32(&t, &(t.view[1].transformed_pos), "Transformed position");

	*out_sink = &t.sink;
	*out_xtmv = &t.base;

	return 0;
}
