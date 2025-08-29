#include <cstdio>
#include "vrpn.hpp"
#include "vroom_device.h"
#include "math/m_api.h"


static bool running = false;
static VroomVRPN *instance = nullptr;

struct HandlerUserdata {
	xrt_pose *pose;
	int sensor;
};

// Impl classe
VroomVRPN::VroomVRPN() {
	head = nullptr;
	leftHand = nullptr;
	rightHand = nullptr;

	headTrackerName = nullptr;
	leftHandTrackerName = nullptr;
	rightHandTrackerName = nullptr;
}

VroomVRPN::~VroomVRPN() {
	if (head != nullptr)
		delete head;
	if (leftHand != nullptr)
		delete leftHand;
	if (rightHand != nullptr)
		delete rightHand;
}

bool
VroomVRPN::mainLoop()
{
	// Process mainloop des trackers
	if (head != nullptr)
		head->mainloop();
	if (leftHand != nullptr)
		leftHand->mainloop();
	if (rightHand != nullptr)
		rightHand->mainloop();

	return true;
}

xrt_pose
VroomVRPN::get_corrected_pose(xrt_pose pose) {
	auto new_pose = pose;

	xrt_pose xform{
		{0,0, 0, 1},
		space_correction.pos
	};
	math_quat_from_euler_angles(&space_correction.rot, &xform.orientation);

	// math_pose_transform_point(&xform, &new_pose_2.position, &new_pose_2.position);
	math_quat_rotate_vec3(&xform.orientation, &new_pose.position, &new_pose.position);
	new_pose.position.x += xform.position.x;
	new_pose.position.y += xform.position.y;
	new_pose.position.z += xform.position.z;

	int mirrorCount = 0;

	// Mirror along X, Y and Z
	if (space_correction.mirrorX) { mirrorCount++; new_pose.orientation.x *= -1; }
	if (space_correction.mirrorY) { mirrorCount++; new_pose.orientation.y *= -1; }
	if (space_correction.mirrorZ) { mirrorCount++; new_pose.orientation.z *= -1; }

	// Go back to correct universe
	if (mirrorCount % 2 != 0) {
		new_pose.orientation.x *= -1;
		new_pose.orientation.y *= -1;
		new_pose.orientation.z *= -1;
	}

	return new_pose;
};

// Callbacks
void
handle_tracker(void* userdata, vrpn_TRACKERCB t)
{
	auto* data = reinterpret_cast<HandlerUserdata*>(userdata);
	xrt_pose *pose = data->pose;

	// Not the sensor we're interested in.
	if (data->sensor != t.sensor)
		return;

	pose->position.x = (float)t.pos[0];
	pose->position.y = (float)t.pos[1];
	pose->position.z = (float)t.pos[2];

	pose->orientation.x = (float)t.quat[0];
	pose->orientation.y = (float)t.quat[1];
	pose->orientation.z = (float)t.quat[2];
	pose->orientation.w = (float)t.quat[3];
}

void *
vroom_vrpn_run(void* ptr)
{
	running = true;

	instance = reinterpret_cast<VroomVRPN *>(ptr);

	// TODO: optimize when the tracker is the same for two or more targets (such as with DTrack)

	if (instance->headTrackerName != nullptr) {
		auto userdata = new HandlerUserdata{
			.pose = &instance->headPose,
			.sensor = instance->headTrackerSensor
		};

		instance->head = new vrpn_Tracker_Remote(instance->headTrackerName);
		instance->head->register_change_handler(userdata, handle_tracker);
	}

	if (instance->leftHandTrackerName != nullptr) {
		auto userdata = new HandlerUserdata{
		    .pose = &instance->leftHandPose,
		    .sensor = instance->leftHandTrackerSensor
		};

		instance->leftHand = new vrpn_Tracker_Remote(instance->leftHandTrackerName);
		instance->leftHand->register_change_handler(userdata, handle_tracker);
	}

	if (instance->rightHandTrackerName != nullptr) {
		auto userdata = new HandlerUserdata{
		    .pose = &instance->rightHandPose,
		    .sensor = instance->rightHandTrackerSensor
		};

		instance->rightHand = new vrpn_Tracker_Remote(instance->rightHandTrackerName);
		instance->rightHand->register_change_handler(userdata, handle_tracker);
	}
	while (running) {
		instance->mainLoop();
	}

	return 0;
}

void
vroom_vrpn_stop()
{
	running = false;
}

void
vrpn_update_pose(const vroom_device *vroom, int body, xrt_pose *out)
{
	auto *vrpn = vroom->vrpn;
	switch (body) {
	case 0:
		*out = vrpn->get_corrected_pose(vrpn->headPose);
		return;
	case 1:
		*out = vrpn->get_corrected_pose(vrpn->leftHandPose);
		return;
	case 2:
		*out = vrpn->get_corrected_pose(vrpn->rightHandPose);
		return;
	default:
		return;
	}
}


