/*!
 * @file
 * @brief  Information and configuration window for CAVE-specific options.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

//

#include "vroom_device.h"
#include "vroom_flystick.h"
#include "vroom_debug.h"
#include <cassert>

#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include <glad/gl.h>

#include <SDL2/SDL.h>

#include "joycon.h"
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_sdl2.h"
#include "imgui/backends/imgui_impl_opengl3.h"


typedef int32_t i32;
typedef uint32_t u32;

#define WIDTH 1280
#define HEIGHT 720

bool running = true;

int64_t reset_frame;
int64_t reset_time;


int
vroom_debug_window(struct vroom_device *vroom)
{

	int width = WIDTH, height = HEIGHT;

	// Set up window
	u32 WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;
	SDL_Window *Window = SDL_CreateWindow("Informations VROOM", 64, 64, width, height, WindowFlags);
	assert(Window);
	SDL_GLContext Context = SDL_GL_CreateContext(Window);
	SDL_GL_MakeCurrent(Window, Context);
	SDL_GL_SetSwapInterval(1); // Synchro verticale

	// Init GLAD context
	int version = gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress);
	printf("GL %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);

	// Set up IMGUI
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	io.IniFilename = "imgui_vroom.ini";
	ImGui::StyleColorsLight();

	ImGui_ImplSDL2_InitForOpenGL(Window, Context);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	ImVec4 clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	// State
	bool showDemoWindow = false;
	xrt_vec3 angles{0, 0, 0};
	angles.x = angles.y = angles.z = 0;
	char buffer[1024];

	reset_time = vroom->created_ns;
	reset_frame = 0;

	bool vrpn_corrected = false;

	while (running) {
		SDL_Event event;

		// Handle SDL events
		while (SDL_PollEvent(&event)) {
			ImGui_ImplSDL2_ProcessEvent(&event);
			if (event.type == SDL_QUIT)
				running = false;
			if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
			    event.window.windowID == SDL_GetWindowID(Window))
				running = false;
		}

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		// Statistics
		{
			ImGui::SetNextWindowPos(ImVec2(16, 16), ImGuiCond_Once);
			ImGui::SetNextWindowSize(ImVec2(368, 100), ImGuiCond_Once);
			ImGui::Begin("Statistics");

			ImGui::Text("Frame: %llu", vroom->frame_count);

			int64_t current_time = os_monotonic_get_ns();
			int64_t total_time_ns = (current_time - reset_time);
			double total_time_s = time_ns_to_s(total_time_ns);
			double mean_frametime = total_time_s / (double)(vroom->frame_count - reset_frame);

			ImGui::Text("Avg framerate: %.1f FPS (%.1f ms)", 1 / mean_frametime, mean_frametime * 1000);
			if (ImGui::Button("Reset framerate")) {
				reset_frame = vroom->frame_count;
				reset_time = os_monotonic_get_ns();
			}

			ImGui::End();
		}

		// Configuration file
		{
			ImGui::SetNextWindowPos(ImVec2(16, 128), ImGuiCond_Once);
			ImGui::SetNextWindowSize(ImVec2(368, 500), ImGuiCond_Once);
			ImGui::Begin("Configuration");

			auto &cfg = vroom->config;

			ImGui::Text("stereo: %s", cfg->stereo ? "true" : "false");

			ImGui::Text("display_count: %u", cfg->display_count);

			snprintf(buffer, 1024, "displays (%d items)", cfg->display_count);
			if (ImGui::TreeNode(buffer)) {

				for (int i = 0; i < cfg->display_count; i++) {
					auto &display = cfg->displays[i];

					snprintf(buffer, 1024, "displays[%d]", i);
					if (ImGui::TreeNode(buffer)) {

						ImGui::Text("name: %s", display.name);

						ImGui::Text("position: %f %f %f", display.position.x, display.position.y, display.position.z);
						ImGui::Text("rotation: %f %f %f", display.rotation.x, display.rotation.y, display.rotation.z);

						ImGui::Text("dimensions: %f %f", display.dimensions.x, display.dimensions.y);

						ImGui::Text("screenPosition: %f %f %f", display.screenPosition.x, display.screenPosition.y);
						ImGui::Text("screenSize: %f %f %f", display.screenSize.x, display.screenSize.y);

						ImGui::Text("textureSize: %f %f %f", display.textureSize.x, display.textureSize.y);

						ImGui::TreePop();
					}
				}

				ImGui::TreePop();
			}

			ImGui::Text("controller_type: %s", cfg->controller_type);

			if (ImGui::TreeNode("tracking")) {

				auto& tr = cfg->tracking;

				ImGui::Text("system: %s", tr.system);

				if (ImGui::TreeNode("dtrack")) {
					auto& dt = tr.dtrack;

					ImGui::Text("port: %d", dt.port);

					if (ImGui::TreeNode("bodies")) {

						ImGui::Text("head: %d", dt.bodies.head);
						ImGui::Text("left: %d", dt.bodies.left);
						ImGui::Text("right: %d", dt.bodies.right);

						ImGui::TreePop();
					}

					ImGui::TreePop();
				}

				if (ImGui::TreeNode("vrpn")) {
					auto& vr = tr.vrpn;

					if (ImGui::TreeNode("trackers")) {
						if (ImGui::TreeNode("head##vrpn_head")) {
							ImGui::Text("tracker: %s", vr.trackers.head.tracker);
							ImGui::Text("sensor: %d", vr.trackers.head.sensor);
							ImGui::TreePop();
						}

						if (ImGui::TreeNode("left##vrpn_left")) {
							ImGui::Text("tracker: %s", vr.trackers.left.tracker);
							ImGui::Text("sensor: %d", vr.trackers.left.sensor);
							ImGui::TreePop();
						}

						if (ImGui::TreeNode("right##vrpn_right")) {
							ImGui::Text("tracker: %s", vr.trackers.right.tracker);
							ImGui::Text("sensor: %d", vr.trackers.right.sensor);
							ImGui::TreePop();
						}

						ImGui::TreePop();

					}

					if (ImGui::TreeNode("space_correction")) {
						ImGui::Text("position: %f %f %f", vr.space_correction.pos.x, vr.space_correction.pos.y, vr.space_correction.pos.z);
						ImGui::Text("rotation: %f %f %f", vr.space_correction.rot.x, vr.space_correction.rot.y, vr.space_correction.rot.z);

						if (ImGui::TreeNode("mirror")) {
							ImGui::Text("x: %s", vr.space_correction.mirror.x ? "true" : "false");
							ImGui::Text("y: %s", vr.space_correction.mirror.y ? "true" : "false");
							ImGui::Text("z: %s", vr.space_correction.mirror.z ? "true" : "false");
							ImGui::TreePop();
						}

						ImGui::TreePop();
					}

				}

				ImGui::TreePop();
			}

			ImGui::End();
		}

		// Options
		{
			ImGui::SetNextWindowPos(ImVec2(16, 640), ImGuiCond_Once);
			ImGui::SetNextWindowSize(ImVec2(368, 100), ImGuiCond_Once);
			ImGui::Begin("Options");

			ImGui::BeginDisabled(!vroom->config->stereo);
			if (!vroom->config->stereo)
				ImGui::Text("Stereoscopy is disabled in the configuration file");
			ImGui::Checkbox("Enable stereoscopy", &vroom->enable_3d);
			ImGui::Checkbox("Invert eyes", &vroom->invert_eyes);
			ImGui::SliderFloat("IPD", &vroom->ipd, 0.050f, 0.072f, "%0.3f m");
			ImGui::EndDisabled();

			ImGui::End();
		}

		// Tracking
		{
			ImGui::SetNextWindowPos(ImVec2(490, 16), ImGuiCond_FirstUseEver, ImVec2(0, 0));
			ImGui::SetNextWindowSize(ImVec2(358, 424), ImGuiCond_FirstUseEver);
			ImGui::Begin("Tracking");

			ImGui::Text("Current system: %s", vroom->config->tracking.system);

			// Head
			{
				ImGui::SeparatorText("Head");
				auto p = vroom->pose;
				ImGui::Text("Position (xyz ): %6.3fm %6.3fm %6.3fm",
				            p.position.x, p.position.y, p.position.z);
				ImGui::Text("Rotation (xyzw): %6.3f  %6.3f  %6.3f  %6.3f",
				            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			}

			// Flystick
			if (vroom->flystick) {
				ImGui::SeparatorText("FlyStick");
				auto p = vroom->flystick->pose;
				ImGui::Text("Position (xyz ): %6.3fm %6.3fm %6.3fm",
				            p.position.x, p.position.y, p.position.z);
				ImGui::Text("Rotation (xyzw): %6.3f  %6.3f  %6.3f  %6.3f",
				            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			}

			// Joy-Con (L)
			if (vroom->joycon_left) {
				ImGui::SeparatorText("Joy-Con (L)");
				if (!vroom->joycon_left->handle_set)
					ImGui::Text("No controller connected");
				else
					ImGui::Text("Connected - Handle: %d", vroom->joycon_left->handle);
				auto p = vroom->joycon_left->pose;
				ImGui::Text("Position (xyz ): %6.3fm %6.3fm %6.3fm",
				            p.position.x, p.position.y, p.position.z);
				ImGui::Text("Rotation (xyzw): %6.3f  %6.3f  %6.3f  %6.3f",
				            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			}

			// Joy-Con (R)
			if (vroom->joycon_right) {
				ImGui::SeparatorText("Joy-Con (R)");
				if (!vroom->joycon_right->handle_set)
					ImGui::Text("No controller connected");
				else
					ImGui::Text("Connected - Handle: %d", vroom->joycon_right->handle);
				auto p = vroom->joycon_right->pose;
				ImGui::Text("Position (xyz ): %6.3fm %6.3fm %6.3fm",
				            p.position.x, p.position.y, p.position.z);
				ImGui::Text("Rotation (xyzw): %6.3f  %6.3f  %6.3f  %6.3f",
				            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			}

			ImGui::End();
		}

		// VRPN Tracking Window
		if (vroom->vrpn != nullptr) {
			ImGui::SetNextWindowPos(ImVec2(500, 128), ImGuiCond_FirstUseEver, ImVec2(0, 0));
			ImGui::SetNextWindowSize(ImVec2(358, 424), ImGuiCond_FirstUseEver);
			ImGui::Begin("VRPN");

			bool any = false;
			auto *vrpn = vroom->vrpn;

			ImGui::Checkbox("Display corrected coordinates", &vrpn_corrected);

			if (ImGui::TreeNode("Space correction##scvrpn")) {
				ImGui::DragFloat3("Position (m)",
					reinterpret_cast<float *>(&vroom->vrpn->space_correction.pos), 0.001);
				ImGui::DragFloat3("Rotation (rad)",
					reinterpret_cast<float *>(&vroom->vrpn->space_correction.rot), 0.001, -M_PI, M_PI);
				ImGui::Checkbox("Mirror X", &vroom->vrpn->space_correction.mirrorX);
				ImGui::SameLine();
				ImGui::Checkbox("Mirror Y", &vroom->vrpn->space_correction.mirrorY);
				ImGui::SameLine();
				ImGui::Checkbox("Mirror Z", &vroom->vrpn->space_correction.mirrorZ);

				ImGui::TreePop();
			}

			ImGui::Separator();

			if (vrpn->head != nullptr) {
				auto pose = vrpn_corrected ? vrpn->get_corrected_pose(vrpn->headPose) : vrpn->headPose;

				ImGui::Text("Head: tracker \"%s\" (sensor %d)", vrpn->headTrackerName,
				            vrpn->headTrackerSensor);
				ImGui::Text("Position xyz : %6.3fm %6.3fm %6.3fm", pose.position.x, pose.position.y,
				            pose.position.z);
				ImGui::Text("Rotation xyzw: %6.3f  %6.3f  %6.3f  %6.3f", pose.orientation.x,
				            pose.orientation.y, pose.orientation.z, pose.orientation.w);
				ImGui::Separator();
				any = true;
			}

			if (vrpn->leftHand != nullptr) {
				auto pose = vrpn_corrected
					            ? vrpn->get_corrected_pose(vrpn->leftHandPose)
					            : vrpn->leftHandPose;

				ImGui::Text("Left hand: tracker \"%s\" (sensor %d)", vrpn->leftHandTrackerName,
				            vrpn->leftHandTrackerSensor);
				ImGui::Text("Position xyz : %6.3fm %6.3fm %6.3fm", pose.position.x, pose.position.y,
				            pose.position.z);
				ImGui::Text("Rotation xyzw: %6.3f  %6.3f  %6.3f  %6.3f", pose.orientation.x,
				            pose.orientation.y, pose.orientation.z, pose.orientation.w);
				ImGui::Separator();
				any = true;
			}

			if (vrpn->rightHand != nullptr) {
				auto pose =
					vrpn_corrected
						? vrpn->get_corrected_pose(vrpn->rightHandPose)
						: vrpn->rightHandPose;

				ImGui::Text("Right hand: tracker \"%s\" (sensor %d)", vrpn->rightHandTrackerName,
				            vrpn->rightHandTrackerSensor);
				ImGui::Text("Position xyz : %6.3fm %6.3fm %6.3fm", pose.position.x, pose.position.y,
				            pose.position.z);
				ImGui::Text("Rotation xyzw: %6.3f  %6.3f  %6.3f  %6.3f", pose.orientation.x,
				            pose.orientation.y, pose.orientation.z, pose.orientation.w);
				any = true;
			}

			if (!any) {
				ImGui::Text("No tracker detected");
			}

			ImGui::End();
		}

		// Overall debug
		{
			ImGui::SetNextWindowPos(ImVec2(500, 128), ImGuiCond_FirstUseEver, ImVec2(0, 0));
			ImGui::SetNextWindowSize(ImVec2(358, 424), ImGuiCond_FirstUseEver);
			ImGui::Begin("Debug");

			ImGui::SeparatorText("Positions");

			ImGui::DragFloat3("Head pos (xyz )", (float *)&vroom->pose.position, 0.001f);
			ImGui::DragFloat4("Head rot (xyzw)", (float *)&vroom->pose.orientation, 0.001f); // convert to degrees?
			if (ImGui::Button("Normalize rotation")) {
				math_quat_normalize(&vroom->pose.orientation);
			}

			ImGui::SeparatorText("Displays");

			if (ImGui::BeginTabBar("Displays")) {
				for (int i = 0; i < vroom->config->display_count; ++i) {
					auto &display_config = vroom->config->displays[i];
					auto &display_pose = vroom->display_transforms[i];

					char text[64]{0};
					if (display_config.name != nullptr) {
						strcpy_s(text, display_config.name);
					} else {
						snprintf(text, 64, "Display %d", i);
					}
					if (ImGui::BeginTabItem(text)) {
						ImGui::Text("Position      : %6.3fm %6.3fm %6.3fm",
						            display_config.position.x,
						            display_config.position.y, display_config.position.z);
						ImGui::Text("Rotation (deg): %6.3f° %6.3f° %6.3f°",
						            display_config.rotation.x,
						            display_config.rotation.y, display_config.rotation.z);
						ImGui::Text("Dimensions    : %6.3fm %6.3fm",
						            display_config.dimensions.x,
						            display_config.dimensions.y);

						ImGui::Text("Screen size: %5d x %5d", display_config.screenSize.x,
						            display_config.screenSize.y);
						ImGui::Text("Screen pos : %5d ; %5d", display_config.screenPosition.x,
						            display_config.screenPosition.y);

						xrt_vec3 transformed{
							vroom->pose.position.x + display_pose.position.x,
							vroom->pose.position.y + display_pose.position.y,
							vroom->pose.position.z + display_pose.position.z
						};

						math_quat_rotate_vec3(&display_pose.orientation, &transformed,
						                      &transformed);

						float left = transformed.x + display_config.dimensions.x / 2;
						float right = display_config.dimensions.x / 2 - transformed.x;
						float bottom = transformed.y + display_config.dimensions.y / 2;
						float top = display_config.dimensions.y / 2 - transformed.y;
						//! Distance to the screen.
						float depth = transformed.z;

						ImGui::Text("Transformed : %6.3fm %6.3fm %6.3fm", transformed.x,
						            transformed.y, transformed.z);

						ImGui::Text("Top   : %6.3fm", top);
						ImGui::Text("Bottom: %6.3fm", bottom);
						ImGui::Text("Left  : %6.3fm", left);
						ImGui::Text("Right : %6.3fm", right);
						ImGui::Text("Depth : %6.3fm", depth);

						ImGui::EndTabItem();
					}
				}
				ImGui::EndTabBar();
			}

			ImGui::End();
		}

		ImGui::Render();
		glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
		glClearColor(clearColor.x, clearColor.y, clearColor.z, 0.f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(Window);
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(Context);
	SDL_DestroyWindow(Window);
	SDL_Quit();

	return 0;
}

void
vroom_close_debug_window()
{
	running = false;
}