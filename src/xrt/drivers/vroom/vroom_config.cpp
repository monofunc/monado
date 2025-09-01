/*!
 * @file
 * @brief VROOM configuration file implementation
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 *
 * @ingroup drv_vroom
 */

#include "vroom_config.hpp"

#include <fstream>
#include <filesystem>
#include "cjson/cJSON.h"
#include <cstring>

#define CONFIG_FILEPATH "vroom.json"

// Helper macros

#define js_setval(var, json, key, type, value)                                                                         \
	{                                                                                                              \
		cJSON *obj = cJSON_GetObjectItem(json, key);                                                           \
		if (obj != nullptr) {                                                                                  \
			var = (type)obj->value;                                                                        \
		}                                                                                                      \
	}                                                                                                              \
	(void *)0

#define js_f(var, json, key) js_setval(var, json, key, float, valuedouble)

#define js_s(var, json, key) js_setval(var, json, key, uint16_t, valueint)
#define js_i(var, json, key) js_setval(var, json, key, int, valueint)

#define js_true(var, json, key)                                                                                        \
	{                                                                                                              \
		cJSON *obj = cJSON_GetObjectItem(json, key);                                                           \
		if (obj != nullptr) {                                                                                  \
			var = cJSON_IsTrue(obj);                                                                       \
		}                                                                                                      \
	}                                                                                                              \
	(void *)0

#define js_string(var, json, key)                                                                                      \
	{                                                                                                              \
		cJSON *obj = cJSON_GetObjectItem(json, key);                                                           \
		if (obj != nullptr) {                                                                                  \
			var = (char *)malloc(strlen(obj->valuestring) + 1);                                            \
			strcpy(var, obj->valuestring);                                                                 \
		} else {                                                                                               \
			var = nullptr;                                                                                 \
		}                                                                                                      \
	}                                                                                                              \
	(void *)0

vroom_config *
vroom_load_config()
{
	const char *filename = CONFIG_FILEPATH;

	// Make sure file exists
	if (!std::filesystem::exists(filename)) {
		// config file does not exist.
		return nullptr;
	}

	// Load file in memory
	auto size = std::filesystem::file_size(filename);
	std::string content(size, '\0');
	std::ifstream in(filename);
	in.read(&content[0], (unsigned int)size);

	// Parse file
	cJSON *json = cJSON_Parse(content.c_str());
	auto *cfg = new vroom_config{};

	// Stereo
	js_true(cfg->stereo, json, "stereo");

	// Views
	cJSON *viewsJson = cJSON_GetObjectItem(json, "displays");
	if (!cJSON_IsArray(viewsJson)) {
		fprintf(stderr, "views is not an array!");
		return nullptr;
	}

	cfg->display_count = cJSON_GetArraySize(viewsJson);
	cfg->displays = new vroom_display[cfg->display_count];

	for (size_t i = 0; i < cfg->display_count; ++i) {
		cJSON *viewJson = cJSON_GetArrayItem(viewsJson, (int)i);
		vroom_display &view = cfg->displays[i];

		{
			xrt_vec3 &pos = view.position;
			cJSON *posJson = cJSON_GetObjectItem(viewJson, "position");
			js_f(pos.x, posJson, "x");
			js_f(pos.y, posJson, "y");
			js_f(pos.z, posJson, "z");
		}

		{
			xrt_vec3 &rot = view.rotation;
			cJSON *rotJson = cJSON_GetObjectItem(viewJson, "rotation");
			js_f(rot.x, rotJson, "x");
			js_f(rot.y, rotJson, "y");
			js_f(rot.z, rotJson, "z");
		}

		{
			xrt_vec2 &dim = view.dimensions;
			cJSON *dimJson = cJSON_GetObjectItem(viewJson, "dimensions");
			js_f(dim.x, dimJson, "width");
			js_f(dim.y, dimJson, "height");
		}

		{
			xrt_vec2_i32 &scP = view.screenPosition;
			xrt_vec2_i32 &scS = view.screenSize;
			cJSON *scrJson = cJSON_GetObjectItem(viewJson, "screen");
			js_i(scP.x, scrJson, "x");
			js_i(scP.y, scrJson, "y");
			js_i(scS.x, scrJson, "width");
			js_i(scS.y, scrJson, "height");
		}

		{
			xrt_vec2_i32 &tex = view.textureSize;
			cJSON *texJson = cJSON_GetObjectItem(viewJson, "textureSize");
			if (texJson != nullptr && cJSON_IsObject(texJson)) {
				js_i(tex.x, texJson, "width");
				js_i(tex.y, texJson, "height");
			} else {
				tex.x = tex.y = -1;
			}
		}

		js_string(view.name, viewJson, "__name");
	}

	js_string(cfg->controller_type, json, "controller");

	// Tracking
	cJSON *trackingJson = cJSON_GetObjectItem(json, "tracking");

	js_string(cfg->tracking.system, trackingJson, "system");

	// DTrack

	if (strcmp(cfg->tracking.system, "dtrack") == 0) {
		cJSON *dtrackJson = cJSON_GetObjectItem(trackingJson, "dtrack");
		auto &dtrack = cfg->tracking.dtrack;
		js_s(dtrack.port, dtrackJson, "port");

		{
			cJSON *partJson = cJSON_GetObjectItem(dtrackJson, "bodies");
			auto &bodies = dtrack.bodies;
			js_i(bodies.head, partJson, "head");
			js_i(bodies.left, partJson, "left");
			js_i(bodies.right, partJson, "right");
		}
	}

	// VRPN
	if (strcmp(cfg->tracking.system, "vrpn") == 0) {
		cJSON *vrpnJson = cJSON_GetObjectItem(trackingJson, "vrpn");
		auto &vrpn = cfg->tracking.vrpn;

		// Trackers
		{
			cJSON *trackersJson = cJSON_GetObjectItem(vrpnJson, "trackers");

			// head
			{
				cJSON *partJson = cJSON_GetObjectItem(trackersJson, "head");
				auto &part = vrpn.trackers.head;
				js_string(part.tracker, partJson, "tracker");
				js_i(part.sensor, partJson, "sensor");
			}

			// left hand
			{
				cJSON *partJson = cJSON_GetObjectItem(trackersJson, "leftHand");
				auto &part = vrpn.trackers.left;
				js_string(part.tracker, partJson, "tracker");
				js_i(part.sensor, partJson, "sensor");
			}

			// right hand
			{
				cJSON *partJson = cJSON_GetObjectItem(trackersJson, "rightHand");
				auto &part = vrpn.trackers.right;
				js_string(part.tracker, partJson, "tracker");
				js_i(part.sensor, partJson, "sensor");
			}
		}

		// space correction
		{
			cJSON *spaceJson = cJSON_GetObjectItem(vrpnJson, "spaceCorrection");
			auto &space = vrpn.space_correction;

			{
				xrt_vec3 &pos = space.pos;
				cJSON *posJson = cJSON_GetObjectItem(spaceJson, "pos");
				js_f(pos.x, posJson, "x");
				js_f(pos.y, posJson, "y");
				js_f(pos.z, posJson, "z");
			}

			{
				xrt_vec3 &rot = space.rot;
				cJSON *rotJson = cJSON_GetObjectItem(spaceJson, "rot");
				js_f(rot.x, rotJson, "x");
				js_f(rot.y, rotJson, "y");
				js_f(rot.z, rotJson, "z");
			}

			{
				auto &rs = space.mirror;
				cJSON *rotJson = cJSON_GetObjectItem(spaceJson, "mirror");
				js_true(rs.x, rotJson, "x");
				js_true(rs.y, rotJson, "y");
				js_true(rs.z, rotJson, "z");
			}
		}
	}

	return cfg;
}

void
set_charptr(char **dest, const char *src)
{
	*dest = new char[strlen(src) + 1]{0};
	strcpy(*dest, src);
}

vroom_config
vroom_create_config()
{
	// Create config
	vroom_config config;

	config.stereo = true;

	config.display_count = 1;
	config.displays = new vroom_display[1];

	set_charptr(&config.controller_type, "joycon");

	auto &display = config.displays[0];
	set_charptr(&display.name, "Face avant");

	display.position = {0, 1, 0};
	display.rotation = {0, 0, 0};
	display.dimensions = {4, 2};
	display.screenPosition = {0, 0};
	display.screenSize = {1280, 720};
	display.textureSize = {1024, 512};

	auto &tracking = config.tracking;
	set_charptr(&tracking.system, "dtrack");

	tracking.dtrack.port = 5000;
	tracking.dtrack.bodies.head = 0;
	tracking.dtrack.bodies.left = 1;
	tracking.dtrack.bodies.right = 2;

	set_charptr(&tracking.vrpn.trackers.head.tracker, "Head@localhost");
	tracking.vrpn.trackers.head.sensor = 0;
	set_charptr(&tracking.vrpn.trackers.left.tracker, "Left@localhost");
	tracking.vrpn.trackers.left.sensor = 0;
	set_charptr(&tracking.vrpn.trackers.right.tracker, "Right@localhost");
	tracking.vrpn.trackers.right.sensor = 0;

	tracking.vrpn.space_correction.pos = {0, 0, 0};
	tracking.vrpn.space_correction.rot = {0, 0, 0};
	tracking.vrpn.space_correction.mirror.x = false;
	tracking.vrpn.space_correction.mirror.y = false;
	tracking.vrpn.space_correction.mirror.z = false;

	return config;
}

void
vroom_save_config(const vroom_config *config, const char *filepath)
{
	// Construct JSON
	cJSON *root = cJSON_CreateObject();

	// Stereo
	cJSON_AddItemToObject(root, "stereo", cJSON_CreateBool(config->stereo));

	// Displays
	cJSON *displays = cJSON_CreateArray();
	cJSON_AddItemToObject(root, "displays", displays);

	for (size_t i = 0; i < config->display_count; i++) {
		auto &display = config->displays[i];
		cJSON *displayJson = cJSON_CreateObject();

		// Name
		cJSON_AddItemToObject(displayJson, "__name", cJSON_CreateString(display.name));

		// Position
		cJSON *position = cJSON_CreateObject();
		cJSON_AddItemToObject(position, "x", cJSON_CreateNumber(display.position.x));
		cJSON_AddItemToObject(position, "y", cJSON_CreateNumber(display.position.y));
		cJSON_AddItemToObject(position, "z", cJSON_CreateNumber(display.position.z));
		cJSON_AddItemToObject(displayJson, "position", position);

		// Rotation
		cJSON *rotation = cJSON_CreateObject();
		cJSON_AddItemToObject(rotation, "x", cJSON_CreateNumber(display.rotation.x));
		cJSON_AddItemToObject(rotation, "y", cJSON_CreateNumber(display.rotation.y));
		cJSON_AddItemToObject(rotation, "z", cJSON_CreateNumber(display.rotation.z));
		cJSON_AddItemToObject(displayJson, "rotation", rotation);

		// Dimensions
		cJSON *dimensions = cJSON_CreateObject();
		cJSON_AddItemToObject(dimensions, "width", cJSON_CreateNumber(display.dimensions.x));
		cJSON_AddItemToObject(dimensions, "height", cJSON_CreateNumber(display.dimensions.y));
		cJSON_AddItemToObject(displayJson, "dimensions", dimensions);

		// Screen
		cJSON *screen = cJSON_CreateObject();
		cJSON_AddItemToObject(screen, "x", cJSON_CreateNumber(display.screenPosition.x));
		cJSON_AddItemToObject(screen, "y", cJSON_CreateNumber(display.screenPosition.y));
		cJSON_AddItemToObject(screen, "width", cJSON_CreateNumber(display.screenSize.x));
		cJSON_AddItemToObject(screen, "height", cJSON_CreateNumber(display.screenSize.y));
		cJSON_AddItemToObject(displayJson, "screen", screen);

		// Texture
		cJSON *texture = cJSON_CreateObject();
		cJSON_AddItemToObject(texture, "width", cJSON_CreateNumber(display.textureSize.x));
		cJSON_AddItemToObject(texture, "height", cJSON_CreateNumber(display.textureSize.y));
		cJSON_AddItemToObject(displayJson, "textureSize", texture);

		// Add display to array
		cJSON_AddItemToArray(displays, displayJson);
	}


	cJSON_AddItemToObject(root, "controller", cJSON_CreateString(config->controller_type));

	// Tracking
	cJSON *tracking = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "tracking", tracking);

	cJSON_AddItemToObject(tracking, "system", cJSON_CreateString(config->tracking.system));

	// DTrack
	cJSON *dtrack = cJSON_CreateObject();
	cJSON_AddItemToObject(tracking, "dtrack", dtrack);
	cJSON_AddItemToObject(dtrack, "port", cJSON_CreateNumber(config->tracking.dtrack.port));

	cJSON *bodies = cJSON_CreateObject();
	cJSON_AddItemToObject(bodies, "head", cJSON_CreateNumber(config->tracking.dtrack.bodies.head));
	cJSON_AddItemToObject(bodies, "left", cJSON_CreateNumber(config->tracking.dtrack.bodies.left));
	cJSON_AddItemToObject(bodies, "right", cJSON_CreateNumber(config->tracking.dtrack.bodies.right));
	cJSON_AddItemToObject(dtrack, "bodies", bodies);

	// VRPN
	cJSON *vrpn = cJSON_CreateObject();
	cJSON_AddItemToObject(tracking, "vrpn", vrpn);

	// VRPN Trackers
	cJSON *trackers = cJSON_CreateObject();
	cJSON_AddItemToObject(vrpn, "trackers", trackers);

	cJSON *head = cJSON_CreateObject();
	cJSON_AddItemToObject(head, "tracker", cJSON_CreateString(config->tracking.vrpn.trackers.head.tracker));
	cJSON_AddItemToObject(head, "sensor", cJSON_CreateNumber(config->tracking.vrpn.trackers.head.sensor));
	cJSON_AddItemToObject(trackers, "head", head);

	cJSON *left = cJSON_CreateObject();
	cJSON_AddItemToObject(left, "tracker", cJSON_CreateString(config->tracking.vrpn.trackers.left.tracker));
	cJSON_AddItemToObject(left, "sensor", cJSON_CreateNumber(config->tracking.vrpn.trackers.left.sensor));
	cJSON_AddItemToObject(trackers, "leftHand", left);

	cJSON *right = cJSON_CreateObject();
	cJSON_AddItemToObject(right, "tracker", cJSON_CreateString(config->tracking.vrpn.trackers.right.tracker));
	cJSON_AddItemToObject(right, "sensor", cJSON_CreateNumber(config->tracking.vrpn.trackers.right.sensor));
	cJSON_AddItemToObject(trackers, "rightHand", right);

	// Space Correction
	cJSON *space = cJSON_CreateObject();
	cJSON_AddItemToObject(vrpn, "spaceCorrection", space);

	cJSON *spacePos = cJSON_CreateObject();
	cJSON_AddItemToObject(spacePos, "x", cJSON_CreateNumber(config->tracking.vrpn.space_correction.pos.x));
	cJSON_AddItemToObject(spacePos, "y", cJSON_CreateNumber(config->tracking.vrpn.space_correction.pos.y));
	cJSON_AddItemToObject(spacePos, "z", cJSON_CreateNumber(config->tracking.vrpn.space_correction.pos.z));
	cJSON_AddItemToObject(space, "pos", spacePos);

	cJSON *spaceRot = cJSON_CreateObject();
	cJSON_AddItemToObject(spaceRot, "x", cJSON_CreateNumber(config->tracking.vrpn.space_correction.rot.x));
	cJSON_AddItemToObject(spaceRot, "y", cJSON_CreateNumber(config->tracking.vrpn.space_correction.rot.y));
	cJSON_AddItemToObject(spaceRot, "z", cJSON_CreateNumber(config->tracking.vrpn.space_correction.rot.z));
	cJSON_AddItemToObject(space, "rot", spaceRot);

	cJSON *mirror = cJSON_CreateObject();
	cJSON_AddItemToObject(mirror, "x", cJSON_CreateBool(config->tracking.vrpn.space_correction.mirror.x));
	cJSON_AddItemToObject(mirror, "y", cJSON_CreateBool(config->tracking.vrpn.space_correction.mirror.y));
	cJSON_AddItemToObject(mirror, "z", cJSON_CreateBool(config->tracking.vrpn.space_correction.mirror.z));
	cJSON_AddItemToObject(space, "mirror", mirror);

	char *json_str = cJSON_Print(root);
	if (json_str) {
		std::ofstream out(filepath);
		if (out.is_open()) {
			out << json_str;
			out.close();
		}
		free(json_str);
	}
}
