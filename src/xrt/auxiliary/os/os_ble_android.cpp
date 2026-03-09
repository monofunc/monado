// Copyright 2023-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android Implementation of os_ble.h using AndroidBleDevice.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup aux_os
 */
#include "os_ble.h"

#include "xrt/xrt_config_android.h"
#include "util/u_logging.h"

#include "android/android_globals.h"
#include "android/android_load_class.hpp"
#include "android/org.freedesktop.monado.auxiliary.hpp"

#include <memory>
#include <mutex>
#include <utility>

using wrap::android::content::Context;
using wrap::org::freedesktop::monado::auxiliary::AndroidBleDevice;

namespace {

struct os_ble_device_android
{
	struct os_ble_device base = {};
	AndroidBleDevice aos_ble_device = {};
};

XRT_MAYBE_UNUSED inline const struct os_ble_device_android *
os_ble_device_android(const struct os_ble_device *ble_dev)
{
	return reinterpret_cast<const struct os_ble_device_android *>(ble_dev);
}

XRT_MAYBE_UNUSED inline struct os_ble_device_android *
os_ble_device_android(struct os_ble_device *ble_dev)
{
	return reinterpret_cast<struct os_ble_device_android *>(ble_dev);
}

inline bool
os_ble_device_android_is_valid(const struct os_ble_device_android *ble_dev)
{
	return ble_dev != nullptr && !ble_dev->aos_ble_device.isNull();
}

inline const jni::Class &
get_load_android_ble_device_class(struct _JavaVM *vm, jobject context)
{
	static std::once_flag aos_ble_device_class_init_flag;
	static jni::Class aos_ble_device_class{};

	using xrt::auxiliary::android::loadClassFromRuntimeApk;

	std::call_once(aos_ble_device_class_init_flag, [&]() {
		try {
			jni::init(vm);
			const auto clazz =
			    loadClassFromRuntimeApk(context, AndroidBleDevice::getFullyQualifiedTypeName());
			if (clazz.isNull()) {
				U_LOG_E("Could not load class '%s' from package '%s'",
				        AndroidBleDevice::getFullyQualifiedTypeName(), XRT_ANDROID_PACKAGE);
				return;
			}

			const auto class_handle = reinterpret_cast<jclass>(clazz.object().getHandle());

			//! Teach the wrapper our class before we start to use it.
			AndroidBleDevice::staticInitClass(class_handle);

			/*!
			 * The 0 is to avoid this being considered "temporary" and to
			 * create a global ref.
			 */
			const auto device_class = jni::Class(class_handle, 0);

			if (device_class.isNull()) {
				U_LOG_E("aos_ble_device_class is null!");
				return;
			}

			std::string clazz_name = device_class.getName();
			if (clazz_name != AndroidBleDevice::getFullyQualifiedTypeName()) {
				U_LOG_E("Unexpected class name: %s", clazz_name.c_str());
				return;
			}

			aos_ble_device_class = device_class;

		} catch (std::exception const &e) {
			U_LOG_E("Could not load class '%s' from package '%s', reason: %s",
			        AndroidBleDevice::getFullyQualifiedTypeName(), XRT_ANDROID_PACKAGE, e.what());
		}
	});
	return aos_ble_device_class;
}

inline int
os_ble_device_android_read(struct os_ble_device *ble_dev, uint8_t *data, size_t size, int milliseconds)
{
	if (data == nullptr || size == 0) {
		U_LOG_E("Invalid null parameter(s) passed to os_ble_device_android_read");
		return -1;
	}

	auto ble_device = os_ble_device_android(ble_dev);
	if (!os_ble_device_android_is_valid(ble_device)) {
		return -1;
	}
	try {
		const auto read_result = ble_device->aos_ble_device.read(milliseconds);
		read_result.getData({data, size});
		return read_result.getStatus();
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.read() failed, reason: %s", e.what());
	}
	return -1;
}

inline int
os_ble_device_android_write(struct os_ble_device *ble_dev, const uint8_t *data, size_t size)
{
	if (data == nullptr || size == 0) {
		U_LOG_E("Invalid null parameter(s) passed to os_ble_device_android_write");
		return -1;
	}

	auto ble_device = os_ble_device_android(ble_dev);
	if (!os_ble_device_android_is_valid(ble_device)) {
		return -1;
	}
	try {
		return ble_device->aos_ble_device.write({data, size}) ? 1 : -1;
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.write() failed, reason: %s", e.what());
	}
	return -1;
}

inline void
os_ble_device_android_destroy(struct os_ble_device *ble_dev)
{
	auto aos_ble_device = os_ble_device_android(ble_dev);
	if (aos_ble_device == nullptr) {
		return;
	}

	try {
		if (!aos_ble_device->aos_ble_device.isNull()) {
			aos_ble_device->aos_ble_device.destroy();
		}
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.destroy() failed, reason: %s", e.what());
	}
	delete aos_ble_device;
}

inline bool
os_ble_device_android_is_connected(const struct os_ble_device *ble_dev)
{
	auto ble_device = os_ble_device_android(ble_dev);
	if (!os_ble_device_android_is_valid(ble_device)) {
		return false;
	}

	try {
		return ble_device->aos_ble_device.isConnected();
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.isConnected() failed, reason: %s", e.what());
	}
	return false;
}

inline bool
os_ble_device_android_get_address(const struct os_ble_device *ble_dev, char out_address[OS_BLE_DEVICE_MAC_ADDRESS_SIZE])
{
	auto ble_device = os_ble_device_android(ble_dev);
	if (!os_ble_device_android_is_valid(ble_device)) {
		return false;
	}

	try {
		const auto address = ble_device->aos_ble_device.getAddress();
		if (address.empty()) {
			return false;
		}
		snprintf(out_address, OS_BLE_DEVICE_MAC_ADDRESS_SIZE, "%s", address.c_str());
		return true;
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.getAddress() failed, reason: %s", e.what());
	}
	return false;
}

inline bool
os_ble_device_android_get_name(const struct os_ble_device *ble_dev, char out_name[OS_BLE_DEVICE_NAME_SIZE])
{
	auto ble_device = os_ble_device_android(ble_dev);
	if (!os_ble_device_android_is_valid(ble_device)) {
		return false;
	}

	try {
		const auto name = ble_device->aos_ble_device.getName();
		if (name.empty()) {
			return false;
		}
		snprintf(out_name, OS_BLE_DEVICE_NAME_SIZE, "%s", name.c_str());
		return true;
	} catch (std::exception const &e) {
		U_LOG_E("AndroidBleDevice.getName() failed, reason: %s", e.what());
	}
	return false;
}

} // anonymous namespace

/*
 *
 * 'Exported' functions.
 *
 */
int
os_ble_broadcast_write_value(const char *service_uuid, const char *char_uuid, uint8_t value)
{
	(void)service_uuid;
	(void)char_uuid;
	(void)value;
	U_LOG_E("not implemented on Android");
	return -1;
}

int
os_ble_open(const char *dev_uuid,
            const char *notify_char_uuid,
            const char *write_char_uuid,
            const char *device_name,
            int major_device_class,
            struct os_ble_device **out_ble)
{
	if (dev_uuid == nullptr || notify_char_uuid == nullptr || out_ble == nullptr) {
		U_LOG_E("Invalid null parameter(s) passed to os_ble_open");
		return -1;
	}

	const auto vm = android_globals_get_vm();
	const auto context = reinterpret_cast<jobject>(android_globals_get_context());

	if (vm == nullptr || context == nullptr) {
		U_LOG_E("JNI VM or context are null!");
		return -1;
	}

	try {
		jni::init(vm);
		const auto &aos_ble_device_class = get_load_android_ble_device_class(vm, context);
		if (aos_ble_device_class.isNull()) {
			U_LOG_E("aos_ble_device_class is null!");
			return -1;
		}

		auto aos_ble_device_java =
		    AndroidBleDevice::openDevice(Context(context),                                  //
		                                 dev_uuid,                                          //
		                                 notify_char_uuid,                                  //
		                                 write_char_uuid != nullptr ? write_char_uuid : "", //
		                                 device_name != nullptr ? device_name : "",         //
		                                 major_device_class);                               //
		if (aos_ble_device_java.isNull()) {
			U_LOG_E("Could not open AndroidBleDevice with UUID %s", dev_uuid);
			return -1;
		}

		auto aos_ble_device = std::make_unique<struct os_ble_device_android>();
		aos_ble_device->aos_ble_device = std::move(aos_ble_device_java);
		aos_ble_device->base.read = os_ble_device_android_read;
		aos_ble_device->base.write = os_ble_device_android_write;
		aos_ble_device->base.is_connected = os_ble_device_android_is_connected;
		aos_ble_device->base.get_address = os_ble_device_android_get_address;
		aos_ble_device->base.get_name = os_ble_device_android_get_name;
		aos_ble_device->base.destroy = os_ble_device_android_destroy;

		*out_ble = &aos_ble_device.release()->base;
		return 1;

	} catch (std::exception const &e) {
		U_LOG_E("Failed to open AndroidBleDevice with UUID %s from package '%s', reason: %s", dev_uuid,
		        XRT_ANDROID_PACKAGE, e.what());
		return -1;
	}
}

int
os_ble_notify_open(const char *dev_uuid, const char *char_uuid, struct os_ble_device **out_ble)
{
	return os_ble_open(dev_uuid, char_uuid, nullptr, nullptr, -1, out_ble);
}
