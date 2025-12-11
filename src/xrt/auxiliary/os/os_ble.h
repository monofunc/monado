// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Wrapper around OS native BLE functions.
 * @author Pete Black <pete.black@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 *
 * @ingroup aux_os
 */

#pragma once

#include "xrt/xrt_config_os.h"
#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OS_BLE_DEVICE_MAC_ADDRESS_SIZE 18
#define OS_BLE_DEVICE_NAME_SIZE 256

/*!
 * @interface os_ble_device
 * Representing a single ble notify attribute on a device.
 *
 * @ingroup aux_os
 */
struct os_ble_device
{
	int (*read)(struct os_ble_device *ble_dev, uint8_t *data, size_t size, int milliseconds);

	int (*write)(struct os_ble_device *ble_dev, const uint8_t *data, size_t size);

	bool (*is_connected)(const struct os_ble_device *ble_dev);

	bool (*get_address)(const struct os_ble_device *ble_dev, char out_address[OS_BLE_DEVICE_MAC_ADDRESS_SIZE]);

	bool (*get_name)(const struct os_ble_device *ble_dev, char out_name[OS_BLE_DEVICE_NAME_SIZE]);

	void (*destroy)(struct os_ble_device *ble_dev);
};

/*!
 * Check if the given ble device is connected.
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline bool
os_ble_is_connected(const struct os_ble_device *ble_dev)
{
	return ble_dev->is_connected(ble_dev);
}

/*!
 * Get the MAC address of the given ble device.
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline bool
os_ble_get_address(const struct os_ble_device *ble_dev, char out_address[OS_BLE_DEVICE_MAC_ADDRESS_SIZE])
{
	return ble_dev->get_address(ble_dev, out_address);
}

/*!
 * Get the name of the given ble device.
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline bool
os_ble_get_name(const struct os_ble_device *ble_dev, char out_name[OS_BLE_DEVICE_NAME_SIZE])
{
	return ble_dev->get_name(ble_dev, out_name);
}

/*!
 * Read data from the ble file descriptor, if any, from the given bledevice.
 *
 * If milliseconds are negative, this call blocks indefinitely, 0 polls,
 * and positive will block for that amount of milliseconds.
 *
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline int
os_ble_read(struct os_ble_device *ble_dev, uint8_t *data, size_t size, int milliseconds)
{
	return ble_dev->read(ble_dev, data, size, milliseconds);
}

/*!
 * Write data to the ble file descriptor, if any, from the given bledevice.
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline int
os_ble_write(struct os_ble_device *ble_dev, const uint8_t *data, size_t size)
{
	return ble_dev->write(ble_dev, data, size);
}

/*!
 * Close and free the given device, does null checking and zeroing.
 *
 * @ingroup aux_os
 */
XRT_MAYBE_UNUSED static inline void
os_ble_destroy(struct os_ble_device **ble_dev_ptr)
{
	struct os_ble_device *ble_dev = *ble_dev_ptr;
	if (ble_dev == NULL) {
		return;
	}

	ble_dev->destroy(ble_dev);
	*ble_dev_ptr = NULL;
}

#if defined(XRT_OS_LINUX) || defined(XRT_OS_ANDROID)
/*!
 * brief: Opens a BLE device with given notify and write characteristic UUIDs.
 *
 * @param dev_uuid           The device / service UUID to open.
 * @param notify_char_uuid   The characteristic UUID to use for notifications.
 * @param write_char_uuid    The characteristic UUID to use for writing, can be NULL if not used.
 * @param device_name        The name of the device to connect to. Can be NULL to skip name filtering.
 * @param major_device_class The major device class to filter devices. Use -1 to skip filtering.
 * @param out_ble           The returned BLE device.
 *
 * @returns Negative on failure, zero on no device found and positive if a
 *          device has been found.
 *
 * @ingroup aux_os
 */
int
os_ble_open(const char *dev_uuid,
            const char *notify_char_uuid,
            const char *write_char_uuid,
            const char *device_name,
            int major_device_class,
            struct os_ble_device **out_ble);

/*!
 * @brief Returns a notification endpoint from the given device uuid and char uuid.
 *
 * @param dev_uuid          The device / service UUID to open.
 * @param notify_char_uuid  The characteristic UUID to use for notifications.
 * @param out_ble           The returned BLE device.
 *
 * @returns Negative on failure, zero on no device found and positive if a
 *          device has been found.
 *
 * @ingroup aux_os
 */
int
os_ble_notify_open(const char *dev_uuid, const char *char_uuid, struct os_ble_device **out_ble);

/*!
 * Returns write startpoints from the given device uuid and char uuid.
 *
 * @returns Negative on failure, zero on no device found and positive if a
 *          device has been found.
 *
 * @ingroup aux_os
 */
int
os_ble_broadcast_write_value(const char *service_uuid, const char *char_uuid, uint8_t value);
#endif


#ifdef __cplusplus
}
#endif
