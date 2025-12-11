// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Stub BLE functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_os
 */

#include "os_ble.h"


int
os_ble_open(const char *dev_uuid,
            const char *notify_char_uuid,
            const char *write_char_uuid,
            const char *device_name,
            int major_device_class,
            struct os_ble_device **out_ble)
{
	(void)dev_uuid;
	(void)notify_char_uuid;
	(void)write_char_uuid;
	(void)device_name;
	(void)major_device_class;
	(void)out_ble;
	return -1;
}

int
os_ble_notify_open(const char *dev_uuid, const char *char_uuid, struct os_ble_device **out_ble)
{
	(void)dev_uuid;
	(void)char_uuid;
	(void)out_ble;
	return -1;
}

int
os_ble_broadcast_write_value(const char *service_uuid, const char *char_uuid, uint8_t value)
{
	(void)service_uuid;
	(void)char_uuid;
	(void)value;
	return -1;
}
