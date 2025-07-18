// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Wrapper around OS native hid functions.
 * @author Simon Zeni <simon.zeni@collabora.com>
 *
 * @ingroup aux_os
 */

#include "os_hid.h"

#include <hidapi.h>

#include <stdlib.h>

int
os_hid_open(const char *path, struct os_hid_device **out_hid)
{
	struct os_hid_device *hrdev = calloc(1, sizeof(*hrdev));
	if (hrdev == NULL) {
		return -1;
	}

	hrdev->handle = hid_open_path(path);
	if (hrdev->handle == NULL) {
		free(hrdev);
		return -1;
	}

	*out_hid = hrdev;
	return 0;
}

void
os_hid_destroy(struct os_hid_device *hid_dev)
{
	if (hid_dev == NULL) {
		return;
	}

	hid_close(hid_dev->handle);
	free(hid_dev);
}

int
os_hid_read(struct os_hid_device *hid_dev, uint8_t *data, size_t size, int milliseconds)
{
	return hid_read_timeout(hid_dev->handle, data, size, milliseconds);
}

int
os_hid_write(struct os_hid_device *hid_dev, const uint8_t *data, size_t size)
{
	return hid_write(hid_dev->handle, data, size);
}

int
os_hid_get_feature(struct os_hid_device *hid_dev, uint8_t report_num, uint8_t *data, size_t size)
{
	data[0] = report_num;
	return hid_get_feature_report(hid_dev->handle, data, size);
}

int
os_hid_get_feature_timeout(struct os_hid_device *hid_dev, void *data, size_t size, uint32_t timeout)
{
	// No timeout to get a feature report in hidapi
	return hid_get_feature_report(hid_dev->handle, data, size);
}

int
os_hid_set_feature(struct os_hid_device *hid_dev, const uint8_t *data, size_t size)
{
	return hid_send_feature_report(hid_dev->handle, data, size);
}

#ifdef XRT_OS_LINUX
int
os_hid_open_hidraw(const char *path, struct os_hid_device **out_hid)
{
	return os_hid_open(path, out_hid);
}
#endif
