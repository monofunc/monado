// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief USB transport helpers for RayNeo glasses.
 * @author ChatGPT 5.3
 * @ingroup drv_rayneo
 */

#include "rayneo_usb.h"

#include <libusb-1.0/libusb.h>

#include <string.h>

#define RAYNEO_USB_FRAME_MAGIC 0x66
#define RAYNEO_USB_FRAME_HEADER_SIZE 3
#define RAYNEO_USB_FRAME_MAX_PAYLOAD_SIZE 52
#define RAYNEO_USB_ERR_SHORT_WRITE -1001

typedef struct rayneo_usb_io_selection
{
	int iface;
	uint8_t ep_in;
	uint8_t ep_out;
	uint8_t ep_in_type;
	uint8_t ep_out_type;
	int in_mps;
	int out_mps;
} rayneo_usb_io_selection_t;

static bool
rayneo_usb_choose_altsetting(const struct libusb_interface_descriptor *alt, rayneo_usb_io_selection_t *out)
{
	rayneo_usb_io_selection_t candidate = {
	    .iface = alt->bInterfaceNumber,
	    .in_mps = 64,
	    .out_mps = 64,
	};

	for (uint8_t i = 0; i < alt->bNumEndpoints; i++) {
		const struct libusb_endpoint_descriptor *ep = &alt->endpoint[i];
		uint8_t type = ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;

		// Only accept interrupt or bulk endpoints.
		if (type != LIBUSB_TRANSFER_TYPE_INTERRUPT && type != LIBUSB_TRANSFER_TYPE_BULK) {
			continue;
		}

		// Mask off high-speed multiplier bits (11-12).
		int mps = ep->wMaxPacketSize & 0x7FF;

		// Take the first matching IN/OUT endpoint, not last.
		if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) != 0) {
			if (candidate.ep_in == 0) {
				candidate.ep_in = ep->bEndpointAddress;
				candidate.ep_in_type = type;
				if (mps > 0) {
					candidate.in_mps = mps;
				}
			}
		} else {
			if (candidate.ep_out == 0) {
				candidate.ep_out = ep->bEndpointAddress;
				candidate.ep_out_type = type;
				if (mps > 0) {
					candidate.out_mps = mps;
				}
			}
		}
	}

	if (candidate.ep_in == 0 || candidate.ep_out == 0) {
		return false;
	}

	*out = candidate;
	return true;
}

static bool
rayneo_usb_is_interrupt_pair(const rayneo_usb_io_selection_t *sel)
{
	return sel->ep_in_type == LIBUSB_TRANSFER_TYPE_INTERRUPT && sel->ep_out_type == LIBUSB_TRANSFER_TYPE_INTERRUPT;
}

// Only scan the active configuration to avoid selecting endpoints from an inactive one.
static int
rayneo_usb_find_io_endpoints(libusb_device_handle *handle, rayneo_usb_io_selection_t *out)
{
	libusb_device *dev = libusb_get_device(handle);
	struct libusb_config_descriptor *cfg = NULL;
	rayneo_usb_io_selection_t fallback = {0};
	bool have_fallback = false;

	if (dev == NULL || libusb_get_active_config_descriptor(dev, &cfg) != 0 || cfg == NULL) {
		return -1;
	}

	// Only check altsetting 0 (the default active one after claim).
	for (uint8_t ii = 0; ii < cfg->bNumInterfaces; ii++) {
		const struct libusb_interface *iface = &cfg->interface[ii];
		if (iface->num_altsetting == 0) {
			continue;
		}

		rayneo_usb_io_selection_t sel = {0};
		if (!rayneo_usb_choose_altsetting(&iface->altsetting[0], &sel)) {
			continue;
		}

		if (rayneo_usb_is_interrupt_pair(&sel)) {
			*out = sel;
			libusb_free_config_descriptor(cfg);
			return 0;
		}

		if (!have_fallback) {
			fallback = sel;
			have_fallback = true;
		}
	}

	libusb_free_config_descriptor(cfg);

	if (have_fallback) {
		*out = fallback;
		return 0;
	}

	return -1;
}

int
rayneo_usb_open(rayneo_device_t *dev, uint16_t vid, uint16_t pid)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *handle = NULL;
	rayneo_usb_io_selection_t io = {0};
	int rc;

	if (dev == NULL) {
		return -1;
	}

	memset(dev, 0, sizeof(*dev));

	rc = libusb_init(&ctx);
	if (rc != 0) {
		return rc;
	}

	handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
	if (handle == NULL) {
		libusb_exit(ctx);
		return -1;
	}

	if (rayneo_usb_find_io_endpoints(handle, &io) != 0) {
		libusb_close(handle);
		libusb_exit(ctx);
		return -1;
	}

	libusb_set_auto_detach_kernel_driver(handle, 1);
	rc = libusb_claim_interface(handle, io.iface);
	if (rc != 0) {
		libusb_close(handle);
		libusb_exit(ctx);
		return rc;
	}

	dev->ctx = ctx;
	dev->handle = handle;
	dev->iface = io.iface;
	dev->ep_in = io.ep_in;
	dev->ep_out = io.ep_out;
	dev->ep_in_type = io.ep_in_type;
	dev->ep_out_type = io.ep_out_type;
	dev->in_mps = io.in_mps;
	dev->out_mps = io.out_mps;

	return 0;
}

void
rayneo_usb_close(rayneo_device_t *dev)
{
	if (dev == NULL) {
		return;
	}

	if (dev->handle != NULL) {
		libusb_release_interface(dev->handle, dev->iface);
		libusb_close(dev->handle);
	}
	if (dev->ctx != NULL) {
		libusb_exit(dev->ctx);
	}

	memset(dev, 0, sizeof(*dev));
}

int
rayneo_usb_send_request_with_payload(
    rayneo_device_t *dev, uint8_t cmd, uint8_t arg, const void *payload, size_t payload_len)
{
	uint8_t packet[1024];
	int transferred = 0;
	int rc;

	if (dev == NULL || dev->handle == NULL || dev->ep_out == 0 || dev->out_mps <= 0 ||
	    dev->out_mps > (int)sizeof(packet)) {
		return -1;
	}

	memset(packet, 0, (size_t)dev->out_mps);
	packet[0] = RAYNEO_USB_FRAME_MAGIC;
	packet[1] = cmd;
	packet[2] = arg;
	if (payload != NULL && payload_len > 0 && dev->out_mps > RAYNEO_USB_FRAME_HEADER_SIZE) {
		size_t max_payload = (size_t)dev->out_mps - RAYNEO_USB_FRAME_HEADER_SIZE;
		if (max_payload > RAYNEO_USB_FRAME_MAX_PAYLOAD_SIZE) {
			max_payload = RAYNEO_USB_FRAME_MAX_PAYLOAD_SIZE;
		}
		size_t to_copy = payload_len < max_payload ? payload_len : max_payload;
		memcpy(packet + RAYNEO_USB_FRAME_HEADER_SIZE, payload, to_copy);
	}

	if (dev->ep_out_type == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
		rc = libusb_interrupt_transfer(dev->handle, dev->ep_out, packet, dev->out_mps, &transferred, 1000);
	} else {
		rc = libusb_bulk_transfer(dev->handle, dev->ep_out, packet, dev->out_mps, &transferred, 1000);
	}

	if (rc != 0) {
		return rc;
	}
	if (transferred != dev->out_mps) {
		return RAYNEO_USB_ERR_SHORT_WRITE;
	}

	return 0;
}

int
rayneo_usb_read_packet(rayneo_device_t *dev, uint8_t *buf, size_t buf_len, int timeout_ms)
{
	int transferred = 0;
	int rc;

	if (dev == NULL || dev->handle == NULL || buf == NULL || buf_len == 0) {
		return -1;
	}

	if (dev->ep_in_type == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
		rc = libusb_interrupt_transfer(dev->handle, dev->ep_in, buf, (int)buf_len, &transferred, timeout_ms);
	} else {
		rc = libusb_bulk_transfer(dev->handle, dev->ep_in, buf, (int)buf_len, &transferred, timeout_ms);
	}

	if (rc == LIBUSB_ERROR_TIMEOUT) {
		return 0;
	}
	if (rc != 0) {
		return -1;
	}

	return transferred;
}
