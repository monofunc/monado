// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief USB device helpers for RayNeo glasses.
 * @author ChatGPT 5.3
 * @ingroup drv_rayneo
 */

#pragma once

#include "xrt/xrt_compiler.h"

struct libusb_context;
struct libusb_device_handle;

/*!
 * RayNeo AR glasses USB device state.
 *
 * This struct stores the opened libusb objects and selected
 * interface/endpoints used by the RayNeo protocol after
 * @ref rayneo_usb_open succeeds.
 */
typedef struct rayneo_device
{
	//! libusb context created by @ref rayneo_usb_open.
	struct libusb_context *ctx;
	//! Opened libusb device handle for the glasses.
	struct libusb_device_handle *handle;
	//! Claimed interface number.
	int iface;
	//! IN endpoint address used for reads.
	uint8_t ep_in;
	//! OUT endpoint address used for writes.
	uint8_t ep_out;
	//! Transfer type for @ref ep_in (LIBUSB_TRANSFER_TYPE_*).
	uint8_t ep_in_type;
	//! Transfer type for @ref ep_out (LIBUSB_TRANSFER_TYPE_*).
	uint8_t ep_out_type;
	//! Max packet size for IN transfers.
	int in_mps;
	//! Max packet size for OUT transfers.
	int out_mps;
} rayneo_device_t;

/*!
 * Open the RayNeo USB device and select endpoints/interface.
 *
 * On success, @p dev is populated and ready for send/read operations.
 *
 * @param dev USB device state to initialize.
 * @param vid USB vendor ID to open.
 * @param pid USB product ID to open.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_usb_open(rayneo_device_t *dev, uint16_t vid, uint16_t pid);

/*!
 * Close the RayNeo USB device and reset @p dev to the empty state.
 *
 * Safe to call on a zero-initialized or already-closed device.
 *
 * @param dev USB device state to close.
 */
void
rayneo_usb_close(rayneo_device_t *dev);

/*!
 * Send one RayNeo protocol request frame.
 *
 * The payload is truncated to the protocol/endpoint payload limit when needed.
 *
 * @param dev Opened USB device state.
 * @param cmd Protocol command byte.
 * @param arg Protocol argument byte.
 * @param payload Optional payload bytes.
 * @param payload_len Number of payload bytes.
 *
 * @return 0 on success, negative or libusb error code on failure.
 */
int
rayneo_usb_send_request_with_payload(
    rayneo_device_t *dev, uint8_t cmd, uint8_t arg, const void *payload, size_t payload_len);

/*!
 * Read one USB packet from the RayNeo USB device.
 *
 * @param dev Opened USB device state.
 * @param buf Target buffer.
 * @param buf_len Size of @p buf in bytes.
 * @param timeout_ms Read timeout in milliseconds.
 *
 * @return Number of bytes read, 0 on timeout, negative on error.
 */
int
rayneo_usb_read_packet(rayneo_device_t *dev, uint8_t *buf, size_t buf_len, int timeout_ms);
