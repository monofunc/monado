// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief IPC message channel using raw Mach IPC for macOS.
 *
 * Each ipc_send() is a mach_msg(MACH_SEND_MSG) and each ipc_receive()
 * is a mach_msg(MACH_RCV_MSG).
 *
 * @author Mono
 * @ingroup ipc_shared
 */

#include "shared/ipc_message_channel.h"

#include "util/u_logging.h"

#include <mach/mach.h>
#include <sys/fileport.h>
#include <unistd.h>

#ifndef XRT_OS_OSX
#error "This file requires macOS"
#endif


/*
 *
 * Message structs
 *
 */

// Simple message
struct ipc_mach_msg
{
	mach_msg_header_t header;
	uint32_t data_size;
	uint8_t data[2048];
};

// Complex message: inline data + Mach port descriptors.
// Must be >= XRT_MAX_IPC_HANDLES
#define IPC_MACH_MAX_PORTS 16

struct ipc_mach_msg_complex
{
	mach_msg_header_t header;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t ports[IPC_MACH_MAX_PORTS];
	uint32_t port_count;
	uint32_t data_size;
	uint8_t data[2048];
};


/*
 *
 * Core transport
 *
 */

void
ipc_message_channel_close(struct ipc_message_channel *imc)
{
	if (imc->send_port != MACH_PORT_NULL) {
		mach_port_deallocate(mach_task_self(), imc->send_port);
		imc->send_port = MACH_PORT_NULL;
	}
	if (imc->recv_port != MACH_PORT_NULL) {
		mach_port_mod_refs(mach_task_self(), imc->recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		imc->recv_port = MACH_PORT_NULL;
	}
}

xrt_result_t
ipc_send(struct ipc_message_channel *imc, const void *data, size_t size)
{
	if (size > sizeof(((struct ipc_mach_msg *)0)->data)) {
		U_LOG_E("ipc_send: data too large (%zu)", size);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct ipc_mach_msg msg = {0};
	msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0);
	// Mach messages must be 4-byte aligned
	mach_msg_size_t msg_size = offsetof(struct ipc_mach_msg, data) + size;
	msg_size = (msg_size + 3) & ~3;
	msg.header.msgh_size = msg_size;
	msg.header.msgh_remote_port = imc->send_port;
	msg.header.msgh_local_port = MACH_PORT_NULL;
	msg.data_size = (uint32_t)size;
	memcpy(msg.data, data, size);

	kern_return_t kr = mach_msg(&msg.header, MACH_SEND_MSG, msg.header.msgh_size, 0, MACH_PORT_NULL,
	                            MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg send failed: %s (size=%u, data_size=%u)", mach_error_string(kr), msg.header.msgh_size,
		        msg.data_size);
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_receive(struct ipc_message_channel *imc, void *out_data, size_t size)
{
	struct ipc_mach_msg msg = {0};
	msg.header.msgh_size = sizeof(msg);
	msg.header.msgh_local_port = imc->recv_port;

	kern_return_t kr =
	    mach_msg(&msg.header, MACH_RCV_MSG, 0, sizeof(msg), imc->recv_port, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive failed: %s", mach_error_string(kr));
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.data_size < size) {
		U_LOG_E("mach_msg received %u bytes but expected %zu", msg.data_size, size);
		return XRT_ERROR_IPC_FAILURE;
	}

	memcpy(out_data, msg.data, size);
	return XRT_SUCCESS;
}


/*
 *
 * FD transport (fileport API for shmem)
 *
 */

xrt_result_t
ipc_receive_fds(struct ipc_message_channel *imc, void *out_data, size_t size, int *out_fds, uint32_t fd_count)
{
	if (fd_count == 0) {
		return ipc_receive(imc, out_data, size);
	}

	// Receive complex message with fileport descriptors
	struct ipc_mach_msg_complex msg = {0};
	msg.header.msgh_size = sizeof(msg);
	msg.header.msgh_local_port = imc->recv_port;

	kern_return_t kr =
	    mach_msg(&msg.header, MACH_RCV_MSG, 0, sizeof(msg), imc->recv_port, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive (fds) failed: %s", mach_error_string(kr));
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.data_size < size) {
		U_LOG_E("fd msg data too short: %u < %zu", msg.data_size, size);
		for (uint32_t i = 0; i < msg.body.msgh_descriptor_count && i < IPC_MACH_MAX_PORTS; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	memcpy(out_data, msg.data, size);

	// Convert fileports back to fds
	for (uint32_t i = 0; i < fd_count; i++) {
		if (i < msg.body.msgh_descriptor_count) {
			out_fds[i] = fileport_makefd(msg.ports[i].name);
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
			if (out_fds[i] < 0) {
				U_LOG_E("fileport_makefd failed for port %u", i);
				// Clean up remaining ports
				for (uint32_t j = i + 1; j < msg.body.msgh_descriptor_count && j < IPC_MACH_MAX_PORTS;
				     j++) {
					mach_port_deallocate(mach_task_self(), msg.ports[j].name);
				}
				// Close already-opened fds
				for (uint32_t j = 0; j < i; j++) {
					close(out_fds[j]);
					out_fds[j] = -1;
				}
				return XRT_ERROR_IPC_FAILURE;
			}
		} else {
			out_fds[i] = -1;
		}
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_send_fds(struct ipc_message_channel *imc, const void *data, size_t size, const int *fds, uint32_t fd_count)
{
	if (fd_count == 0) {
		return ipc_send(imc, data, size);
	}

	if (fd_count > IPC_MACH_MAX_PORTS) {
		U_LOG_E("Too many fds: %u > %d", fd_count, IPC_MACH_MAX_PORTS);
		return XRT_ERROR_IPC_FAILURE;
	}
	if (size > sizeof(((struct ipc_mach_msg_complex *)0)->data)) {
		U_LOG_E("ipc_send_fds: data too large (%zu)", size);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct ipc_mach_msg_complex msg = {0};
	msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	msg.header.msgh_remote_port = imc->send_port;
	msg.header.msgh_local_port = MACH_PORT_NULL;
	msg.body.msgh_descriptor_count = fd_count;

	// Convert fds to fileports
	for (uint32_t i = 0; i < fd_count; i++) {
		fileport_t fp = MACH_PORT_NULL;
		int ret = fileport_makeport(fds[i], &fp);
		if (ret != 0) {
			U_LOG_E("fileport_makeport failed for fd %d", fds[i]);
			// Deallocate already-created fileports
			for (uint32_t j = 0; j < i; j++) {
				mach_port_deallocate(mach_task_self(), msg.ports[j].name);
			}
			return XRT_ERROR_IPC_FAILURE;
		}
		msg.ports[i].name = fp;
		msg.ports[i].disposition = MACH_MSG_TYPE_MOVE_SEND;
		msg.ports[i].type = MACH_MSG_PORT_DESCRIPTOR;
	}

	msg.port_count = fd_count;
	msg.data_size = (uint32_t)size;
	memcpy(msg.data, data, size);
	mach_msg_size_t fds_msg_size = offsetof(struct ipc_mach_msg_complex, data) + size;
	fds_msg_size = (fds_msg_size + 3) & ~3;
	msg.header.msgh_size = fds_msg_size;

	kern_return_t kr = mach_msg(&msg.header, MACH_SEND_MSG, msg.header.msgh_size, 0, MACH_PORT_NULL,
	                            MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg send (fds) failed: %s", mach_error_string(kr));
		// MOVE_SEND ports not consumed on failure, deallocate them.
		for (uint32_t i = 0; i < fd_count; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}


/*
 *
 * Shared memory handles
 *
 */

xrt_result_t
ipc_receive_handles_shmem(struct ipc_message_channel *imc,
                          void *out_data,
                          size_t size,
                          xrt_shmem_handle_t *out_handles,
                          uint32_t handle_count)
{
	return ipc_receive_fds(imc, out_data, size, out_handles, handle_count);
}

xrt_result_t
ipc_send_handles_shmem(struct ipc_message_channel *imc,
                       const void *data,
                       size_t size,
                       const xrt_shmem_handle_t *handles,
                       uint32_t handle_count)
{
	return ipc_send_fds(imc, data, size, handles, handle_count);
}


/*
 *
 * Graphics buffer handle transport (direct Mach port descriptors)
 *
 */

xrt_result_t
ipc_send_handles_graphics_buffer(struct ipc_message_channel *imc,
                                 const void *data,
                                 size_t size,
                                 const xrt_graphics_buffer_handle_t *handles,
                                 uint32_t handle_count)
{
	if (handle_count > IPC_MACH_MAX_PORTS) {
		U_LOG_E("Too many graphics buffer handles: %u > %d", handle_count, IPC_MACH_MAX_PORTS);
		return XRT_ERROR_IPC_FAILURE;
	}
	if (size > sizeof(((struct ipc_mach_msg_complex *)0)->data)) {
		U_LOG_E("ipc_send_handles_graphics_buffer: data too large (%zu)", size);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct ipc_mach_msg_complex msg = {0};

	// Complex message bit tells the kernel to process descriptors
	msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	msg.header.msgh_remote_port = imc->send_port;
	msg.header.msgh_local_port = MACH_PORT_NULL;
	msg.body.msgh_descriptor_count = handle_count;

	// xrt_graphics_buffer_handle_t is mach_port_t on macOS, transfer directly
	for (uint32_t i = 0; i < handle_count; i++) {
		msg.ports[i].name = handles[i];
		msg.ports[i].disposition = MACH_MSG_TYPE_COPY_SEND;
		msg.ports[i].type = MACH_MSG_PORT_DESCRIPTOR;
	}

	msg.port_count = handle_count;
	msg.data_size = (uint32_t)size;
	memcpy(msg.data, data, size);

	mach_msg_size_t cmsg_size = offsetof(struct ipc_mach_msg_complex, data) + size;
	// Mach messages must be 4-byte aligned
	cmsg_size = (cmsg_size + 3) & ~3;
	msg.header.msgh_size = cmsg_size;

	kern_return_t kr = mach_msg(&msg.header, MACH_SEND_MSG, msg.header.msgh_size, 0, MACH_PORT_NULL,
	                            MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg send (graphics buffer) failed: %s (size=%u)", mach_error_string(kr), cmsg_size);
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_receive_handles_graphics_buffer(struct ipc_message_channel *imc,
                                    void *out_data,
                                    size_t size,
                                    xrt_graphics_buffer_handle_t *out_handles,
                                    uint32_t handle_count)
{
	struct ipc_mach_msg_complex msg = {0};
	msg.header.msgh_size = sizeof(msg);
	msg.header.msgh_local_port = imc->recv_port;

	kern_return_t kr =
	    mach_msg(&msg.header, MACH_RCV_MSG, 0, sizeof(msg), imc->recv_port, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive (graphics buffer) failed: %s", mach_error_string(kr));
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.body.msgh_descriptor_count > handle_count) {
		U_LOG_E("Graphics buffer port count too large: got %u descriptors, max %u",
		        msg.body.msgh_descriptor_count, handle_count);
		for (uint32_t i = 0; i < msg.body.msgh_descriptor_count && i < IPC_MACH_MAX_PORTS; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.data_size < size) {
		U_LOG_E("Graphics buffer msg data too short: %u < %zu", msg.data_size, size);
		for (uint32_t i = 0; i < msg.body.msgh_descriptor_count && i < IPC_MACH_MAX_PORTS; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	memcpy(out_data, msg.data, size);

	// xrt_graphics_buffer_handle_t is mach_port_t on macOS, no conversion needed
	uint32_t actual_count = msg.body.msgh_descriptor_count;
	for (uint32_t i = 0; i < actual_count; i++) {
		out_handles[i] = msg.ports[i].name;
	}

	for (uint32_t i = actual_count; i < handle_count; i++) {
		out_handles[i] = MACH_PORT_NULL;
	}

	return XRT_SUCCESS;
}


/*
 *
 * Sync handle transport (direct Mach port descriptors)
 *
 */

xrt_result_t
ipc_send_handles_graphics_sync(struct ipc_message_channel *imc,
                               const void *data,
                               size_t size,
                               const xrt_graphics_sync_handle_t *handles,
                               uint32_t handle_count)
{
	if (handle_count == 0) {
		return ipc_send(imc, data, size);
	}

	if (handle_count > IPC_MACH_MAX_PORTS) {
		U_LOG_E("Too many sync handles: %u > %d", handle_count, IPC_MACH_MAX_PORTS);
		return XRT_ERROR_IPC_FAILURE;
	}
	if (size > sizeof(((struct ipc_mach_msg_complex *)0)->data)) {
		U_LOG_E("ipc_send_handles_graphics_sync: data too large (%zu)", size);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct ipc_mach_msg_complex msg = {0};

	msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	msg.header.msgh_remote_port = imc->send_port;
	msg.header.msgh_local_port = MACH_PORT_NULL;
	msg.body.msgh_descriptor_count = handle_count;

	for (uint32_t i = 0; i < handle_count; i++) {
		msg.ports[i].name = handles[i];
		msg.ports[i].disposition = MACH_MSG_TYPE_COPY_SEND;
		msg.ports[i].type = MACH_MSG_PORT_DESCRIPTOR;
	}

	msg.port_count = handle_count;
	msg.data_size = (uint32_t)size;
	memcpy(msg.data, data, size);

	mach_msg_size_t cmsg_size = offsetof(struct ipc_mach_msg_complex, data) + size;
	cmsg_size = (cmsg_size + 3) & ~3;
	msg.header.msgh_size = cmsg_size;

	kern_return_t kr = mach_msg(&msg.header, MACH_SEND_MSG, msg.header.msgh_size, 0, MACH_PORT_NULL,
	                            MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg send (sync) failed: %s (size=%u)", mach_error_string(kr), cmsg_size);
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_receive_handles_graphics_sync(struct ipc_message_channel *imc,
                                  void *out_data,
                                  size_t size,
                                  xrt_graphics_sync_handle_t *out_handles,
                                  uint32_t handle_count)
{
	if (handle_count == 0) {
		return ipc_receive(imc, out_data, size);
	}

	struct ipc_mach_msg_complex msg = {0};
	msg.header.msgh_size = sizeof(msg);
	msg.header.msgh_local_port = imc->recv_port;

	kern_return_t kr =
	    mach_msg(&msg.header, MACH_RCV_MSG, 0, sizeof(msg), imc->recv_port, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive (sync) failed: %s", mach_error_string(kr));
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.body.msgh_descriptor_count > handle_count) {
		U_LOG_E("Sync port count too large: got %u descriptors, max %u", msg.body.msgh_descriptor_count,
		        handle_count);
		for (uint32_t i = 0; i < msg.body.msgh_descriptor_count && i < IPC_MACH_MAX_PORTS; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	if (msg.data_size < size) {
		U_LOG_E("Sync msg data too short: %u < %zu", msg.data_size, size);
		for (uint32_t i = 0; i < msg.body.msgh_descriptor_count && i < IPC_MACH_MAX_PORTS; i++) {
			mach_port_deallocate(mach_task_self(), msg.ports[i].name);
		}
		return XRT_ERROR_IPC_FAILURE;
	}

	memcpy(out_data, msg.data, size);

	// Sync handles are Mach ports, no conversion needed
	uint32_t actual_count = msg.body.msgh_descriptor_count;
	for (uint32_t i = 0; i < actual_count; i++) {
		out_handles[i] = msg.ports[i].name;
	}

	for (uint32_t i = actual_count; i < handle_count; i++) {
		out_handles[i] = MACH_PORT_NULL;
	}

	return XRT_SUCCESS;
}
