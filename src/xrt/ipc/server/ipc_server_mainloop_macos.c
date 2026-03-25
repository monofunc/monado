// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Mach IPC server mainloop for macOS.
 *
 * Registers a Mach service via bootstrap_check_in, accepts client
 * connections via port exchange, and starts per-client threads.
 *
 * Connection handshake:
 *   1. Client sends CONNECT to service_port with client's recv_port
 *   2. Server extracts client recv_port (= server's send_port to this client)
 *   3. Server allocates per-client recv_port
 *   4. Server sends CONNECT_ACK to client with server's recv_port + shmem name
 *   5. Both sides now have a dedicated bidirectional port pair
 *
 * @author Mono
 * @ingroup ipc_server
 */

#include "server/ipc_server.h"
#include "shared/ipc_message_channel.h"

#include "util/u_logging.h"

#include <mach/mach.h>
#include <servers/bootstrap.h>
#include <string.h>
#include <unistd.h>

#ifndef XRT_IPC_MACH_SERVICE_NAME
#define XRT_IPC_MACH_SERVICE_NAME "org.monado.compositor"
#endif

#define BROKER_MSG_REGISTER 100


/*
 *
 * Handshake message structs
 *
 */

struct ipc_mach_connect_msg
{
	mach_msg_header_t header;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t client_port;
};

struct ipc_mach_connect_ack_msg
{
	mach_msg_header_t header;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t server_port;
	char shmem_name[64];
};

struct ipc_mach_register_msg
{
	mach_msg_header_t header;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t accept_port;
};


/*
 *
 * Broker registration
 *
 */

static int
init_via_broker(struct ipc_server_mainloop *ml)
{
	kern_return_t kr;

	mach_port_t broker_port = MACH_PORT_NULL;
	kr = bootstrap_look_up(bootstrap_port, XRT_IPC_MACH_SERVICE_NAME, &broker_port);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("bootstrap_look_up for broker failed: %s", mach_error_string(kr));
		return -1;
	}

	mach_port_t accept_port = MACH_PORT_NULL;
	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &accept_port);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_allocate failed: %s", mach_error_string(kr));
		mach_port_deallocate(mach_task_self(), broker_port);
		return -1;
	}

	kr = mach_port_insert_right(mach_task_self(), accept_port, accept_port, MACH_MSG_TYPE_MAKE_SEND);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_insert_right failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), accept_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_deallocate(mach_task_self(), broker_port);
		return -1;
	}

	struct ipc_mach_register_msg reg = {0};
	reg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	reg.header.msgh_size = sizeof(reg);
	reg.header.msgh_remote_port = broker_port;
	reg.header.msgh_local_port = MACH_PORT_NULL;
	reg.header.msgh_id = BROKER_MSG_REGISTER;
	reg.body.msgh_descriptor_count = 1;
	reg.accept_port.name = accept_port;
	reg.accept_port.disposition = MACH_MSG_TYPE_COPY_SEND;
	reg.accept_port.type = MACH_MSG_PORT_DESCRIPTOR;

	kr = mach_msg(&reg.header, MACH_SEND_MSG, sizeof(reg), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE,
	              MACH_PORT_NULL);

	mach_port_deallocate(mach_task_self(), broker_port);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("Failed to send REGISTER to broker: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), accept_port, MACH_PORT_RIGHT_RECEIVE, -1);
		return -1;
	}

	ml->service_port = accept_port;
	U_LOG_I("Registered with Mach IPC broker (accept_port=%u)", accept_port);
	return 0;
}


/*
 *
 * Mainloop functions
 *
 */

int
ipc_server_mainloop_init(struct ipc_server_mainloop *ml, bool no_stdin)
{
	(void)no_stdin;

	// Try bootstrap_check_in first (standalone monado-service with own plist)
	mach_port_t service_port = MACH_PORT_NULL;
	kern_return_t kr = bootstrap_check_in(bootstrap_port, XRT_IPC_MACH_SERVICE_NAME, &service_port);

	if (kr == KERN_SUCCESS) {
		ml->service_port = service_port;
		U_LOG_I("Mach IPC initialized via bootstrap_check_in (port %u)", service_port);
		return 0;
	}

	// bootstrap_check_in failed, connect to broker instead
	U_LOG_I("bootstrap_check_in failed (%s), connecting to broker", mach_error_string(kr));
	return init_via_broker(ml);
}

void
ipc_server_mainloop_poll(struct ipc_server *vs, struct ipc_server_mainloop *ml)
{
	if (ml->service_port == MACH_PORT_NULL) {
		return;
	}

	// Wait for a CONNECT message from a client
	// Use oversized buffer, complex messages may include kernel trailers
	union {
		struct ipc_mach_connect_msg msg;
		uint8_t buf[256];
	} connect_buf = {0};
	struct ipc_mach_connect_msg *connect_msg = &connect_buf.msg;
	connect_msg->header.msgh_size = sizeof(connect_buf);
	connect_msg->header.msgh_local_port = ml->service_port;

	kern_return_t kr = mach_msg(&connect_msg->header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(connect_buf),
	                            ml->service_port, 500, MACH_PORT_NULL);

	if (kr == MACH_RCV_TIMED_OUT) {
		return;
	}

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive on service port failed: %s", mach_error_string(kr));
		return;
	}

	// Extract client's recv port
	mach_port_t client_send_port = connect_msg->client_port.name;
	if (client_send_port == MACH_PORT_NULL) {
		U_LOG_E("Client sent NULL port");
		return;
	}

	// Allocate a per-client receive port
	mach_port_t server_recv_port = MACH_PORT_NULL;
	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &server_recv_port);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_allocate failed: %s", mach_error_string(kr));
		mach_port_deallocate(mach_task_self(), client_send_port);
		return;
	}

	// Insert a send right so the client can send to us
	kr = mach_port_insert_right(mach_task_self(), server_recv_port, server_recv_port, MACH_MSG_TYPE_MAKE_SEND);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_insert_right failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), server_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_deallocate(mach_task_self(), client_send_port);
		return;
	}

	// Build shmem name
	char shmem_name[64];
	snprintf(shmem_name, sizeof(shmem_name), "/monado_shm_%d", getpid());

	// Send CONNECT_ACK to client with our recv port + shmem name
	struct ipc_mach_connect_ack_msg ack = {0};
	ack.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	ack.header.msgh_size = sizeof(ack);
	ack.header.msgh_remote_port = client_send_port;
	ack.header.msgh_local_port = MACH_PORT_NULL;
	ack.body.msgh_descriptor_count = 1;
	ack.server_port.name = server_recv_port;
	ack.server_port.disposition = MACH_MSG_TYPE_MOVE_SEND;
	ack.server_port.type = MACH_MSG_PORT_DESCRIPTOR;
	strncpy(ack.shmem_name, shmem_name, sizeof(ack.shmem_name) - 1);

	kr = mach_msg(&ack.header, MACH_SEND_MSG, sizeof(ack), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE,
	              MACH_PORT_NULL);

	if (kr != KERN_SUCCESS) {
		U_LOG_E("Failed to send CONNECT_ACK: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), server_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_deallocate(mach_task_self(), client_send_port);
		return;
	}

	U_LOG_I("New Mach IPC client connected (send=%u, recv=%u)", client_send_port, server_recv_port);

	// Start the per-client thread
	ipc_server_handle_client_connected_mach(vs, client_send_port, server_recv_port);
}

void
ipc_server_mainloop_deinit(struct ipc_server_mainloop *ml)
{
	if (ml->service_port != MACH_PORT_NULL) {
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->service_port = MACH_PORT_NULL;
	}
	U_LOG_I("Mach IPC mainloop deinitialized");
}
