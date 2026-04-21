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
#include <mach/notify.h>
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
 * Peer-death notification
 *
 * macOS does not signal mid-IPC peer exits. Detection comes from
 * MACH_NOTIFY_DEAD_NAME armed on each client's send port. The mainloop unions
 * notify_port and service_port into a port set so one mach_msg handles both
 * paths. On notification, the mainloop flags peer_dead on the matching client
 * and wakes its per-client thread with a sentinel message. That thread would
 * otherwise sit in mach_msg until its next 500ms timeout
 *
 */

struct ipc_mach_wakeup_msg
{
	mach_msg_header_t header;
};

// Wake a per-client thread blocked in mach_msg() on its own receive right.
// We hold the receive right, so we mint a send right with mach_port_insert_right
// and hand it off via MOVE_SEND. A send failure degrades to the 500ms timeout
// path, so no error log
static void
wake_per_client_thread(mach_port_t recv_port)
{
	kern_return_t kr = mach_port_insert_right(mach_task_self(), recv_port, recv_port, MACH_MSG_TYPE_MAKE_SEND);
	if (kr != KERN_SUCCESS) {
		return;
	}

	struct ipc_mach_wakeup_msg wake = {0};
	wake.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND, 0);
	wake.header.msgh_size = sizeof(wake);
	wake.header.msgh_remote_port = recv_port;
	wake.header.msgh_local_port = MACH_PORT_NULL;
	wake.header.msgh_id = IPC_MACH_SENTINEL_SHUTDOWN;

	kr = mach_msg(&wake.header, MACH_SEND_MSG | MACH_SEND_TIMEOUT, sizeof(wake), 0, MACH_PORT_NULL, 0,
	              MACH_PORT_NULL);
	if (kr != KERN_SUCCESS) {
		// MOVE_SEND is not consumed on failure, release the send right we just inserted
		mach_port_deallocate(mach_task_self(), recv_port);
	}
}

static void
handle_dead_name_notification(struct ipc_server *vs, mach_port_name_t dead)
{
	os_mutex_lock(&vs->global_state.lock);
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &vs->threads[i].ics;
		if (ics->server_thread_index < 0 || ics->imc.send_port != dead) {
			continue;
		}
		ics->peer_dead = true;
		// Wake under lock so common_shutdown cannot race with our send into recv_port
		wake_per_client_thread(ics->imc.recv_port);
		break;
	}
	os_mutex_unlock(&vs->global_state.lock);

	// The DEAD_NAME notification transferred the dead send right to us, release it
	mach_port_deallocate(mach_task_self(), dead);
}


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

	ml->service_port = MACH_PORT_NULL;
	ml->notify_port = MACH_PORT_NULL;
	ml->port_set = MACH_PORT_NULL;

	// Try bootstrap_check_in first (standalone monado-service with own plist)
	mach_port_t service_port = MACH_PORT_NULL;
	kern_return_t kr = bootstrap_check_in(bootstrap_port, XRT_IPC_MACH_SERVICE_NAME, &service_port);

	if (kr == KERN_SUCCESS) {
		ml->service_port = service_port;
		U_LOG_I("Mach IPC initialized via bootstrap_check_in (port %u)", service_port);
	} else {
		// bootstrap_check_in failed, connect to broker instead
		U_LOG_I("bootstrap_check_in failed (%s), connecting to broker", mach_error_string(kr));
		if (init_via_broker(ml) != 0) {
			return -1;
		}
	}

	// notify_port is where the kernel delivers MACH_NOTIFY_DEAD_NAME messages.
	// port_set unions it with service_port so one mach_msg polls both
	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_PORT_SET, &ml->port_set);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_allocate(PORT_SET) failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->service_port = MACH_PORT_NULL;
		return -1;
	}

	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &ml->notify_port);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_allocate(RECEIVE) for notify_port failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), ml->port_set, MACH_PORT_RIGHT_PORT_SET, -1);
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->port_set = MACH_PORT_NULL;
		ml->service_port = MACH_PORT_NULL;
		return -1;
	}

	kr = mach_port_insert_member(mach_task_self(), ml->service_port, ml->port_set);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_insert_member(service_port) failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), ml->notify_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_mod_refs(mach_task_self(), ml->port_set, MACH_PORT_RIGHT_PORT_SET, -1);
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->notify_port = MACH_PORT_NULL;
		ml->port_set = MACH_PORT_NULL;
		ml->service_port = MACH_PORT_NULL;
		return -1;
	}

	kr = mach_port_insert_member(mach_task_self(), ml->notify_port, ml->port_set);
	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_port_insert_member(notify_port) failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), ml->notify_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_mod_refs(mach_task_self(), ml->port_set, MACH_PORT_RIGHT_PORT_SET, -1);
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->notify_port = MACH_PORT_NULL;
		ml->port_set = MACH_PORT_NULL;
		ml->service_port = MACH_PORT_NULL;
		return -1;
	}

	return 0;
}

void
ipc_server_mainloop_poll(struct ipc_server *vs, struct ipc_server_mainloop *ml)
{
	if (ml->port_set == MACH_PORT_NULL) {
		return;
	}

	// Receive buffer sized for either a CONNECT request or a DEAD_NAME
	// notification, with headroom for kernel-attached trailers
	union {
		struct ipc_mach_connect_msg connect_msg;
		mach_dead_name_notification_t dead_name_msg;
		uint8_t buf[256];
	} recv_buf = {0};
	recv_buf.connect_msg.header.msgh_size = sizeof(recv_buf);
	recv_buf.connect_msg.header.msgh_local_port = ml->port_set;

	kern_return_t kr = mach_msg(&recv_buf.connect_msg.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(recv_buf),
	                            ml->port_set, 500, MACH_PORT_NULL);

	if (kr == MACH_RCV_TIMED_OUT) {
		return;
	}

	if (kr != KERN_SUCCESS) {
		U_LOG_E("mach_msg receive on port set failed: %s", mach_error_string(kr));
		return;
	}

	mach_msg_id_t msg_id = recv_buf.connect_msg.header.msgh_id;

	if (msg_id == MACH_NOTIFY_DEAD_NAME) {
		handle_dead_name_notification(vs, recv_buf.dead_name_msg.not_port);
		return;
	}

	// Anything else is a CONNECT request on service_port
	struct ipc_mach_connect_msg *connect_msg = &recv_buf.connect_msg;

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

	// Start the per-client thread. The handler arms peer-death notification
	// under global_state.lock once the slot is committed
	ipc_server_handle_client_connected_mach(vs, client_send_port, server_recv_port);
}

void
ipc_server_mainloop_deinit(struct ipc_server_mainloop *ml)
{
	if (ml->service_port != MACH_PORT_NULL) {
		mach_port_mod_refs(mach_task_self(), ml->service_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->service_port = MACH_PORT_NULL;
	}
	if (ml->notify_port != MACH_PORT_NULL) {
		mach_port_mod_refs(mach_task_self(), ml->notify_port, MACH_PORT_RIGHT_RECEIVE, -1);
		ml->notify_port = MACH_PORT_NULL;
	}
	if (ml->port_set != MACH_PORT_NULL) {
		mach_port_mod_refs(mach_task_self(), ml->port_set, MACH_PORT_RIGHT_PORT_SET, -1);
		ml->port_set = MACH_PORT_NULL;
	}
	U_LOG_I("Mach IPC mainloop deinitialized");
}
