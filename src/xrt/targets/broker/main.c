// Copyright 2026, Monado Contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Mach service discovery broker for macOS.
 * @author Mono
 * @ingroup targets
 *
 * Registers as the Mach service and forwards client CONNECT messages
 * to the compositor server. Monitors server lifetime via dead-name
 * notifications.
 */

#include <mach/mach.h>
#include <servers/bootstrap.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef XRT_IPC_MACH_SERVICE_NAME
#define XRT_IPC_MACH_SERVICE_NAME "org.monado.compositor"
#endif

#define BROKER_MSG_REGISTER 100

struct connect_msg
{
	mach_msg_header_t header;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t port;
};

static mach_port_t service_port = MACH_PORT_NULL;
static mach_port_t server_accept_port = MACH_PORT_NULL;

static void
clear_server(void)
{
	if (server_accept_port != MACH_PORT_NULL) {
		mach_port_deallocate(mach_task_self(), server_accept_port);
		server_accept_port = MACH_PORT_NULL;
		fprintf(stderr, "monado-broker: server unregistered\n");
	}
}

static void
register_server(mach_port_t port)
{
	clear_server();
	server_accept_port = port;

	// Monitor server process lifetime.
	mach_port_t prev = MACH_PORT_NULL;
	kern_return_t kr = mach_port_request_notification(
	    mach_task_self(), server_accept_port, MACH_NOTIFY_DEAD_NAME, 0, service_port,
	    MACH_MSG_TYPE_MAKE_SEND_ONCE, &prev);
	if (kr != KERN_SUCCESS) {
		fprintf(stderr, "monado-broker: dead name notification failed: %s\n", mach_error_string(kr));
	}

	fprintf(stderr, "monado-broker: server registered (accept_port=%u)\n", port);
}

static void
forward_connect(struct connect_msg *client_msg)
{
	if (server_accept_port == MACH_PORT_NULL) {
		fprintf(stderr, "monado-broker: no server registered, dropping CONNECT\n");
		if (client_msg->body.msgh_descriptor_count > 0) {
			mach_port_deallocate(mach_task_self(), client_msg->port.name);
		}
		return;
	}

	struct connect_msg fwd = {0};
	fwd.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	fwd.header.msgh_size = sizeof(fwd);
	fwd.header.msgh_remote_port = server_accept_port;
	fwd.header.msgh_local_port = MACH_PORT_NULL;
	fwd.header.msgh_id = 0;
	fwd.body.msgh_descriptor_count = 1;
	fwd.port.name = client_msg->port.name;
	fwd.port.disposition = MACH_MSG_TYPE_MOVE_SEND;
	fwd.port.type = MACH_MSG_PORT_DESCRIPTOR;

	kern_return_t kr =
	    mach_msg(&fwd.header, MACH_SEND_MSG, sizeof(fwd), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
	if (kr != KERN_SUCCESS) {
		fprintf(stderr, "monado-broker: forward CONNECT failed: %s\n", mach_error_string(kr));
		// MOVE_SEND not consumed on failure, deallocate client port.
		mach_port_deallocate(mach_task_self(), client_msg->port.name);
		return;
	}

	fprintf(stderr, "monado-broker: forwarded CONNECT to server\n");
}

int
main(void)
{
	kern_return_t kr;

	kr = bootstrap_check_in(bootstrap_port, XRT_IPC_MACH_SERVICE_NAME, &service_port);
	if (kr != KERN_SUCCESS) {
		fprintf(stderr, "monado-broker: bootstrap_check_in failed: %s\n", mach_error_string(kr));
		return 1;
	}
	fprintf(stderr, "monado-broker: started on %s (port=%u)\n", XRT_IPC_MACH_SERVICE_NAME, service_port);

	while (true) {
		union {
			struct connect_msg msg;
			uint8_t buf[512];
		} recv_buf = {0};
		recv_buf.msg.header.msgh_size = sizeof(recv_buf);
		recv_buf.msg.header.msgh_local_port = service_port;

		kr = mach_msg(&recv_buf.msg.header, MACH_RCV_MSG, 0, sizeof(recv_buf), service_port,
		              MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
		if (kr != KERN_SUCCESS) {
			fprintf(stderr, "monado-broker: mach_msg receive failed: %s\n", mach_error_string(kr));
			continue;
		}

		mach_msg_id_t msg_id = recv_buf.msg.header.msgh_id;

		if (msg_id == MACH_NOTIFY_DEAD_NAME) {
			// Kernel adds an extra user-ref on the dead name, deallocate it.
			mach_dead_name_notification_t *notif = (mach_dead_name_notification_t *)&recv_buf;
			mach_port_deallocate(mach_task_self(), notif->not_port);
			fprintf(stderr, "monado-broker: server process died\n");
			clear_server();
			continue;
		}

		if (msg_id == BROKER_MSG_REGISTER) {
			if (recv_buf.msg.body.msgh_descriptor_count > 0) {
				register_server(recv_buf.msg.port.name);
			}
			continue;
		}

		if (recv_buf.msg.body.msgh_descriptor_count > 0) {
			forward_connect(&recv_buf.msg);
		}
	}

	return 0;
}
