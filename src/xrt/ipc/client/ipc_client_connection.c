// Copyright 2020-2024, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Just the client connection setup/teardown bits.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup ipc_client
 */

#include "os/os_threading.h"
#include "xrt/xrt_results.h"
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "xrt/xrt_instance.h"
#include "xrt/xrt_handles.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_config_android.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_file.h"
#include "util/u_debug.h"
#include "util/u_git_tag.h"
#include "util/u_truncate_printf.h"

#include "shared/ipc_utils.h"
#include "shared/ipc_protocol.h"
#include "client/ipc_client_connection.h"

#include "ipc_client_generated.h"


#include <stdio.h>
#if !defined(XRT_OS_WINDOWS)
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#include <limits.h>

#ifdef XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER
#include "android/android_ahardwarebuffer_allocator.h"
#endif

#ifdef XRT_OS_ANDROID
#include "android/ipc_client_android.h"
#endif // XRT_OS_ANDROID

DEBUG_GET_ONCE_BOOL_OPTION(ipc_ignore_version, "IPC_IGNORE_VERSION", false)

#ifdef XRT_OS_ANDROID

static bool
ipc_client_socket_connect(struct ipc_connection *ipc_c, struct _JavaVM *vm, void *context)
{
	ipc_c->ica = ipc_client_android_create(vm, context);

	if (ipc_c->ica == NULL) {
		IPC_ERROR(ipc_c, "Client create error!");
		return false;
	}

	int socket = ipc_client_android_blocking_connect(ipc_c->ica);
	if (socket < 0) {
		IPC_ERROR(ipc_c, "Service Connect error!");
		return false;
	}
	// The ownership belongs to the Java object. Dup because the fd will be
	// closed when client destroy.
	socket = dup(socket);
	if (socket < 0) {
		IPC_ERROR(ipc_c, "Failed to dup fd with error %d!", errno);
		return false;
	}

	// Set socket.
	ipc_c->imc.ipc_handle = socket;

	return true;
}

#elif defined(XRT_OS_WINDOWS)

#if defined(NO_XRT_SERVICE_LAUNCH) || !defined(XRT_SERVICE_EXECUTABLE)
static HANDLE
ipc_connect_pipe(struct ipc_connection *ipc_c, const char *pipe_name)
{
	HANDLE pipe_inst = CreateFileA(pipe_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (pipe_inst == INVALID_HANDLE_VALUE) {
		DWORD err = GetLastError();
		IPC_ERROR(ipc_c, "Connect to %s failed: %lu %s", pipe_name, err, ipc_winerror(err));
	}
	return pipe_inst;
}
#else
// N.B. quality of life fallback to try launch the XRT_SERVICE_EXECUTABLE if pipe is not found
static HANDLE
ipc_connect_pipe(struct ipc_connection *ipc_c, const char *pipe_name)
{
	HANDLE pipe_inst = CreateFileA(pipe_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (pipe_inst != INVALID_HANDLE_VALUE) {
		return pipe_inst;
	}
	DWORD err = GetLastError();
	IPC_ERROR(ipc_c, "Connect to %s failed: %lu %s", pipe_name, err, ipc_winerror(err));
	if (err != ERROR_FILE_NOT_FOUND) {
		return INVALID_HANDLE_VALUE;
	}
	IPC_INFO(ipc_c, "Trying to launch " XRT_SERVICE_EXECUTABLE "...");
	HMODULE hmod;
	if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	                        (LPCSTR)ipc_connect_pipe, &hmod)) {
		IPC_ERROR(ipc_c, "GetModuleHandleExA failed: %lu %s", err, ipc_winerror(err));
		return INVALID_HANDLE_VALUE;
	}
	char current_path[MAX_PATH] = {0};
	if (!GetModuleFileNameA(hmod, current_path, sizeof(current_path))) {
		IPC_ERROR(ipc_c, "GetModuleFileNameA failed: %lu %s", err, ipc_winerror(err));
		return INVALID_HANDLE_VALUE;
	}
	char *p = strrchr(current_path, '\\');
	if (!p) {
		IPC_ERROR(ipc_c, "failed to parse the path %s", current_path);
		return INVALID_HANDLE_VALUE;
	}

	char service_path[MAX_PATH] = {0};
	snprintf(service_path, MAX_PATH, "%s\\%s", current_path, XRT_SERVICE_EXECUTABLE);
	STARTUPINFOA si = {.cb = sizeof(si)};
	PROCESS_INFORMATION pi;
	if (!CreateProcessA(NULL, service_path, NULL, NULL, false, 0, NULL, NULL, &si, &pi)) {
		*p = 0;
		p = strrchr(service_path, '\\');
		if (!p) {
			err = GetLastError();
			IPC_INFO(ipc_c, XRT_SERVICE_EXECUTABLE " not found in %s: %lu %s", service_path, err,
			         ipc_winerror(err));
			return INVALID_HANDLE_VALUE;
		}

		snprintf(service_path, MAX_PATH, "%s\\service\\%s", current_path, XRT_SERVICE_EXECUTABLE);
		if (!CreateProcessA(NULL, service_path, NULL, NULL, false, 0, NULL, NULL, &si, &pi)) {
			err = GetLastError();
			IPC_INFO(ipc_c, XRT_SERVICE_EXECUTABLE " not found at %s: %lu %s", service_path, err,
			         ipc_winerror(err));
			return INVALID_HANDLE_VALUE;
		}
	}
	IPC_INFO(ipc_c, "Launched %s (pid %d)... Waiting for %s...", service_path, pi.dwProcessId, pipe_name);
	CloseHandle(pi.hThread);
	for (int i = 0;; i++) {
		pipe_inst = CreateFileA(pipe_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
		if (pipe_inst != INVALID_HANDLE_VALUE) {
			IPC_INFO(ipc_c, "Connected to %s after %d msec on try %d!", pipe_name, i * 100, i + 1);
			break;
		}
		err = GetLastError();
		if (err != ERROR_FILE_NOT_FOUND || WaitForSingleObject(pi.hProcess, 100) != WAIT_TIMEOUT) {
			IPC_ERROR(ipc_c, "Connect to %s failed: %lu %s", pipe_name, err, ipc_winerror(err));
			break;
		}
	}
	CloseHandle(pi.hProcess);
	return pipe_inst;
}
#endif

static bool
ipc_client_socket_connect(struct ipc_connection *ipc_c)
{
	const char pipe_prefix[] = "\\\\.\\pipe\\";
#define prefix_len sizeof(pipe_prefix) - 1
	char pipe_name[MAX_PATH + prefix_len];
	snprintf(pipe_name, sizeof(pipe_name), "%s", pipe_prefix);

	if (u_file_get_path_in_runtime_dir(XRT_IPC_MSG_SOCK_FILENAME, pipe_name + prefix_len, MAX_PATH) == -1) {
		U_LOG_E("u_file_get_path_in_runtime_dir failed!");
		return false;
	}

	HANDLE pipe_inst = ipc_connect_pipe(ipc_c, pipe_name);
	if (pipe_inst == INVALID_HANDLE_VALUE) {
		return false;
	}
	DWORD mode = PIPE_READMODE_MESSAGE | PIPE_WAIT;
	if (!SetNamedPipeHandleState(pipe_inst, &mode, NULL, NULL)) {
		DWORD err = GetLastError();
		IPC_ERROR(ipc_c, "SetNamedPipeHandleState(PIPE_READMODE_MESSAGE | PIPE_WAIT) failed: %lu %s", err,
		          ipc_winerror(err));
		return false;
	}

	// Set socket.
	ipc_c->imc.ipc_handle = pipe_inst;

	return true;
}

#elif defined(XRT_OS_OSX)

#include <mach/mach.h>
#include <servers/bootstrap.h>

#ifndef XRT_IPC_MACH_SERVICE_NAME
#define XRT_IPC_MACH_SERVICE_NAME "org.monado.compositor"
#endif

// Handshake message structs
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

static bool
ipc_client_socket_connect(struct ipc_connection *ipc_c)
{
	kern_return_t kr;

	// Look up the server's service port
	mach_port_t service_port = MACH_PORT_NULL;
	kr = bootstrap_look_up(bootstrap_port, XRT_IPC_MACH_SERVICE_NAME, &service_port);
	if (kr != KERN_SUCCESS) {
		IPC_ERROR(ipc_c, "bootstrap_look_up failed: %s", mach_error_string(kr));
		return false;
	}

	// Allocate our receive port
	mach_port_t client_recv_port = MACH_PORT_NULL;
	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &client_recv_port);
	if (kr != KERN_SUCCESS) {
		IPC_ERROR(ipc_c, "mach_port_allocate failed: %s", mach_error_string(kr));
		mach_port_deallocate(mach_task_self(), service_port);
		return false;
	}

	// Create a send right so the server can reply to us
	kr = mach_port_insert_right(mach_task_self(), client_recv_port, client_recv_port, MACH_MSG_TYPE_MAKE_SEND);
	if (kr != KERN_SUCCESS) {
		IPC_ERROR(ipc_c, "mach_port_insert_right failed: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), client_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_deallocate(mach_task_self(), service_port);
		return false;
	}

	// Send CONNECT to service port with our recv port
	struct ipc_mach_connect_msg connect_msg = {0};
	connect_msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	connect_msg.header.msgh_size = sizeof(connect_msg);
	connect_msg.header.msgh_remote_port = service_port;
	connect_msg.header.msgh_local_port = MACH_PORT_NULL;
	connect_msg.body.msgh_descriptor_count = 1;
	connect_msg.client_port.name = client_recv_port;
	connect_msg.client_port.disposition = MACH_MSG_TYPE_COPY_SEND;
	connect_msg.client_port.type = MACH_MSG_PORT_DESCRIPTOR;

	kr = mach_msg(&connect_msg.header, MACH_SEND_MSG, sizeof(connect_msg), 0, MACH_PORT_NULL,
	              MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
	if (kr != KERN_SUCCESS) {
		IPC_ERROR(ipc_c, "Failed to send CONNECT: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), client_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		mach_port_deallocate(mach_task_self(), service_port);
		return false;
	}

	// Done with service port
	mach_port_deallocate(mach_task_self(), service_port);

	// Receive CONNECT_ACK with server's port + shmem name (5s timeout)
	// Use oversized buffer, complex messages may include kernel trailers
	union {
		struct ipc_mach_connect_ack_msg msg;
		uint8_t buf[512];
	} ack_buf = {0};
	struct ipc_mach_connect_ack_msg *ack = &ack_buf.msg;
	ack->header.msgh_size = sizeof(ack_buf);
	ack->header.msgh_local_port = client_recv_port;

	kr = mach_msg(&ack->header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(ack_buf), client_recv_port, 5000,
	              MACH_PORT_NULL);
	if (kr != KERN_SUCCESS) {
		IPC_ERROR(ipc_c, "Failed to receive CONNECT_ACK: %s", mach_error_string(kr));
		mach_port_mod_refs(mach_task_self(), client_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		return false;
	}

	// Extract server's per-client recv port (we got a send right)
	mach_port_t server_send_port = ack->server_port.name;
	if (server_send_port == MACH_PORT_NULL) {
		IPC_ERROR(ipc_c, "Server sent NULL port in ACK");
		mach_port_mod_refs(mach_task_self(), client_recv_port, MACH_PORT_RIGHT_RECEIVE, -1);
		return false;
	}

	// Set up the message channel
	ipc_c->imc.send_port = server_send_port;
	ipc_c->imc.recv_port = client_recv_port;

	IPC_INFO(ipc_c, "Connected via Mach IPC (send=%u, recv=%u, shmem=%s)", server_send_port, client_recv_port,
	         ack->shmem_name);

	return true;
}

#elif defined(XRT_OS_UNIX)

static bool
ipc_client_socket_connect(struct ipc_connection *ipc_c)
{
#ifdef SOCK_CLOEXEC
	// Make sure the socket is not inherited by child processes. For one, when there is an fd to the socket
	// in the child process closing the client connection (or killing the connected process)
	// may not be seen in the server as client disconnection.
	const int flags = SOCK_CLOEXEC;
#else
	const int flags = 0;
#endif
	struct sockaddr_un addr = XRT_STRUCT_INIT;
	int ret;


	// create our IPC socket

	ret = socket(PF_UNIX, SOCK_STREAM | flags, 0);
	if (ret < 0) {
		IPC_ERROR(ipc_c, "Socket Create Error!");
		return false;
	}

	int socket = ret;

	char sock_file[PATH_MAX];

	ssize_t size = u_file_get_path_in_runtime_dir(XRT_IPC_MSG_SOCK_FILENAME, sock_file, PATH_MAX);
	if (size == -1) {
		IPC_ERROR(ipc_c, "Could not get socket file name");
		return false;
	}

	// Make sure the path fits.
	const int dst_size = (int)ARRAY_SIZE(addr.sun_path);
	if (size >= dst_size) {
		IPC_ERROR(ipc_c, "Total IPC path too long (%i > %i)", (int)size, dst_size);
		return false;
	}

	// Struct zero init at declaration.
	addr.sun_family = AF_UNIX;
	// Use truncate here to avoid warnings.
	u_truncate_snprintf(addr.sun_path, dst_size, "%s", sock_file);

	ret = connect(socket, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		IPC_ERROR(ipc_c, "Failed to connect to socket %s: %s!", sock_file, strerror(errno));
		close(socket);
		return false;
	}

	// Set socket.
	ipc_c->imc.ipc_handle = socket;

	return true;
}

#endif


static xrt_result_t
ipc_client_setup_shm(struct ipc_connection *ipc_c)
{
	/*
	 * Get our shared memory area from the server.
	 */

	xrt_result_t xret = ipc_call_instance_get_shm_fd(ipc_c, &ipc_c->ism_handle, 1);
	if (xret != XRT_SUCCESS) {
		IPC_ERROR(ipc_c, "Failed to retrieve shm fd!");
		return xret;
	}

	/*
	 * Now map it.
	 */

	const size_t size = sizeof(struct ipc_shared_memory);

#ifdef XRT_OS_WINDOWS
	DWORD access = FILE_MAP_READ | FILE_MAP_WRITE;

	ipc_c->ism = MapViewOfFile(ipc_c->ism_handle, access, 0, 0, size);
#else
	const int flags = MAP_SHARED;
	const int access = PROT_READ | PROT_WRITE;

	ipc_c->ism = mmap(NULL, size, access, flags, ipc_c->ism_handle, 0);
#endif

	if (ipc_c->ism == NULL) {
		IPC_ERROR(ipc_c, "Failed to mmap shm!");
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
ipc_client_check_git_tag(struct ipc_connection *ipc_c)
{
	// Does the git tags match?
	if (strncmp(u_git_tag, ipc_c->ism->u_git_tag, IPC_VERSION_NAME_LEN) == 0) {
		return XRT_SUCCESS;
	}

	IPC_ERROR(ipc_c, "Monado client library version %s does not match service version %s", u_git_tag,
	          ipc_c->ism->u_git_tag);

	if (!debug_get_bool_option_ipc_ignore_version()) {
		IPC_ERROR(ipc_c, "Set IPC_IGNORE_VERSION=1 to ignore this version conflict");
		return XRT_ERROR_IPC_FAILURE;
	}

	// Error is ignored.
	return XRT_SUCCESS;
}

static xrt_result_t
ipc_client_describe_client(struct ipc_connection *ipc_c, const struct xrt_application_info *a_info)
{
#ifdef XRT_OS_WINDOWS
	DWORD pid = GetCurrentProcessId();
#else
	pid_t pid = getpid();
#endif

	struct ipc_client_description desc = {0};
	desc.info = *a_info;
	desc.pid = pid; // Extra info.

	xrt_result_t xret = ipc_call_instance_describe_client(ipc_c, &desc);
	if (xret != XRT_SUCCESS) {
		IPC_ERROR(ipc_c, "Failed to set instance description!");
		return xret;
	}

	return XRT_SUCCESS;
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
ipc_client_connection_init(struct ipc_connection *ipc_c,
                           enum u_logging_level log_level,
                           const struct xrt_instance_info *i_info)
{
	xrt_result_t xret;

	U_ZERO(ipc_c);
#if !defined(XRT_OS_OSX)
	ipc_c->imc.ipc_handle = XRT_IPC_HANDLE_INVALID;
#endif
	ipc_c->imc.log_level = log_level;
	ipc_c->ism_handle = XRT_SHMEM_HANDLE_INVALID;

	// Must be done first.
	int ret = os_mutex_init(&ipc_c->mutex);
	if (ret != 0) {
		U_LOG_E("Failed to init mutex!");
		return XRT_ERROR_IPC_FAILURE;
	}

	// Connect the service.
#ifdef XRT_OS_ANDROID
	struct _JavaVM *vm = i_info->platform_info.vm;
	void *context = i_info->platform_info.context;

	if (!ipc_client_socket_connect(ipc_c, vm, context)) {
#else
	if (!ipc_client_socket_connect(ipc_c)) {
#endif
		IPC_ERROR(ipc_c,
		          "Failed to connect to monado service process\n\n"
		          "###\n"
		          "#\n"
		          "# Please make sure that the service process is running\n"
		          "#\n"
		          "# It is called \"monado-service\"\n"
		          "# In build trees, it is located "
		          "\"build-dir/src/xrt/targets/service/monado-service\"\n"
		          "#\n"
		          "###");
		os_mutex_destroy(&ipc_c->mutex);
		return XRT_ERROR_IPC_FAILURE;
	}

	// Do this first so we can use it to check git tags.
	xret = ipc_client_setup_shm(ipc_c);
	if (xret != XRT_SUCCESS) {
		goto err_fini; // Already logged.
	}

	// Requires shm.
	xret = ipc_client_check_git_tag(ipc_c);
	if (xret != XRT_SUCCESS) {
		goto err_fini; // Already logged.
	}

	// Do this last.
	xret = ipc_client_describe_client(ipc_c, &i_info->app_info);
	if (xret != XRT_SUCCESS) {
		goto err_fini; // Already logged.
	}

	return XRT_SUCCESS;

err_fini:
	ipc_client_connection_fini(ipc_c);

	return xret;
}

void
ipc_client_connection_fini(struct ipc_connection *ipc_c)
{
	if (ipc_c->ism_handle != XRT_SHMEM_HANDLE_INVALID) {
		/// @todo how to tear down the shared memory?
	}
	ipc_message_channel_close(&ipc_c->imc);
	os_mutex_destroy(&ipc_c->mutex);

#ifdef XRT_OS_ANDROID
	ipc_client_android_destroy(&(ipc_c->ica));
#endif
}
