# Copyright 2026, Monado Contributors
# SPDX-License-Identifier: BSL-1.0

set(XRT_IPC_MACH_SERVICE_NAME "org.monado.compositor" CACHE STRING "Mach service name for the IPC broker")

add_executable(monado-broker main.c)
target_compile_definitions(monado-broker PRIVATE "XRT_IPC_MACH_SERVICE_NAME=\"${XRT_IPC_MACH_SERVICE_NAME}\"")

configure_file(broker.plist.in "${CMAKE_CURRENT_BINARY_DIR}/${XRT_IPC_MACH_SERVICE_NAME}.plist.in" @ONLY)
file(GENERATE
	OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${XRT_IPC_MACH_SERVICE_NAME}.plist"
	INPUT "${CMAKE_CURRENT_BINARY_DIR}/${XRT_IPC_MACH_SERVICE_NAME}.plist.in"
	TARGET monado-broker
)
