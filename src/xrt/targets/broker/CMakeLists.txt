# Copyright 2026, Monado Contributors
# SPDX-License-Identifier: BSL-1.0

add_executable(monado-broker main.c)

file(GENERATE
	OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/org.monado.compositor.plist"
	INPUT "${CMAKE_CURRENT_SOURCE_DIR}/org.monado.compositor.plist"
	TARGET monado-broker
)
