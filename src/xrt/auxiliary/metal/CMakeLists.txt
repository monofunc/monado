# Copyright 2025-2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	aux_metal STATIC
	mtl_documentation.h
	mtl_format.h
	mtl_format.m
	mtl_image_collection.h
	mtl_image_collection.m
	)
target_link_libraries(aux_metal PUBLIC aux-includes aux_util aux_vk)
target_link_libraries(
	aux_metal PRIVATE "-framework Metal" "-framework IOSurface" "-framework Foundation"
			  "-framework CoreFoundation"
	)
