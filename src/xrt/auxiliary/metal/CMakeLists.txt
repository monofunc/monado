# Copyright 2025-2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(aux_metal STATIC mtl_documentation.h mtl_format.h mtl_format.m)
target_link_libraries(aux_metal PUBLIC aux-includes aux_util)
target_link_libraries(aux_metal PRIVATE "-framework Metal")
