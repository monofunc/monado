# Copyright 2025-2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(aux_allocation STATIC a_allocator.c a_allocator.h)

target_link_libraries(aux_allocation PUBLIC aux_util aux_vk)
