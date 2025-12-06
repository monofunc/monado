// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal format conversion functions.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup aux_metal
 */

#import <Metal/Metal.h>

#include "util/u_logging.h"

#include "mtl_format.h"


// clang-format off
static const int64_t VK_FORMAT_R5G6B5_UNORM_PACK16        = 4;
static const int64_t VK_FORMAT_R8G8B8_UNORM               = 23;
static const int64_t VK_FORMAT_R8G8B8_SRGB                = 29;
static const int64_t VK_FORMAT_B8G8R8_UNORM               = 30;
static const int64_t VK_FORMAT_R8G8B8A8_UNORM             = 37;
static const int64_t VK_FORMAT_R8G8B8A8_SRGB              = 43;
static const int64_t VK_FORMAT_B8G8R8A8_UNORM             = 44;
static const int64_t VK_FORMAT_B8G8R8A8_SRGB              = 50;
static const int64_t VK_FORMAT_A2B10G10R10_UNORM_PACK32   = 64;
static const int64_t VK_FORMAT_R16G16B16_UNORM            = 84;
static const int64_t VK_FORMAT_R16G16B16_SFLOAT           = 90;
static const int64_t VK_FORMAT_R16G16B16A16_UNORM         = 91;
static const int64_t VK_FORMAT_R16G16B16A16_SFLOAT        = 97;
static const int64_t VK_FORMAT_R32_SFLOAT                 = 100;
static const int64_t VK_FORMAT_D16_UNORM                  = 124;
static const int64_t VK_FORMAT_X8_D24_UNORM_PACK32        = 125;
static const int64_t VK_FORMAT_D32_SFLOAT                 = 126;
static const int64_t VK_FORMAT_S8_UINT                    = 127;
static const int64_t VK_FORMAT_D24_UNORM_S8_UINT          = 129;
static const int64_t VK_FORMAT_D32_SFLOAT_S8_UINT         = 130;
// clang-format on


int64_t
mtl_format_to_vk(int64_t format)
{
	switch (format) {
	case MTLPixelFormatBGRA8Unorm: return VK_FORMAT_B8G8R8A8_UNORM;
	case MTLPixelFormatBGRA8Unorm_sRGB: return VK_FORMAT_B8G8R8A8_SRGB;
	case MTLPixelFormatRGBA8Unorm: return VK_FORMAT_R8G8B8A8_UNORM;
	case MTLPixelFormatRGBA8Unorm_sRGB: return VK_FORMAT_R8G8B8A8_SRGB;
	case MTLPixelFormatRGBA16Float: return VK_FORMAT_R16G16B16A16_SFLOAT;
	case MTLPixelFormatRGB10A2Unorm: return VK_FORMAT_A2B10G10R10_UNORM_PACK32;
	case MTLPixelFormatDepth32Float: return VK_FORMAT_D32_SFLOAT;
	case MTLPixelFormatDepth32Float_Stencil8: return VK_FORMAT_D32_SFLOAT_S8_UINT;
	default: U_LOG_W("Cannot convert Metal format %" PRIu64 " to Vulkan format!", format); return 0;
	}
}

int64_t
mtl_format_from_vk(int64_t format)
{
	switch (format) {
	case VK_FORMAT_B8G8R8A8_UNORM: return MTLPixelFormatBGRA8Unorm;
	case VK_FORMAT_B8G8R8A8_SRGB: return MTLPixelFormatBGRA8Unorm_sRGB;
	case VK_FORMAT_R8G8B8A8_UNORM: return MTLPixelFormatRGBA8Unorm;
	case VK_FORMAT_R8G8B8A8_SRGB: return MTLPixelFormatRGBA8Unorm_sRGB;
	case VK_FORMAT_R16G16B16A16_SFLOAT: return MTLPixelFormatRGBA16Float;
	case VK_FORMAT_A2B10G10R10_UNORM_PACK32: return MTLPixelFormatRGB10A2Unorm;
	case VK_FORMAT_D32_SFLOAT: return MTLPixelFormatDepth32Float;
	case VK_FORMAT_D32_SFLOAT_S8_UINT: return MTLPixelFormatDepth32Float_Stencil8;
	default: U_LOG_W("Cannot convert Vulkan format %" PRIu64 " to Metal format!", format); return 0;
	}
}
