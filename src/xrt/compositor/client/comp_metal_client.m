// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metal client side glue to compositor implementation.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup comp_client
 */

#include "xrt/xrt_compositor.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"

#include "metal/mtl_format.h"
#include "metal/mtl_image_collection.h"

#import <Metal/Metal.h>

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>


DEBUG_GET_ONCE_LOG_OPTION(log, "METAL_COMPOSITOR_LOG", U_LOGGING_INFO)

#define LOGT(...) U_LOG_IFL_T(debug_get_log_option_log(), __VA_ARGS__)
#define LOGD(...) U_LOG_IFL_D(debug_get_log_option_log(), __VA_ARGS__)
#define LOGI(...) U_LOG_IFL_I(debug_get_log_option_log(), __VA_ARGS__)
#define LOGW(...) U_LOG_IFL_W(debug_get_log_option_log(), __VA_ARGS__)
#define LOGE(...) U_LOG_IFL_E(debug_get_log_option_log(), __VA_ARGS__)


/*
 *
 * Structs.
 *
 */

/*!
 * Wraps the real compositor swapchain providing a Metal based interface.
 *
 * @ingroup comp_client
 * @implements xrt_swapchain_metal
 */
struct client_metal_swapchain
{
	//! Implements @ref xrt_swapchain_metal
	struct xrt_swapchain_metal base;

	//! Owning reference to the backing native swapchain.
	struct xrt_swapchain_native *xscn;

	//! Allocation collection for Metal textures (owning reference).
	struct xrt_allocation_collection *xac;

	//! Non-owning reference to our parent compositor.
	struct client_metal_compositor *c;
};

/*!
 * Wraps the real compositor providing a Metal based interface.
 *
 * @ingroup comp_client
 * @implements xrt_compositor_metal
 */
struct client_metal_compositor
{
	//! Implements @ref xrt_compositor_metal
	struct xrt_compositor_metal base;

	//! Owning reference to the backing native compositor.
	struct xrt_compositor_native *xcn;

	//! Command queue from the application (retained).
	id<MTLCommandQueue> command_queue;

	//! Metal device (retained).
	id<MTLDevice> device;
};


/*
 *
 * Helpers.
 *
 */

/*!
 * Down-cast helper.
 * @private @memberof client_metal_swapchain
 */
static inline struct client_metal_swapchain *
client_metal_swapchain(struct xrt_swapchain *xsc)
{
	return (struct client_metal_swapchain *)xsc;
}

/*!
 * Down-cast helper.
 * @private @memberof client_metal_compositor
 */
static inline struct client_metal_compositor *
client_metal_compositor(struct xrt_compositor *xc)
{
	return (struct client_metal_compositor *)xc;
}

static inline struct xrt_swapchain *
to_native_swapchain(struct xrt_swapchain *xsc)
{
	return &client_metal_swapchain(xsc)->xscn->base;
}

static inline struct xrt_compositor *
to_native_compositor(struct xrt_compositor *xc)
{
	return &client_metal_compositor(xc)->xcn->base;
}


/*
 *
 * Swapchain functions.
 *
 */

static void
client_metal_swapchain_destroy(struct xrt_swapchain *xsc)
{
	struct client_metal_swapchain *sc = client_metal_swapchain(xsc);

	// Clear image pointers.
	for (uint32_t i = 0; i < sc->base.base.image_count; i++) {
		sc->base.images[i] = NULL;
	}

	// Release the allocation collection (handles cleanup of Metal textures).
	xrt_allocation_collection_reference(&sc->xac, NULL);

	// Drop our reference to the native swapchain.
	xrt_swapchain_native_reference(&sc->xscn, NULL);

	free(sc);
}

static xrt_result_t
client_metal_swapchain_acquire_image(struct xrt_swapchain *xsc, uint32_t *out_index)
{
	// Pipe down call into native swapchain.
	return xrt_swapchain_acquire_image(to_native_swapchain(xsc), out_index);
}

static xrt_result_t
client_metal_swapchain_wait_image(struct xrt_swapchain *xsc, int64_t timeout_ns, uint32_t index)
{
	// Pipe down call into native swapchain.
	return xrt_swapchain_wait_image(to_native_swapchain(xsc), timeout_ns, index);
}

static xrt_result_t
client_metal_swapchain_barrier_image(struct xrt_swapchain *xsc, enum xrt_barrier_direction direction, uint32_t index)
{
	return XRT_SUCCESS;
}

static xrt_result_t
client_metal_swapchain_release_image(struct xrt_swapchain *xsc, uint32_t index)
{
	// Pipe down call into native swapchain.
	return xrt_swapchain_release_image(to_native_swapchain(xsc), index);
}


/*
 *
 * Compositor functions.
 *
 */

static xrt_result_t
client_metal_compositor_begin_session(struct xrt_compositor *xc, const struct xrt_begin_session_info *info)
{
	// Pipe down call into native compositor.
	return xrt_comp_begin_session(to_native_compositor(xc), info);
}

static xrt_result_t
client_metal_compositor_end_session(struct xrt_compositor *xc)
{
	// Pipe down call into native compositor.
	return xrt_comp_end_session(to_native_compositor(xc));
}

static xrt_result_t
client_metal_compositor_wait_frame(struct xrt_compositor *xc,
                                   int64_t *out_frame_id,
                                   int64_t *predicted_display_time,
                                   int64_t *predicted_display_period)
{
	// Pipe down call into native compositor.
	return xrt_comp_wait_frame(    //
	    to_native_compositor(xc),  //
	    out_frame_id,              //
	    predicted_display_time,    //
	    predicted_display_period); //
}

static xrt_result_t
client_metal_compositor_begin_frame(struct xrt_compositor *xc, int64_t frame_id)
{
	// Pipe down call into native compositor.
	return xrt_comp_begin_frame(to_native_compositor(xc), frame_id);
}

static xrt_result_t
client_metal_compositor_discard_frame(struct xrt_compositor *xc, int64_t frame_id)
{
	// Pipe down call into native compositor.
	return xrt_comp_discard_frame(to_native_compositor(xc), frame_id);
}

static xrt_result_t
client_metal_compositor_layer_begin(struct xrt_compositor *xc, const struct xrt_layer_frame_data *data)
{
	// Pipe down call into native compositor.
	return xrt_comp_layer_begin(to_native_compositor(xc), data);
}

static xrt_result_t
client_metal_compositor_layer_projection(struct xrt_compositor *xc,
                                         struct xrt_device *xdev,
                                         struct xrt_swapchain *xsc[XRT_MAX_VIEWS],
                                         const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscn[XRT_MAX_VIEWS];

	xcn = to_native_compositor(xc);
	assert(data->type == XRT_LAYER_PROJECTION);
	for (uint32_t i = 0; i < data->view_count; ++i) {
		xscn[i] = &client_metal_swapchain(xsc[i])->xscn->base;
	}

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_projection(xcn, xdev, xscn, &d);
}

static xrt_result_t
client_metal_compositor_layer_projection_depth(struct xrt_compositor *xc,
                                               struct xrt_device *xdev,
                                               struct xrt_swapchain *xsc[XRT_MAX_VIEWS],
                                               struct xrt_swapchain *d_xsc[XRT_MAX_VIEWS],
                                               const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscn[XRT_MAX_VIEWS];
	struct xrt_swapchain *d_xscn[XRT_MAX_VIEWS];

	assert(data->type == XRT_LAYER_PROJECTION_DEPTH);

	xcn = to_native_compositor(xc);
	for (uint32_t i = 0; i < data->view_count; ++i) {
		xscn[i] = to_native_swapchain(xsc[i]);
		d_xscn[i] = to_native_swapchain(d_xsc[i]);
	}

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_projection_depth(xcn, xdev, xscn, d_xscn, &d);
}

static xrt_result_t
client_metal_compositor_layer_quad(struct xrt_compositor *xc,
                                   struct xrt_device *xdev,
                                   struct xrt_swapchain *xsc,
                                   const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscfb;

	assert(data->type == XRT_LAYER_QUAD);

	xcn = to_native_compositor(xc);
	xscfb = to_native_swapchain(xsc);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_quad(xcn, xdev, xscfb, &d);
}

static xrt_result_t
client_metal_compositor_layer_cube(struct xrt_compositor *xc,
                                   struct xrt_device *xdev,
                                   struct xrt_swapchain *xsc,
                                   const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscfb;

	assert(data->type == XRT_LAYER_CUBE);

	xcn = to_native_compositor(xc);
	xscfb = to_native_swapchain(xsc);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_cube(xcn, xdev, xscfb, &d);
}

static xrt_result_t
client_metal_compositor_layer_cylinder(struct xrt_compositor *xc,
                                       struct xrt_device *xdev,
                                       struct xrt_swapchain *xsc,
                                       const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscfb;

	assert(data->type == XRT_LAYER_CYLINDER);

	xcn = to_native_compositor(xc);
	xscfb = to_native_swapchain(xsc);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_cylinder(xcn, xdev, xscfb, &d);
}

static xrt_result_t
client_metal_compositor_layer_equirect1(struct xrt_compositor *xc,
                                        struct xrt_device *xdev,
                                        struct xrt_swapchain *xsc,
                                        const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscfb;

	assert(data->type == XRT_LAYER_EQUIRECT1);

	xcn = to_native_compositor(xc);
	xscfb = to_native_swapchain(xsc);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_equirect1(xcn, xdev, xscfb, &d);
}

static xrt_result_t
client_metal_compositor_layer_equirect2(struct xrt_compositor *xc,
                                        struct xrt_device *xdev,
                                        struct xrt_swapchain *xsc,
                                        const struct xrt_layer_data *data)
{
	struct xrt_compositor *xcn;
	struct xrt_swapchain *xscfb;

	assert(data->type == XRT_LAYER_EQUIRECT2);

	xcn = to_native_compositor(xc);
	xscfb = to_native_swapchain(xsc);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_equirect2(xcn, xdev, xscfb, &d);
}

static xrt_result_t
client_metal_compositor_layer_passthrough(struct xrt_compositor *xc,
                                          struct xrt_device *xdev,
                                          const struct xrt_layer_data *data)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	assert(data->type == XRT_LAYER_PASSTHROUGH);

	struct xrt_layer_data d = *data;
	d.flip_y = !d.flip_y;

	return xrt_comp_layer_passthrough(&c->xcn->base, xdev, &d);
}

static xrt_result_t
client_metal_compositor_layer_commit(struct xrt_compositor *xc, xrt_graphics_sync_handle_t sync_handle)
{
	COMP_TRACE_MARKER();

	struct client_metal_compositor *c = client_metal_compositor(xc);

	// We make the sync object, not st/oxr which is our user.
	assert(!xrt_graphics_sync_handle_is_valid(sync_handle));

	// TODO: implement proper synchronization with Metal fences/events

	COMP_TRACE_IDENT(layer_commit);

	return xrt_comp_layer_commit(&c->xcn->base, XRT_GRAPHICS_SYNC_HANDLE_INVALID);
}

static xrt_result_t
client_metal_compositor_get_swapchain_create_properties(struct xrt_compositor *xc,
                                                        const struct xrt_swapchain_create_info *info,
                                                        struct xrt_swapchain_create_properties *xsccp)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	return xrt_comp_get_swapchain_create_properties(&c->xcn->base, info, xsccp);
}

static xrt_result_t
client_metal_swapchain_create(struct xrt_compositor *xc,
                              const struct xrt_swapchain_create_info *xinfo,
                              struct xrt_swapchain **out_xsc)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);
	struct xrt_swapchain_create_properties xsccp = {0};
	xrt_result_t xret = XRT_SUCCESS;

	// Get create properties.
	xret = xrt_comp_get_swapchain_create_properties(xc, xinfo, &xsccp);
	if (xret != XRT_SUCCESS) {
		LOGE("Failed to get create properties: %u", xret);
		return xret;
	}

	struct xrt_swapchain_create_info xinfo_copy = *xinfo;

	// Update the create info.
	xinfo_copy.bits |= xsccp.extra_bits;
	xinfo_copy.format = mtl_format_to_vk(xinfo_copy.format);

	// Create native swapchain (for compositor layer submission).
	struct xrt_swapchain_native *xscn = NULL;
	xret = xrt_comp_native_create_swapchain(c->xcn, &xinfo_copy, &xscn);
	if (xret != XRT_SUCCESS) {
		LOGE("Failed to create native swapchain: %u", xret);
		return xret;
	}
	assert(xscn != NULL);

	// Allocate the client swapchain.
	struct client_metal_swapchain *sc = U_TYPED_CALLOC(struct client_metal_swapchain);
	if (sc == NULL) {
		LOGE("Failed to allocate client_metal_swapchain");
		xrt_swapchain_native_reference(&xscn, NULL);
		return XRT_ERROR_ALLOCATION;
	}

	// Setup the client swapchain.
	sc->base.base.destroy = client_metal_swapchain_destroy;
	sc->base.base.acquire_image = client_metal_swapchain_acquire_image;
	sc->base.base.wait_image = client_metal_swapchain_wait_image;
	sc->base.base.barrier_image = client_metal_swapchain_barrier_image;
	sc->base.base.release_image = client_metal_swapchain_release_image;
	sc->base.base.image_count = xscn->base.image_count;
	sc->base.base.reference.count = 1;
	sc->xscn = xscn;
	sc->c = c;

	// Check if the native swapchain has an allocation collection we can reuse.
	bool reuse_xac = false;
	if (xscn->xac != NULL) {
		LOGI("xac set on native");
		// Get the first texture to check if the device matches.
		id<MTLTexture> textures[XRT_MAX_SWAPCHAIN_IMAGES] = {};
		xret = xrt_allocation_collection_get_all( //
		    xscn->xac,                            //
		    XRT_ALLOCATION_TYPE_METAL_TEXTURE,    //
		    sizeof(textures[0]),                  //
		    &textures);                           //
		if (xret == XRT_SUCCESS && textures[0] != nil) {
			// Check if the texture's device matches our compositor's device.
			if ([textures[0] device] == c->device) {
				reuse_xac = true;
			}
		}
	}

	if (reuse_xac) {
		LOGI("Reusing allocation collection from native swapchain");
		// Reuse the allocation collection from the native swapchain.
		xrt_allocation_collection_reference(&sc->xac, xscn->xac);
	} else {
		LOGI("Creating out own image collection");
		// Create the image collection with Metal textures using the helper.
		xret = mtl_image_collection_create( //
		    c->device,                      //
		    &xinfo_copy,                    //
		    xscn->base.image_count,         //
		    &sc->xac);                      //
		if (xret != XRT_SUCCESS) {
			LOGE("Failed to create Metal allocation collection: %u", xret);
			xrt_swapchain_native_reference(&xscn, NULL);
			free(sc);
			return xret;
		}
	}

	// Get the Metal textures from the allocation collection.
	xret = xrt_allocation_collection_get_all( //
	    sc->xac,                              //
	    XRT_ALLOCATION_TYPE_METAL_TEXTURE,    //
	    sizeof(sc->base.images[0]),           //
	    sc->base.images);                     //
	if (xret != XRT_SUCCESS) {
		LOGE("Failed to get Metal textures from allocation collection: %u", xret);
		xrt_allocation_collection_reference(&sc->xac, NULL);
		xrt_swapchain_native_reference(&xscn, NULL);
		free(sc);
		return xret;
	}

	*out_xsc = &sc->base.base;
	return XRT_SUCCESS;
}

static xrt_result_t
client_metal_compositor_passthrough_create(struct xrt_compositor *xc, const struct xrt_passthrough_create_info *info)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	// Pipe down call into native compositor.
	return xrt_comp_create_passthrough(&c->xcn->base, info);
}

static xrt_result_t
client_metal_compositor_passthrough_layer_create(struct xrt_compositor *xc,
                                                 const struct xrt_passthrough_layer_create_info *info)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	// Pipe down call into native compositor.
	return xrt_comp_create_passthrough_layer(&c->xcn->base, info);
}

static xrt_result_t
client_metal_compositor_passthrough_destroy(struct xrt_compositor *xc)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	// Pipe down call into native compositor.
	return xrt_comp_destroy_passthrough(&c->xcn->base);
}

static void
client_metal_compositor_destroy(struct xrt_compositor *xc)
{
	struct client_metal_compositor *c = client_metal_compositor(xc);

	// Release Metal objects.
	if (c->device != nil) {
		[c->device release];
		c->device = nil;
	}
	if (c->command_queue != nil) {
		[c->command_queue release];
		c->command_queue = nil;
	}

	free(c);
}


/*
 *
 * Hardcoded format list.
 *
 */

static const int64_t metal_formats[] = {
    MTLPixelFormatBGRA8Unorm,            // 80 - Common color format
    MTLPixelFormatBGRA8Unorm_sRGB,       // 81 - sRGB version
    MTLPixelFormatRGBA8Unorm,            // 70 - RGBA
    MTLPixelFormatRGBA8Unorm_sRGB,       // 71 - sRGB version
    MTLPixelFormatRGBA16Float,           // 115 - HDR
    MTLPixelFormatRGB10A2Unorm,          // 90 - 10-bit color
    MTLPixelFormatDepth32Float,          // 252 - Depth
    MTLPixelFormatDepth32Float_Stencil8, // 260 - Depth + Stencil
};

#define METAL_FORMAT_COUNT (sizeof(metal_formats) / sizeof(metal_formats[0]))


/*
 *
 * Exported functions.
 *
 */

struct xrt_compositor_metal *
xrt_gfx_metal_provider_create(struct xrt_compositor_native *xcn, void *command_queue)
{
	// Validate that command_queue is not NULL.
	if (command_queue == NULL) {
		LOGE("command_queue parameter is NULL");
		return NULL;
	}

	struct client_metal_compositor *c = U_TYPED_CALLOC(struct client_metal_compositor);
	if (c == NULL) {
		LOGE("Failed to allocate client_metal_compositor");
		return NULL;
	}

	c->base.base.get_swapchain_create_properties = client_metal_compositor_get_swapchain_create_properties;
	c->base.base.create_swapchain = client_metal_swapchain_create;
	c->base.base.create_passthrough = client_metal_compositor_passthrough_create;
	c->base.base.create_passthrough_layer = client_metal_compositor_passthrough_layer_create;
	c->base.base.destroy_passthrough = client_metal_compositor_passthrough_destroy;
	c->base.base.begin_session = client_metal_compositor_begin_session;
	c->base.base.end_session = client_metal_compositor_end_session;
	c->base.base.wait_frame = client_metal_compositor_wait_frame;
	c->base.base.begin_frame = client_metal_compositor_begin_frame;
	c->base.base.discard_frame = client_metal_compositor_discard_frame;
	c->base.base.layer_begin = client_metal_compositor_layer_begin;
	c->base.base.layer_projection = client_metal_compositor_layer_projection;
	c->base.base.layer_projection_depth = client_metal_compositor_layer_projection_depth;
	c->base.base.layer_quad = client_metal_compositor_layer_quad;
	c->base.base.layer_cube = client_metal_compositor_layer_cube;
	c->base.base.layer_cylinder = client_metal_compositor_layer_cylinder;
	c->base.base.layer_equirect1 = client_metal_compositor_layer_equirect1;
	c->base.base.layer_equirect2 = client_metal_compositor_layer_equirect2;
	c->base.base.layer_passthrough = client_metal_compositor_layer_passthrough;
	c->base.base.layer_commit = client_metal_compositor_layer_commit;
	c->base.base.destroy = client_metal_compositor_destroy;
	c->xcn = xcn;

	// Retain the command queue passed in from the application.
	c->command_queue = [(id<MTLCommandQueue>)command_queue retain];

	// Get and retain the device from the command queue.
	c->device = [[c->command_queue device] retain];

	// Set up hardcoded Metal format list.
	c->base.base.info.format_count = METAL_FORMAT_COUNT;
	for (uint32_t i = 0; i < METAL_FORMAT_COUNT; i++) {
		c->base.base.info.formats[i] = metal_formats[i];
	}

	// Copy other info from native compositor.
	c->base.base.info.max_texture_size = xcn->base.info.max_texture_size;

	return &c->base;
}
