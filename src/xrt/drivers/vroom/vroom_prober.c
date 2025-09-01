// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  CAVE prober code.
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "vroom_interface.h"


/*!
 * @implements xrt_auto_prober
 */
struct vroom_auto_prober
{
	struct xrt_auto_prober base;
};

//! @private @memberof vroom_auto_prober
static inline struct vroom_auto_prober *
vroom_auto_prober(struct xrt_auto_prober *p)
{
	return (struct vroom_auto_prober *)p;
}

//! @private @memberof vroom_auto_prober
static void
vroom_auto_prober_destroy(struct xrt_auto_prober *p)
{
	struct vroom_auto_prober *cap = vroom_auto_prober(p);

	free(cap);
}

//! @public @memberof vroom_auto_prober
static int
vroom_auto_prober_autoprobe(struct xrt_auto_prober *xap,
                            cJSON *attached_data,
                            bool no_hmds,
                            struct xrt_prober *xp,
                            struct xrt_device **out_xdevs)
{
	struct vroom_auto_prober *cap = vroom_auto_prober(xap);
	(void)cap;

	// Do not create a sample HMD if we are not looking for HMDs.
	if (no_hmds) {
		return 0;
	}

	struct xrt_device *device = vroom_create();

	out_xdevs[0] = device;
	out_xdevs[1] = vroom_get_controller(device, 1); // JoyCon Left
	out_xdevs[2] = vroom_get_controller(device, 2); // JoyCon Right

	return 3;
}

struct xrt_auto_prober *
vroom_create_auto_prober()
{
	struct vroom_auto_prober *cap = U_TYPED_CALLOC(struct vroom_auto_prober);
	cap->base.name = "VROOM";
	cap->base.destroy = vroom_auto_prober_destroy;
	cap->base.lelo_dallas_autoprobe = vroom_auto_prober_autoprobe;

	return &cap->base;
}
