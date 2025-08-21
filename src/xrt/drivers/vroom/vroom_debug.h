/*!
 * @file
 * @brief  Interface for the CAVE information window.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#pragma once

#include "vroom_device.h"

#ifdef __cplusplus
extern "C" {
#endif

int
vroom_debug_window(struct vroom_device *vroom);

void
vroom_close_debug_window();

#ifdef __cplusplus
}
#endif