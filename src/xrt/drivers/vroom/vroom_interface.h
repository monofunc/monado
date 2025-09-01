// Copyright 2022-2025, INSA Rouen Normandie
// SPDX-License-Identifier: BSL-1.0
// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to CAVE driver.
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_vroom
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_vroom CAVE driver
 * @ingroup drv
 */

/*!
 * Create a auto prober for a CAVE device.
 *
 * @ingroup drv_vroom
 */
struct xrt_auto_prober *
vroom_create_auto_prober(void);

/*!
 * Create a CAVE device.
 *
 * This is only exposed so that the prober (in one source file)
 * can call the construction function (in another.)
 * @ingroup drv_vroom
 */
struct xrt_device *
vroom_create(void);

struct xrt_device *
vroom_get_controller(struct xrt_device *dev, int controller);

/*!
 * @dir drivers/vroom
 *
 * @brief @ref drv_vroom files.
 */


#ifdef __cplusplus
}
#endif
