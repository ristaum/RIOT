/*
 * Copyright (C) 2023 mi6527ri
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mlx90393
 *
 * @{
 * @file
 * @brief       Default configuration
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 */

#ifndef MLX90393_PARAMS_H
#define MLX90393_PARAMS_H

#include "board.h"
#include "mlx90393.h"
#include "mlx90393_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef MLX90393_PARAM_PARAM1
#define MLX90393_PARAM_PARAM1
#endif

#ifndef MLX90393_PARAMS
#define MLX90393_PARAMS
#endif
/**@}*/

/**
 * @brief   Configuration struct
 */
static const mlx90393_params_t mlx90393_params[] =
{
    MLX90393_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* MLX90393_PARAMS_H */
/** @} */
