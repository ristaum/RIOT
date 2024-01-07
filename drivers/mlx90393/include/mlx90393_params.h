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
 * @brief       Default configuration for MLX90393 device driver
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
 * @name    Set default configuration parameters for the MLX90393
 * @{
 */
#ifndef MLX90393_PARAM_INT_PIN
#define MLX90393_PARAM_INT_PIN      (GPIO_PIN(PORT_C, 8))
#endif

#ifndef MLX90393_PARAM_OFFSET
#define MLX90393_PARAM_OFFSET                       \
{                                                   \
    .x              = 0,                            \
    .y              = 0,                            \
    .z              = 0                             \
}
#endif

#ifndef MLX90393_PARAM_THRESHOLD
#define MLX90393_PARAM_THRESHOLD                    \
{                                                   \
    .xy             = 0,                            \
    .z              = 0,                            \
    .temp           = 0                             \
}
#endif

/* default configuration for SPI mode */
#if MODULE_MLX90393_SPI
#ifndef MLX90393_PARAM_SPI_CS_PIN
#define MLX90393_PARAM_SPI_CS_PIN    (GPIO_PIN(PORT_C, 6))
#endif

#ifndef MLX90393_PARAMS_SPI
#define MLX90393_PARAMS_SPI                         \
{                                                   \
    .spi            = SPI_DEV(0),                   \
    .cs_pin         = MLX90393_PARAM_SPI_CS_PIN,    \
    .mode           = MLX90393_MODE_BURST,          \
    .int_pin        = MLX90393_PARAM_INT_PIN,       \
    .gain           = MLX90393_GAIN_1X,             \
    .resolution     = MLX90393_RES_19,              \
    .temp_comp      = MLX90393_TEMP_COMP_OFF,       \
    .offset         = MLX90393_PARAM_OFFSET,        \
    .odr            = MLX90393_ODR_10HZ,            \
    .treshold       = MLX90393_PARAM_THRESHOLD      \
}
#endif

/* default configuration for I2C mode */
#elif MODULE_MLX90393_I2C
#ifndef MLX90393_PARAM_I2C_ADDR
#define MLX90393_PARAM_I2C_ADDR       (0x0C)
#endif

#ifndef MLX90393_PARAMS_I2C
#define MLX90393_PARAMS_I2C                         \
{                                                   \
    .i2c            = I2C_DEV(0),                   \
    .addr           = MLX90393_PARAM_I2C_ADDR,      \
    .mode           = MLX90393_MODE_BURST,          \
    .int_pin        = MLX90393_PARAM_INT_PIN,       \
    .gain           = MLX90393_GAIN_1X,             \
    .resolution     = MLX90393_RES_19,              \
    .temp_comp      = MLX90393_TEMP_COMP_OFF,       \
    .offset         = MLX90393_PARAM_OFFSET,        \
    .odr            = MLX90393_ODR_10HZ,            \
    .treshold       = MLX90393_PARAM_THRESHOLD      \
}
#endif
#endif
/**@}*/

/**
 * @brief   Configure params for MLX90393
 */
static const mlx90393_params_t mlx90393_params[] =
{
#if MODULE_MLX90393_SPI
    MLX90393_PARAMS_SPI
#elif MODULE_MLX90393_I2C
    MLX90393_PARAMS_I2C
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* MLX90393_PARAMS_H */
/** @} */
