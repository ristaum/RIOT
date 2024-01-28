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
/* Default measurement mode */
#ifndef MLX90393_PARAM_MODE
#define MLX90393_PARAM_MODE             (MLX90393_MODE_BURST)
#endif
/* Default interrupt pin */
#ifndef MLX90393_PARAM_INT_PIN
#define MLX90393_PARAM_INT_PIN          (GPIO_PIN(PORT_C, 8))
#endif
/* Default gain */
#ifndef MLX90393_PARAM_GAIN
#define MLX90393_PARAM_GAIN             (MLX90393_GAIN_1X)
#endif
/* Default resolution */
#ifndef MLX90393_PARAM_RES
#define MLX90393_PARAM_RES              (MLX90393_RES_19)
#endif
/* Default sampling rate */
#ifndef MLX90393_PARAM_ODR
#define MLX90393_PARAM_ODR              (MLX90393_ODR_10HZ)
#endif
/* Default oversampling ratio */
#ifndef MLX90393_PARAM_OSR
#define MLX90393_PARAM_OSR                          \
{                                                   \
    .mag            = MLX90393_OSR_1,               \
    .temp           = MLX90393_OSR_1                \
}
#endif
/* Default digital filter */
#ifndef MLX90393_PARAM_DIG_FILT
#define MLX90393_PARAM_DIG_FILT         (MLX90393_DIG_FILT_1)
#endif
/* Default thresholds for Wake-up On Change mode */
#ifndef MLX90393_PARAM_THRESHOLD
#define MLX90393_PARAM_THRESHOLD                    \
{                                                   \
    .xy             = 0xFFFF,                       \
    .z              = 1000,                         \
    .temp           = 0xFFFF                        \
}
#endif

/* Default configuration for SPI mode */
#if MODULE_MLX90393_SPI
/* Default SPI device */
#ifndef MLX90393_PARAM_SPI
#define MLX90393_PARAM_SPI              (SPI_DEV(0))
#endif
/* Default SPI chip select pin */
#ifndef MLX90393_PARAM_SPI_CS_PIN
#define MLX90393_PARAM_SPI_CS_PIN       (GPIO_PIN(PORT_B, 6))
#endif
/* Default SPI clock speed */
#ifndef MLX90393_PARAM_SPI_CLK
#define MLX90393_PARAM_SPI_CLK          (SPI_CLK_10MHZ)
#endif
/* Default SPI params */
#ifndef MLX90393_PARAMS_SPI
#define MLX90393_PARAMS_SPI                         \
{                                                   \
    .spi            = MLX90393_PARAM_SPI,           \
    .cs_pin         = MLX90393_PARAM_SPI_CS_PIN,    \
    .clk            = MLX90393_PARAM_SPI_CLK,       \
    .mode           = MLX90393_PARAM_MODE,          \
    .int_pin        = MLX90393_PARAM_INT_PIN,       \
    .gain           = MLX90393_PARAM_GAIN,          \
    .resolution     = MLX90393_PARAM_RES,           \
    .odr            = MLX90393_PARAM_ODR,           \
    .oversampling   = MLX90393_PARAM_OSR,           \
    .dig_filt       = MLX90393_PARAM_DIG_FILT,      \
    .treshold       = MLX90393_PARAM_THRESHOLD      \
}
#endif

/* Default configuration for I2C mode */
#elif MODULE_MLX90393_I2C
/* Default I2C device */
#ifndef MLX90393_PARAM_I2C
#define MLX90393_PARAM_I2C            (I2C_DEV(0))
#endif
/* Default I2C device address */
#ifndef MLX90393_PARAM_I2C_ADDR
#define MLX90393_PARAM_I2C_ADDR       (0x0C)
#endif
/* Default I2C params */
#ifndef MLX90393_PARAMS_I2C
#define MLX90393_PARAMS_I2C                         \
{                                                   \
    .i2c            = MLX90393_PARAM_I2C,           \
    .addr           = MLX90393_PARAM_I2C_ADDR,      \
    .mode           = MLX90393_PARAM_MODE,          \
    .int_pin        = MLX90393_PARAM_INT_PIN,       \
    .gain           = MLX90393_PARAM_GAIN,          \
    .resolution     = MLX90393_PARAM_RES,           \
    .odr            = MLX90393_PARAM_ODR,           \
    .oversampling   = MLX90393_PARAM_OSR,           \
    .dig_filt       = MLX90393_PARAM_DIG_FILT,      \
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
