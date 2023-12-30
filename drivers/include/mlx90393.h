/*
 * Copyright (C) 2023 mi6527ri
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mlx90393 MLX90393
 * @ingroup     drivers_sensors
 * @brief       Device driver for the MLX90393 3-axis magnetometer
 *
 * @{
 *
 * @file
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 */

#ifndef MLX90393_H
#define MLX90393_H

#include "periph/i2c.h"
#include "periph/gpio.h"

/* Add header includes here */

#ifdef __cplusplus
extern "C" //{
#endif

/**
 * @brief   3d data container of the MLX90393 sensor
 */
typedef struct {
    int x_axis;                  /** Magnometer data from x-axis */
    int y_axis;                  /** Magnometer data from y_axis */
    int z_axis;                  /** Magnometer data from z_axis */
    int temp;
} mlx90393_3d_data_t;

typedef enum {
    MLX90393_MODE_BURST,
    MLX90393_MODE_SINGLE_MEASUREMENT,
    MLX90393_MODE_WAKE_UP_ON_CHANGE_ABSOLUTE,
    MLX90393_MODE_WAKE_UP_ON_CHANGE_RELATIVE
} mlx90393_mode_t;

typedef enum {
    MLX90393_ODR_50HZ = 0x01,
    MLX90393_ODR_25HZ = 0x02,
    MLX90393_ODR_12_5HZ = 0x04,
    MLX90393_ODR_10HZ = 0x05,
    MLX90393_ODR_6_25HZ = 0x08,
    MLX90393_ODR_5HZ = 0x0A,
    MLX90393_ODR_2_5HZ = 0x14,
    MLX90393_ODR_2HZ = 0x19,
    MLX90393_ODR_1_25HZ = 0x28,
    MLX90393_ODR_1HZ = 0x32
} mlx90393_odr_t;

typedef struct {
    int xy;
    int z;
    int temp;
} mlx90393_treshold_t;

typedef enum {
    MLX90393_GAIN_5X,
    MLX90393_GAIN_4X,
    MLX90393_GAIN_3X,
    MLX90393_GAIN_2_5X,
    MLX90393_GAIN_2X,
    MLX90393_GAIN_1_67X,
    MLX90393_GAIN_1_33X,
    MLX90393_GAIN_1X
} mlx90393_gain_t;

typedef enum {
    MLX90393_RES_16,
    MLX90393_RES_17,
    MLX90393_RES_18,
    MLX90393_RES_19,
} mlx90393_resolution_t;

typedef enum {
    MLX90393_TEMP_COMP_OFF,
    MLX90393_TEMP_COMP_ON
} mlx90393_temp_comp_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mlx90393_offset_t;

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    i2c_t i2c;                      /** I2C device */
    uint8_t addr;                   /** Magnometer I2C address */
    mlx90393_mode_t mode;           /**  */
    gpio_t int_pin;
    mlx90393_gain_t gain;
    mlx90393_resolution_t resolution;
    mlx90393_temp_comp_t temp_comp;
    mlx90393_offset_t offset;
    mlx90393_odr_t odr;
    mlx90393_treshold_t treshold;
} mlx90393_params_t;

/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
    /** Device initialization parameters */
    mlx90393_params_t params;
    uint16_t ref_temp;
} mlx90393_t;

enum {
    MLX90393_SUCCESS = 0,
    MLX90393_ERROR_I2C = -1,
    MLX90393_ERROR_SPI = -2,
    MLX90393_ERROR_NO_PIN = -3,
    MLX90393_ERROR = -4
};

/**
 * @brief   Initialize the given device
 *
 * @param[inout] dev        Device descriptor of the driver
 * @param[in]    params     Initialization parameters
 *
 * @return                  0 on success
 */
int mlx90393_init(mlx90393_t *dev, const mlx90393_params_t *params);

int mlx90393_read_measurement(mlx90393_t *dev, mlx90393_3d_data_t *data);

int mlx90393_reset(mlx90393_t *dev);

#ifdef __cplusplus
//}
#endif

#endif /* MLX90393_H */
/** @} */
