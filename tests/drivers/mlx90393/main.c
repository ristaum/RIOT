/*
 * Copyright (C) 2023 Michael Ristau
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mlx90393
 * @{
 *
 * @file
 * @brief       Test application for the MLX90393 magnetometer driver.
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 *
 * ## About
 *
 * The test application demonstrates the use of different functions of
 * the MLX90393 sensor driver depending on the used modules and configuration params.
 *
 * By default the test application uses the I2C bus and the default params set
 * defined in file mlx90393_params.h. The default params use the Burst mode
 * and interrupts to wait for the sensor data to be ready.
 *
 * ## Usage
 * To compile and execute the test application, use command in the test directory:
 * make BOARD=... flash
 *
 * To test the different driver functions you can overwrite the parameters in the
 * default configuration set.
 *
 * Some examples:
 *
 * Wake-up on change mode absolute:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CFLAGS="-DMLX90393_PARAM_MODE=MLX90393_MODE_WOC_ABSOLUTE" \
 * make BOARD=... flash
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Single measurement mode:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CFLAGS="-DMLX90393_PARAM_MODE=MLX90393_MODE_SINGLE_MEASUREMENT" \
 * make BOARD=... flash
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Deactivate interrupt:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CFLAGS="-DMLX90393_PARAM_INT_PIN=GPIO_UNDEF" \
 * make BOARD=... flash
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * To test the sensor with the SPI Interface you can use the mlx90393_spi module:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * DRIVER='mlx90393_spi' \
 * make BOARD=... flash
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "mlx90393.h"
#include "mlx90393_params.h"
#include "ztimer.h"

#define BUS_ERROR                   -EIO
#define DEVICE_NOT_AVAILABLE        -ENXIO
#define INVALID_PARAM               -EINVAL
#define DEVICE_ERROR                -EFAULT

static void print_error(int error)
{
    switch (error)
    {
    case 0:
        puts("No error");
        break;
    case BUS_ERROR:
        puts("Communication bus error");
        break;
    case DEVICE_NOT_AVAILABLE:
        puts("Connectivity error, device not available");
        break;
    case INVALID_PARAM:
        puts("Invalid configuration parameter");
        break;
    case DEVICE_ERROR:
        puts("Device error");
        break;
    default:
        break;
    }
}

int main(void)
{
    puts("MAG3110 magnetometer driver test application\n\r");
#if MODULE_MLX90393_SPI
    printf("Initializing MLX90393 magnetometer at SPI_%i", mlx90393_params[0].spi);
#elif MODULE_MLX90393_I2C
    printf("Initializing MLX90393 magnetometer at I2C_%i", mlx90393_params[0].i2c);
#endif
    mlx90393_t dev;
    int error = 0;
    if ((error = mlx90393_init(&dev, &mlx90393_params[0])) != 0) {
        puts("[FAILED]");
        print_error(error);
        return -1;
    }
    puts("[SUCCESS]");

    unsigned count = 0;

    mlx90393_data_t data;
    puts("Starting read data from the device");
    while (1) {
        if ((error = mlx90393_read(&dev, &data)) != 0) {
            puts("Failed to read data from the device");
            print_error(error);
            return -1;
        }
        printf("Field strength: X: %ld uT Y: %ld uT Z: %ld uT\n\r",
            data.x_axis, data.y_axis, data.z_axis);
        printf("Temperature: %d d°C\n\r", data.temp);

        if (dev.params->mode == MLX90393_MODE_SINGLE_MEASUREMENT) {
            ztimer_sleep(ZTIMER_SEC, 1);
        }

        /*
         * the continuous measurement is stopped, the sensor is set to idle mode
         * and started again after 5 seconds every 50 cycles
         */
        count++;
        if (dev.params->mode != MLX90393_MODE_SINGLE_MEASUREMENT && count == 50) {
            mlx90393_stop_cont(&dev);
            puts("Measurement stopped and sensor set to idle mode.");
            ztimer_sleep(ZTIMER_SEC, 5);
            mlx90393_start_cont(&dev);
            puts("Measurement started again.");
            count = 0;
        }
    }
    return 0;
}
