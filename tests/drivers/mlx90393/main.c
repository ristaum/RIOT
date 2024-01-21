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
 * @}
 */

#define MLX90393_PARAM_GAIN             (MLX90393_GAIN_2X)

#include <stdio.h>
#include <stdlib.h>
#include "mlx90393.h"
#include "mlx90393_params.h"
#include "shell.h"

static mlx90393_t dev;

static void print_error(int error)
{
    switch (error)
    {
    case MLX90393_SUCCESS:
        puts("No error");
        break;
    case MLX90393_ERROR_I2C:
        puts("I2C error");
        break;
    case MLX90393_ERROR_SPI:
        puts("SPI error");
        break;
    case MLX90393_ERROR_NOT_AVAILABLE:
        puts("Connectivity error, device not available");
        break;
    case MLX90393_ERROR_NO_PIN:
        puts("No Interrupt pin configured");
        break;
    case MLX90393_ERROR:
        puts("Device error");
        break;
    default:
        break;
    }
}

static int shell_mlx90393_init_burst_mode(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
#if MODULE_MLX90393_SPI
    printf("Initializing MLX90393 magnetometer in burst mode at SPI_%i", mlx90393_params[0].spi);
#elif MODULE_MLX90393_I2C
    printf("Initializing MLX90393 magnetometer in burst mode at I2C_%i", mlx90393_params[0].i2c);
#endif
    int error = 0;
    if ((error = mlx90393_init(&dev, &mlx90393_params[0])) != MLX90393_SUCCESS) {
        puts("[FAILED]");
        print_error(error);
        return -1;
    }
    puts("[SUCCESS]");
    return 0;
}

static int shell_mlx90393_init_woc_mode(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    mlx90393_params_t params = mlx90393_params[0];
    params.mode = MLX90393_MODE_WAKE_UP_ON_CHANGE_ABSOLUTE;
    params.treshold.z = 1000;
    params.treshold.xy = 0xFFFF;
    params.treshold.temp = 0xFFFF;
#if MODULE_MLX90393_SPI
    printf("Initializing MLX90393 magnetometer in wake-up on change mode at SPI_%i", mlx90393_params[0].spi);
#elif MODULE_MLX90393_I2C
    printf("Initializing MLX90393 magnetometer in wake-up on change mode at I2C_%i", mlx90393_params[0].i2c);
#endif
    int error = 0;
    if ((error = mlx90393_init(&dev, &params)) != MLX90393_SUCCESS) {
        puts("[FAILED]");
        print_error(error);
        return -1;
    }
    puts("[SUCCESS]");
    return 0;
}

static int shell_mlx90393_init_sm_mode(int argc, char **argv)
{
    (void)argc;
    (void)argv;

     mlx90393_params_t params = mlx90393_params[0];
     params.mode = MLX90393_MODE_SINGLE_MEASUREMENT;
#if MODULE_MLX90393_SPI
    printf("Initializing MLX90393 magnetometer in single measurement mode at SPI_%i", mlx90393_params[0].spi);
#elif MODULE_MLX90393_I2C
    printf("Initializing MLX90393 magnetometer in single measurement mode at I2C_%i", mlx90393_params[0].i2c);
#endif
    int error = 0;
    if ((error = mlx90393_init(&dev, &params)) != MLX90393_SUCCESS) {
        puts("[FAILED]");
        print_error(error);
        return -1;
    }
    puts("[SUCCESS]");
    return 0;
}

static int shell_mlx90393_read(int argc, char **argv)
{
    int n = 1;
    if (argc > 1) {
        n = atoi(argv[1]);
    }

    mlx90393_data_t data;
    int error = 0;
    for (int i = 0; i < n; i++) {
        if ((error = mlx90393_read(&dev, &data)) != MLX90393_SUCCESS) {
            puts("Failed to read data from the device\n\r");
            print_error(error);
            return -1;
        }   
        printf("Field strength: X: %d uT Y: %d uT Z: %d uT\n\r", data.x_axis, data.y_axis, data.z_axis);
        printf("Temperature: %d d°C\n\r", data.temp);
    }
    return 0;
}

static int shell_mlx90393_stop_cont(int argc, char **argv) {
    (void)argc;
    (void)argv;

    int error = 0;
    if ((error = mlx90393_stop_cont(&dev)) != MLX90393_SUCCESS) {
        puts("Failed to stop continuous measurement mode\n\r");
        print_error(error);
        return -1;
    }
    puts("Stopped continuous measurement mode\n\r");
    return 0;
}

static int shell_mlx90393_start_cont(int argc, char **argv) {
    (void)argc;
    (void)argv;

    int error = 0;
    if ((error = mlx90393_start_cont(&dev)) != MLX90393_SUCCESS) {
        puts("Failed to start continuous measurement mode\n\r");
        print_error(error);
        return -1;
    }
    puts("Started continuous measurement mode\n\r");
    return 0;
}

static int shell_mlx90393_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int error = 0;
    if ((error = mlx90393_reset(&dev)) != MLX90393_SUCCESS) {
        puts("Failed to reset device\n\r");
        print_error(error);
        return -1;
    }
    puts("reset device\n\r");
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "init_bm", "init the device in burst mode", shell_mlx90393_init_burst_mode },
    { "init_woc", "init the device in woc mode", shell_mlx90393_init_woc_mode },
    { "init_sm", "init the device in sm mode", shell_mlx90393_init_sm_mode },
    { "read", "read a single or several measurements", shell_mlx90393_read },
    { "stop_cont", "stop continuous measurement mode",  shell_mlx90393_stop_cont},
    { "start_cont", "start continuous measurement mode",  shell_mlx90393_start_cont},
    { "reset", "reset the device", shell_mlx90393_reset },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("MAG3110 magnetometer driver test application\n\r");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
