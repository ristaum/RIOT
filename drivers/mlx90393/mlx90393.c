/*
 * Copyright (C) 2023 mi6527ri
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
 * @brief       Device driver implementation for the MLX90393
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 *
 * @}
 */

#include "mlx90393.h"
#include "mlx90393_constants.h"
#include "mlx90393_params.h"
#include "ztimer.h"

#if MODULE_MLX90393_SPI
#define DEV_SPI             (dev->params.spi)
#define DEV_CS_PIN          (dev->params.cs_pin)
#elif MODULE_MLX90393_I2C
#define DEV_I2C             (dev->params.i2c)
#define DEV_ADDR            (dev->params.addr)
#endif
#define DEV_MODE            (dev->params.mode)
#define DEV_INT_PIN         (dev->params.int_pin)
#define DEV_ODR             (dev->params.odr)
#define DEV_GAIN            (dev->params.gain)
#define DEV_RESOLUTION      (dev->params.resolution)
#define DEV_TEMP_COMP       (dev->params.temp_comp)
#define DEV_X_OFFSET        (dev->params.offset.x)
#define DEV_Y_OFFSET        (dev->params.offset.y)
#define DEV_Z_OFFSET        (dev->params.offset.z)

#if MODULE_MLX90393_SPI

static int _init_bus(const mlx90393_t *dev)
{
    if (spi_init_cs(DEV_SPI, DEV_CS_PIN) != SPI_OK) {
        return MLX90393_ERROR_SPI;
    }
    return MLX90393_SUCCESS;
}

static void _acquire(mlx90393_t *dev)
{
    spi_acquire(DEV_SPI, DEV_CS_PIN, SPI_MODE_3, SPI_CLK_10MHZ);
}

static void _release(mlx90393_t *dev)
{
    spi_release(DEV_SPI);
}

static int _write_byte(mlx90393_t *dev, uint8_t data)
{
    spi_transfer_byte(DEV_SPI, DEV_CS_PIN, true, data);
    return MLX90393_SUCCESS;
}

static int _read_byte(mlx90393_t *dev, void *buffer)
{
    *((uint8_t*)buffer) = spi_transfer_byte(DEV_SPI, DEV_CS_PIN, true, 0);
    return MLX90393_SUCCESS;
}

static int _write_bytes(mlx90393_t *dev, void *data, size_t len)
{
    spi_transfer_bytes(DEV_SPI, DEV_CS_PIN, true, data, NULL, len);
    return MLX90393_SUCCESS;
}

static int _read_bytes(mlx90393_t *dev, void *buffer, size_t len)
{
    spi_transfer_bytes(DEV_SPI, DEV_CS_PIN, true, NULL, buffer, len);
    return MLX90393_SUCCESS;
}

#elif MODULE_MLX90393_I2C

static int _init_bus(const mlx90393_t *dev)
{
    (void) dev;
    return MLX90393_SUCCESS;
}

static void _acquire(mlx90393_t *dev)
{
    i2c_acquire(DEV_I2C);
}

static void _release(mlx90393_t *dev)
{
    i2c_release(DEV_I2C);
}

static int _write_byte(mlx90393_t *dev, uint8_t data)
{
    return i2c_write_byte(DEV_I2C, DEV_ADDR, data, 0) ? MLX90393_ERROR_I2C : MLX90393_SUCCESS;
}

static int _read_byte(mlx90393_t *dev, void *buffer)
{
    return i2c_read_byte(DEV_I2C, DEV_ADDR, buffer, 0) ? MLX90393_ERROR_I2C : MLX90393_SUCCESS;
}

static int _write_bytes(mlx90393_t *dev, void *data, size_t len)
{
    return i2c_write_bytes(DEV_I2C, DEV_ADDR, data, len, 0) ? MLX90393_ERROR_I2C : MLX90393_SUCCESS;
}

static int _read_bytes(mlx90393_t *dev, void *buffer, size_t len)
{
    return i2c_read_bytes(DEV_I2C, DEV_ADDR, buffer, len, 0) ? MLX90393_ERROR_I2C : MLX90393_SUCCESS;
}

#endif

static int _check_status_byte(mlx90393_t *dev) 
{
    uint8_t status;
    int error = 0;
    if ((error = _read_byte(dev, &status)) != 0) {
        return error;
    }
    return (status & MLX90393_STATUS_ERROR) ? MLX90393_ERROR : MLX90393_SUCCESS;
}

static int _write_register(mlx90393_t *dev, uint8_t addr, uint16_t value) 
{
    uint8_t buffer[4];
    buffer[0] = MLX90393_COMMAND_WR;
    buffer[1] = (uint8_t) (value >> 8);
    buffer[2] = (uint8_t) (value & 0xFF);
    buffer[3] = addr << 2;
    int error = 0;
    if ((error = _write_bytes(dev, buffer, 4)) != 0) {
        return error;
    }
    if ((error = _check_status_byte(dev)) != 0) {
        return error;
    }
    return MLX90393_SUCCESS;
}

static int _read_register(mlx90393_t *dev, uint8_t addr, uint16_t *value)
{
    uint8_t buffer_send[2];
    buffer_send[0] = MLX90393_COMMAND_RR;
    buffer_send[1] = addr << 2;
    int error = 0;
    if ((error = _write_bytes(dev, buffer_send, 2)) != 0) {
        return error;
    }
    uint8_t buffer_receive[3];
    if ((error = _read_bytes(dev, buffer_receive, 3)) != 0) {
        return error;
    }
    if (buffer_receive[0] & MLX90393_STATUS_ERROR) {
        return MLX90393_ERROR;
    }
    *value = (uint16_t)((buffer_receive[1] << 8) | buffer_receive[2]);

    return MLX90393_SUCCESS;
}

static int _write_register_bits(mlx90393_t *dev, uint8_t addr, uint16_t mask, uint16_t value) 
{
    uint16_t reg_value;
    int error = 0;
    if ((error = _read_register(dev, addr, &reg_value)) != 0) {
        return error;
    }
    reg_value &= ~mask;
    reg_value |= (mask & value);
    if ((error = _write_register(dev, addr, reg_value)) != 0) {
        return error;
    }
    return MLX90393_SUCCESS;
}

static int _calculate_temp(uint16_t raw_temp, uint16_t ref_temp)
{
    return (MLX90393_TEMP_OFFSET + (raw_temp - ref_temp) / MLX90393_TEMP_RESOLUTION) * 100;
}

static float _get_gain_factor(mlx90393_gain_t gain)
{
    switch (gain)
    {
    case MLX90393_GAIN_5X:
        return 5.0f;
    case MLX90393_GAIN_4X:
        return 4.0f;
    case MLX90393_GAIN_3X:
        return 3.0f;
    case MLX90393_GAIN_2_5X:
        return 2.5f;
    case MLX90393_GAIN_2X:
        return 2.0f;
    case MLX90393_GAIN_1_67X:
        return 1.6666667f;
    case MLX90393_GAIN_1_33X:
        return 1.3333333f;
    case MLX90393_GAIN_1X:
        return 1.0f;
    default:
        return -1;
    }
}

int mlx90393_init(mlx90393_t *dev, const mlx90393_params_t *params)
{
    assert(dev);
    assert(params);
    dev->params = *params;
    int error = 0;
    if ((error = _init_bus(dev)) != 0) {
        return error;
    }
    _acquire(dev);

    /* reset mlx90393 */
    if ((error = _write_byte(dev, MLX90393_COMMAND_EX)) != 0) {
        _release(dev);
        return error;
    }
    if ((error = _check_status_byte(dev)) != 0) {
        _release(dev);
        return error;
    }
    ztimer_sleep(ZTIMER_USEC, MLX90393_COMMAND_EX_TIMEOUT);
    if (_write_byte(dev, MLX90393_COMMAND_RT) != 0) {
        _release(dev);
        return MLX90393_ERROR_I2C;
    }
    error = _check_status_byte(dev);
    if (error) {
        return error;
    }
    ztimer_sleep(ZTIMER_USEC, MLX90393_COMMAND_RT_TIMEOUT);
    /* store ref temp in dev */
    if ((error = _read_register(dev, MLX90393_REG_REF_TEMP, &dev->ref_temp)) != 0) {
        _release(dev);
        return error;
    }
    /* gain */
    if ((error = _write_register_bits(dev, MLX90393_REG_CONF0, MLX90393_MASK_GAIN_SEL, DEV_GAIN)) != 0) {
        _release(dev);
        return error;
    }
    /* resolution */
    uint16_t xyz_resolution_value_mask = (DEV_RESOLUTION << 9) |
                                         (DEV_RESOLUTION << 7) |
                                         (DEV_RESOLUTION << 5);
    if ((error = _write_register_bits(dev, MLX90393_REG_CONF2, MLX90393_MASK_RES_XYZ, xyz_resolution_value_mask)) != 0) {
        _release(dev);
        return error;
    }
    /* temp compensation / offset */
    //TODO does not work properly
    if (DEV_TEMP_COMP == MLX90393_TEMP_COMP_ON) {
        uint16_t offset_x = 0x8000 + DEV_X_OFFSET / MLX90393_XY_SENS;
        uint16_t offset_y = 0x8000 + DEV_Y_OFFSET / MLX90393_XY_SENS;
        uint16_t offset_z = 0x8000 + DEV_Z_OFFSET / MLX90393_Z_SENS;

        /* enable temp compensation */
        if ((error = _write_register_bits(dev, MLX90393_REG_CONF1, MLX90393_MASK_TCMP_EN, 0x400)) != 0) {
            _release(dev);
            return error;
        }
        /* set offsets */
        if ((error = _write_register(dev, MLX90393_REG_OFFSET_X, offset_x)) != 0) {
            _release(dev);
            return error;
        }
        if ((error = _write_register(dev, MLX90393_REG_OFFSET_Y, offset_y)) != 0) {
            _release(dev);
            return error;
        }
        if ((error = _write_register(dev, MLX90393_REG_OFFSET_Z, offset_z)) != 0) {
            _release(dev);
            return error;
        }
    }
    /* burst more */
    if (DEV_MODE == MLX90393_MODE_BURST) {
        /* set burst data rate */
        if ((error = _write_register_bits(dev, MLX90393_REG_CONF1, MLX90393_MASK_BDR, DEV_ODR)) != 0) {
            _release(dev);
            return error;
        }
        /* start burst mode */
        if ((error = _write_byte(dev, MLX90393_COMMAND_SB)) != 0) {
            _release(dev);
            return error;
        }
        if ((error = _check_status_byte(dev)) != 0) {
            _release(dev);
            return error;
        }
    }
    /* wake up on change mode */
    else if (DEV_MODE == MLX90393_MODE_WAKE_UP_ON_CHANGE_ABSOLUTE || DEV_MODE == MLX90393_MODE_WAKE_UP_ON_CHANGE_RELATIVE) {
        if (!gpio_is_valid(DEV_INT_PIN)) {
            _release(dev);
            return MLX90393_ERROR_NO_PIN;
        }
        /* set absolute or relative wake up on change mode */
        int wake_up_on_chane_mode = 0;
        if (DEV_MODE == MLX90393_MODE_WAKE_UP_ON_CHANGE_RELATIVE) {
            wake_up_on_chane_mode = 1;
        }
        if ((error = _write_register_bits(dev, MLX90393_REG_CONF1, MLX90393_MASK_WOC_DIFF, wake_up_on_chane_mode))) {
            _release(dev);
            return error;
        }
        /* set burst data rate */
        if ((error = _write_register_bits(dev, MLX90393_REG_CONF1, MLX90393_MASK_BDR, DEV_ODR)) != 0) {
            _release(dev);
            return error;
        }
        /* set tresholds */
        float gain = _get_gain_factor(DEV_GAIN);
        uint16_t raw_xy_threshold = dev->params.treshold.xy / (MLX90393_XY_SENS * (1 << DEV_RESOLUTION) * gain);
        uint16_t raw_z_threshold = dev->params.treshold.z / (MLX90393_Z_SENS * (1 << DEV_RESOLUTION) * gain);
        uint16_t raw_temp_threshold = dev->params.treshold.temp * MLX90393_TEMP_RESOLUTION / 100;

        if ((error = _write_register(dev, MLX90393_REG_WOXY_THRESHOLD, raw_xy_threshold))) {
            _release(dev);
            return error;
        }
        if ((error = _write_register(dev, MLX90393_REG_WOZ_THRESHOLD, raw_z_threshold))) {
            _release(dev);
            return error;
        }
        if ((error = _write_register(dev, MLX90393_REG_WOT_THRESHOLD, raw_temp_threshold))) {
            _release(dev);
            return error;
        }
        /* set wake up on change mode */
        if ((error = _write_byte(dev, MLX90393_COMMAND_SW)) != 0) {
            _release(dev);
            return error;
        }
        if((error = _check_status_byte(dev)) != 0) {
            _release(dev);
            return error;
        }
    }

    _release(dev);
    return MLX90393_SUCCESS;
}

static void _isr(void *lock)
{
    mutex_unlock(lock);
}

int mlx90393_read_measurement(mlx90393_t *dev, mlx90393_3d_data_t *data) 
{
    _acquire(dev);
    int error = 0;

    /* start single measurement */
    if (DEV_MODE == MLX90393_MODE_SINGLE_MEASUREMENT) {
        if ((error = _write_byte(dev, MLX90393_COMMAND_SM)) != 0) {
            _release(dev);
            return error;
        }
        if ((error = _check_status_byte(dev)) != 0) {
            return error;
        }
    }
    _release(dev);

    /* wait for interrupt if used */
    if (gpio_is_valid(DEV_INT_PIN)) {
        mutex_t lock = MUTEX_INIT_LOCKED;
        gpio_init_int(DEV_INT_PIN, GPIO_IN_PU, GPIO_RISING, _isr, &lock);
        mutex_lock(&lock);
        gpio_irq_disable(DEV_INT_PIN);
    }

    _acquire(dev);

    /* read measurement */
    if ((error = _write_byte(dev, MLX90393_COMMAND_RM)) != 0) {
        _release(dev);
        return error;
    }
    /* check status byte */
    uint8_t buffer[9];
    if ((error = _read_bytes(dev, buffer, 9)) != 0) {
        _release(dev);
        return error;
    }
    if (buffer[0] & MLX90393_STATUS_ERROR) {
        _release(dev);
        return MLX90393_ERROR;
    }
    /* convert read data */
    int16_t raw_x, raw_y, raw_z;
    uint16_t raw_temp;
    raw_temp = (uint16_t)((buffer[1] << 8) | buffer[2]);
    raw_x = (int16_t)((buffer[3] << 8) | buffer[4]);
    raw_y = (int16_t)((buffer[5] << 8) | buffer[6]);
    raw_z = (int16_t)((buffer[7] << 8) | buffer[8]);

    data->temp = _calculate_temp(raw_temp, dev->ref_temp);
    float gain = _get_gain_factor(DEV_GAIN);
    data->x_axis = (int)raw_x * gain * MLX90393_XY_SENS * (1 << DEV_RESOLUTION);
    data->y_axis = (int)raw_y * gain * MLX90393_XY_SENS * (1 << DEV_RESOLUTION);
    data->z_axis = (int)raw_z * gain * MLX90393_Z_SENS * (1 << DEV_RESOLUTION);

    _release(dev);
    return MLX90393_SUCCESS;
}

int mlx90393_reset(mlx90393_t *dev)
{
    _acquire(dev);

    int error = 0;
    if ((error = _write_byte(dev, MLX90393_COMMAND_RT)) != 0) {
        _release(dev);
        return error;
    }
    return _check_status_byte(dev);
}

