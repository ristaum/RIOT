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
 * @brief       Internal addresses, registers and constants
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 */

#ifndef MLX90393_CONSTANTS_H
#define MLX90393_CONSTANTS_H

#ifdef __cplusplus
extern "C" //{
#endif

/**
 * Register Map
 */
#define MLX90393_REG_CONF0          0x00
#define MLX90393_REG_CONF1          0x01
#define MLX90393_REG_CONF2          0x02
#define MLX90393_REG_SENS_TC        0x03
#define MLX90393_REG_OFFSET_X       0x04
#define MLX90393_REG_OFFSET_Y       0x05
#define MLX90393_REG_OFFSET_Z       0x06
#define MLX90393_REG_WOXY_THRESHOLD 0x07
#define MLX90393_REG_WOZ_THRESHOLD  0x08
#define MLX90393_REG_WOT_THRESHOLD  0x09
#define MLX90393_REG_REF_TEMP       0x24

/**
 * Command Set
 */
#define MLX90393_COMMAND_SB         0x1F
#define MLX90393_COMMAND_SW         0x2F
#define MLX90393_COMMAND_SM         0x3F
#define MLX90393_COMMAND_RM         0x4F
#define MLX90393_COMMAND_RR         0x50
#define MLX90393_COMMAND_WR         0x60
#define MLX90393_COMMAND_EX         0x80
#define MLX90393_COMMAND_HR         0xD0
#define MLX90393_COMMAND_HS         0xE0
#define MLX90393_COMMAND_RT         0xF0

/**
 * @brief status byte bit map
 * 
 */
#define MLX90393_STATUS_RESET       0x04
#define MLX90393_STATUS_ERROR       0x10

/**
 * @brief timeout durations in us
 * 
 */
#define MLX90393_COMMAND_EX_TIMEOUT     1000
#define MLX90393_COMMAND_RT_TIMEOUT     1500

/**
 * @brief configuration bit masks
 * 
 */
#define MLX90393_MASK_BDR           0x3F
#define MLX90393_MASK_WOC_DIFF      0x1000
#define MLX90393_MASK_TCMP_EN       0x400
#define MLX90393_MASK_GAIN_SEL      0x70
#define MLX90393_MASK_RES_XYZ       0x7E0

/**
 * @brief 
 * 
 */
#define MLX90393_TEMP_OFFSET        35
#define MLX90393_TEMP_RESOLUTION    45.2
#define MLX90393_XY_SENS            0.15
#define MLX90393_Z_SENS             0.242

#ifdef __cplusplus
//}
#endif

#endif /* MLX90393_CONSTANTS_H */
/** @} */
