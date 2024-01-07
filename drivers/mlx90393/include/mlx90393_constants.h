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
 * @brief       MLX90393 nternal addresses, registers and constants
 *
 * @author      Michael Ristau <michael.ristau@fh-erfurt.de>
 */

#ifndef MLX90393_CONSTANTS_H
#define MLX90393_CONSTANTS_H

#ifdef __cplusplus
extern "C" //{
#endif

/**
 * @brief   Register address map
 */
#define MLX90393_REG_CONF0          0x00    /**< Configuration register 0 for several parameters **/
#define MLX90393_REG_CONF1          0x01    /**< Configuration register 1 for several parameters **/
#define MLX90393_REG_CONF2          0x02    /**< Configuration register 2 for several parameters **/
#define MLX90393_REG_SENS_TC        0x03    /**< Sensitivity drift compensation factor register **/
#define MLX90393_REG_OFFSET_X       0x04    /**< Offset register for x axis **/
#define MLX90393_REG_OFFSET_Y       0x05    /**< Offset register for y axis **/
#define MLX90393_REG_OFFSET_Z       0x06    /**< Offset register for z axis **/
#define MLX90393_REG_WOXY_THRESHOLD 0x07    /**< Wake up on change threshold register for x and y axis **/
#define MLX90393_REG_WOZ_THRESHOLD  0x08    /**< Wake up on change threshold register for z axis **/
#define MLX90393_REG_WOT_THRESHOLD  0x09    /**< Wake up on change threshold register for temp **/
#define MLX90393_REG_CONN_TEST      0x0A    /**< Free available register used for connectivity test **/
#define MLX90393_REG_REF_TEMP       0x24    /**< Reference temperature register **/

/**
 * @brief   Configuration parameter bit masks
 */
#define MLX90393_MASK_BDR           0x003F  /**< burst data rate **/
#define MLX90393_MASK_WOC_DIFF      0x1000  /**< wake up on change mode (relative or absolute) **/
#define MLX90393_MASK_TCMP_EN       0x0400  /**< enable temperature compensation **/
#define MLX90393_MASK_GAIN_SEL      0x0070  /**< analog chain gain **/
#define MLX90393_MASK_RES_XYZ       0x07E0  /**< xyz resolution **/
#define MLX90393_MASK_COMM_MODE     0x6000  /**< set allowed bus communication (I2C or SPI) **/

/**
 * @brief   Command Set
 */
#define MLX90393_COMMAND_SB         0x1F    /**< start burst mode **/
#define MLX90393_COMMAND_SW         0x2F    /**< start wake up on change mode **/
#define MLX90393_COMMAND_SM         0x3F    /**< start single measurement mode **/
#define MLX90393_COMMAND_RM         0x4F    /**< read measurement **/
#define MLX90393_COMMAND_RR         0x50    /**< read register **/
#define MLX90393_COMMAND_WR         0x60    /**< write register **/
#define MLX90393_COMMAND_EX         0x80    /**< exit mode **/
#define MLX90393_COMMAND_HR         0xD0    /**< memory recall **/
#define MLX90393_COMMAND_HS         0xE0    /**< memory store **/
#define MLX90393_COMMAND_RT         0xF0    /**< reset **/

/**
 * @brief   Status byte bit map
 * 
 */
#define MLX90393_STATUS_RESET       0x04    /**< reset bit **/
#define MLX90393_STATUS_ERROR       0x10    /**< error bit **/

/**
 * @brief   Timeout durations in us
 */
#define MLX90393_COMMAND_EX_TIMEOUT     1000    /**< Timeout after exit command **/
#define MLX90393_COMMAND_RT_TIMEOUT     1500    /**< Timeout after reset command **/

/**
 * @brief   Temperature conversion constants
 */
#define MLX90393_TEMP_OFFSET        3500    /**< Temperature offset (35 * 100 for avoiding float values) **/
#define MLX90393_TEMP_RESOLUTION    452     /**< Temperature sensor resolution (45.2 * 10 for avoiding float values) **/

/**
 * @brief   Magnetic flux conversion constants
 */
#define MLX90393_XY_SENS            150     /**< xy sensitivity in nT/LSB (0.15 * 1000 for avoiding float values) **/
#define MLX90393_Z_SENS             242     /**< z sensitivity in nT/LSB (0.242 * 1000 for avoiding float values) **/

#ifdef __cplusplus
//}
#endif

#endif /* MLX90393_CONSTANTS_H */
/** @} */
