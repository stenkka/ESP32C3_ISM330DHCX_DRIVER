/****************************************************************************
 * include/nuttx/sensors/ism330dhcx.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_ISM330DHCX_H
#define __INCLUDE_NUTTX_SENSORS_ISM330DHCX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_ISM330DHCX)

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/
/* number of records in driver's fifo record buffer*/
#define FIFO_RECORD_BUFFER_SAMPLES 32
/* ISM330DHCX Register Definitions *********************************************/

#define ISM330DHCX_WHO_AM_I_REG    (0x0F)
#define ISM330DHCX_CTRL_REG_1      (0x20)
#define ISM330DHCX_CTRL_REG_2      (0x21)
#define ISM330DHCX_CTRL_REG_3      (0x22)
#define ISM330DHCX_CTRL_REG_4      (0x23)
#define ISM330DHCX_CTRL_REG_5      (0x24)
#define ISM330DHCX_STATUS_REG      (0x27)
#define ISM330DHCX_OUT_X_L_REG     (0x28)
#define ISM330DHCX_OUT_X_H_REG     (0x29)
#define ISM330DHCX_OUT_Y_L_REG     (0x2A)
#define ISM330DHCX_OUT_Y_H_REG     (0x2B)
#define ISM330DHCX_OUT_Z_L_REG     (0x2C)
#define ISM330DHCX_OUT_Z_H_REG     (0x2D)
#define ISM330DHCX_TEMP_OUT_L_REG  (0x2E)
#define ISM330DHCX_TEMP_OUT_H_REG  (0x2F)
#define ISM330DHCX_INT_CFG_REG     (0x30)
#define ISM330DHCX_INT_SRC_REG     (0x31)
#define ISM330DHCX_INT_THS_L_REG   (0x32)
#define ISM330DHCX_INT_THS_H_REG   (0x33)

/* ISM330DHCX CTRL_REG_1 Bit Definitions ***************************************/

#define ISM330DHCX_CTRL_REG_1_TEMP_EN_BM      (1<<7)   /* Enable the temperature sensor */
#define ISM330DHCX_CTRL_REG_1_OM_1_BM         (1<<6)   /* Select the operating mode of X and Y axis bit 1 */
#define ISM330DHCX_CTRL_REG_1_OM_0_BM         (1<<5)   /* Select the operating mode of X and Y axis bit 0 */
#define ISM330DHCX_CTRL_REG_1_DO2_BM          (1<<4)   /* Output data rate selection bit 2 */
#define ISM330DHCX_CTRL_REG_1_DO1_BM          (1<<3)   /* Output data rate selection bit 1 */
#define ISM330DHCX_CTRL_REG_1_DO0_BM          (1<<2)   /* Output data rate selection bit 2 */
#define ISM330DHCX_CTRL_REG_1_FAST_ODR_BM     (1<<1)   /* Enable higher output data rates */

/* ISM330DHCX CTRL_REG_2 Bit Definitions ***************************************/

#define ISM330DHCX_CTRL_REG_2_FS_1_BM         (1<<6)   /* Full scale selection bit 1 */
#define ISM330DHCX_CTRL_REG_2_FS_0_BM         (1<<5)   /* Full scale selection bit 0 */
#define ISM330DHCX_CTRL_REG_2_REBOOT_BM       (1<<3)   /* Reboot Memory Content */
#define ISM330DHCX_CTRL_REG_2_SOFT_RST_BM     (1<<2)   /* Soft Reset */

/* ISM330DHCX CTRL_REG_4 Bit Definitions ***************************************/

#define ISM330DHCX_CTRL_REG_4_OMZ_1_BM        (1<<3)   /* Select the operating mode of Z axis bit 1 */
#define ISM330DHCX_CTRL_REG_4_OMZ_0_BM        (1<<2)   /* Select the operating mode of Z axis bit 0 */

/* ISM330DHCX CTRL_REG_5 Bit Definitions ***************************************/

#define ISM330DHCX_CTRL_REG_5_BDU_BM          (1<<6)   /* Enable block data update for magnetic data (prevent race conditions while reading) */

/* SPI BUS PARAMETERS *******************************************************/

#define ISM330DHCX_SPI_FREQUENCY    (1000000)        /* 1 MHz */
#define ISM330DHCX_SPI_MODE         (SPIDEV_MODE0)   /* Device uses SPI Mode 3: CPOL=0, CPHA=0 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ISM330DHCX
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct ism330dhcx_config_s
{
  /* Since multiple ISM330DHCX can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired ISM330DHCX chip via their chip select inputs.
   */

  int spi_devid;

  /* The IRQ number must be provided for each so ISM330DHCX device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the ISM330DHCX interrupt handler to the GPIO interrupt of the
   * concrete ISM330DHCX instance.
   */

  int (*attach)(FAR struct ism330dhcx_config_s *, xcpt_t);
};


struct ism330dhcx_fifo_record_t
{
	uint8_t TAG;
	uint16_t DATA_X;
	uint16_t DATA_Y;
	uint16_t DATA_Z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ism330dhcx_register
 *
 * Description:
 *   Register the ISM330DHCX character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/acc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             ISM330DHCX
 *   config  - configuration for the ISM330DHCX driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ism330dhcx_register(
  FAR const char *devpath, 
  FAR struct spi_dev_s *spi);
  
int ism330dhcx_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_ISM330DHCX */
#endif /* __INCLUDE_NUTTX_SENSORS_ISM330DHCX_H */

