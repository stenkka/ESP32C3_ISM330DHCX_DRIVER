/****************************************************************************
 * drivers/sensors/ism330dhcx.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/ism330dhcx.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_ISM330DHCX)

/****************************************************************************
 * Private
 ****************************************************************************/

struct ism330dhcx_sensor_data_s
{
  int16_t x_acc;        /* Measurement result for x axis accelerometer */
  int16_t y_acc;        /* Measurement result for y axis accelerometer */
  int16_t z_acc;        /* Measurement result for z axis accelerometer */
  int16_t x_gyro;        /* Measurement result for x axis gyro */
  int16_t y_gyro;        /* Measurement result for y axis gyro */
  int16_t z_gyro;        /* Measurement result for z axis gyro */
  int16_t temperature;  /* Measurement result for temperature sensor */
};

struct ism330dhcx_dev_s
{
  FAR struct ism330dhcx_dev_s *flink;  /* Supports a singly linked list of
                                        * drivers */
  FAR struct spi_dev_s *spi;           /* Pointer to the SPI instance */
  FAR struct ism330dhcx_config_s *config;  /* Pointer to the configuration
                                            * of the ISM330DHCX sensor */
  sem_t datasem;                       /* Manages exclusive access to this
                                        * structure */
  struct ism330dhcx_sensor_data_s data;  /* The data as measured by the sensor */
  struct work_s work;                  /* The work queue is responsible for
                                        * retrieving the data from the
                                        * sensor after the arrival of new
                                        * data was signalled in an interrupt */
};

struct ism330dhcx_sensor_data_ring_buffer
{
	int16_t* data;
	uint8_t index;
	uint8_t num;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ism330dhcx_read_register(
  FAR struct ism330dhcx_dev_s *dev,
  uint8_t const reg_addr,
  uint8_t * reg_data);
static void ism330dhcx_write_register(
  FAR struct ism330dhcx_dev_s *dev,
  uint8_t const reg_addr,
  uint8_t const reg_data);
static void ism330dhcx_reset(
  FAR struct ism330dhcx_dev_s *dev);
static void ism330dhcx_read_measurement_data(
  FAR struct ism330dhcx_dev_s *dev);
static void ism330dhcx_read_acc_data(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * x_acc, uint16_t * y_acc,
  uint16_t * z_acc);
static void ism330dhcx_read_gyro_data(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * x_gyro, uint16_t * y_gyro,
  uint16_t * z_gyro);

static void ism330dhcx_read_temperature(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * temperature);
static int ism330dhcx_interrupt_handler(
  int irq, FAR void *context);
static void ism330dhcx_worker(
  FAR void *arg);

static int ism330dhcx_open(
  FAR struct file *filep);
static int ism330dhcx_close(
  FAR struct file *filep);

static ssize_t ism330dhcx_read(
  FAR struct file *, 
  FAR char *, 
  size_t);
static ssize_t ism330dhcx_write(
  FAR struct file *filep,
  FAR const char *buffer,
  size_t buflen);
static int ism330dhcx_ioctl(
  FAR struct file *filep,
  int cmd, 
  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ism330dhcx_fops =
{
  ism330dhcx_open,    /* open */
  ism330dhcx_close,   /* close */
  ism330dhcx_read,    /* read */
  ism330dhcx_write,   /* write */
  NULL,               /* seek */
  ism330dhcx_ioctl,   /* ioctl */
  NULL                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};

/* Single linked list to store instances of drivers */

static struct ism330dhcx_dev_s *g_ism330dhcx_list = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ism330dhcx_read_register
 ****************************************************************************/

static void ism330dhcx_read_register(
  FAR struct ism330dhcx_dev_s *dev,
  uint8_t const reg_addr, 
  uint8_t * reg_data)
{
  /* Lock the SPI bus so that only one device 
   * can access it at the same time
   */
  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read 
   * the MSB needs to be set to indicate the read indication.
   */
  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */
  *reg_data = (uint8_t) (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: ism330dhcx_write_register
 ****************************************************************************/

static void ism330dhcx_write_register(
  FAR struct ism330dhcx_dev_s *dev,
  uint8_t const reg_addr,
  uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device 
   * can access it at the same time
   */
  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */
  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */
  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: ism330dhcx_reset
 ****************************************************************************/

static void ism330dhcx_reset(FAR struct ism330dhcx_dev_s *dev)
{
  sninfo("RESETTING SW\n");
  ism330dhcx_write_register(
    dev,
    0x12,
    0x05); // IF_INC | SW_RESET

  up_mdelay(100);
}

/****************************************************************************
 * Name: ism330dhcx_interrupt_handler
 ****************************************************************************/

static void ism330dhcx_read_measurement_data(
  FAR struct ism330dhcx_dev_s *dev)
{
  /* Magnetic data */
  uint16_t x_acc = 0;
  uint16_t y_acc = 0;
  uint16_t z_acc = 0;
  ism330dhcx_read_acc_data(dev, &x_acc, &y_acc, &z_acc);

  /* Gyro data */
  uint16_t x_gyro = 0;
  uint16_t y_gyro = 0;
  uint16_t z_gyro = 0;
  ism330dhcx_read_gyro_data(dev, &x_gyro, &y_gyro, &z_gyro);


  /* Temperature */
  uint16_t temperature = 0;
  ism330dhcx_read_temperature(dev, &temperature);

  /* Acquire the semaphore before the data is copied */
  int ret = nxsem_wait(&dev->datasem);
  if (ret != OK) {
    snerr("ERROR: Could not acquire dev->datasem: %d\n", ret);
    return;
  }

  /* Copy retrieve data to internal data structure */
  dev->data.x_acc = (int16_t) (x_acc);
  dev->data.y_acc = (int16_t) (y_acc);
  dev->data.z_acc = (int16_t) (z_acc);
  dev->data.x_gyro = (int16_t) (x_gyro);
  dev->data.y_gyro = (int16_t) (y_gyro);
  dev->data.z_gyro = (int16_t) (z_gyro);

  dev->data.temperature = (int16_t) (temperature);

  /* Give back the semaphore */
  nxsem_post(&dev->datasem);

  /* Feed sensor data to entropy pool */
  //add_sensor_randomness(
  //  (x_acc << 16) ^ (y_acc << 10) ^ (z_acc << 2) ^ temperature);
}

/****************************************************************************
 * Name: ism330dhcx_read_gyro_data
 ****************************************************************************/

static void ism330dhcx_read_gyro_data(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * x_gyro, 
  uint16_t * y_gyro,
  uint16_t * z_gyro)
{
  /* Lock the SPI bus so that only one device 
   * can access it at the same time
   */
  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set 
   *   -> Read Indication 
   */
  // OUTX_L_G 0x22
  // OUTY_L_G 0x24
  // OUTZ_L_G 0x26

  SPI_SEND(dev->spi, (0x22 | 0x80)); /* RX */
  *x_gyro  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0); /* LSB */
  *x_gyro |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *y_gyro  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *y_gyro |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *z_gyro  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *z_gyro |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  /* Set CS to high which deselects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus*/
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: ism330dhcx_read_acc_data
 ****************************************************************************/

static void ism330dhcx_read_acc_data(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * x_acc, 
  uint16_t * y_acc,
  uint16_t * z_acc)
{
  /* Lock the SPI bus so that only one device 
   * can access it at the same time
   */
  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set 
   *   -> Read Indication 
   */
  // OUTX_L_A 0x28
  // OUTY_L_A 0x2A
  // OUTZ_L_A 0x2C
  SPI_SEND(dev->spi, (0x28 | 0x80)); /* RX */
  *x_acc  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *x_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *y_acc  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *y_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *z_acc  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *z_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  /* Set CS to high which deselects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: ism330dhcx_read_temperature
 ****************************************************************************/

static void ism330dhcx_read_temperature(
  FAR struct ism330dhcx_dev_s *dev,
  uint16_t * temperature)
{
  /* Lock the SPI bus so that only one device 
   * can access it at the same time
   */
  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set 
   *   -> Read Indication 
   */
  SPI_SEND(dev->spi, (0x20 | 0x80));

  /* RX */
  *temperature  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);    /* OUT_TEMP_L */
  *temperature |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);    /* OUT_TEMP_H */

  /* Set CS to high which deselects the ISM330DHCX */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: ism330dhcx_interrupt_handler
 ****************************************************************************/

static int ism330dhcx_interrupt_handler(int irq, FAR void *context)
{
  /* This function should be called upon a rising edge on the ISM330DHCX DRDY
   * pin since it signals that new data has been measured.
   */
  FAR struct ism330dhcx_dev_s *priv = 0;
  int ret;

  /* Find out which ISM330DHCX device caused the interrupt */
  for (priv = g_ism330dhcx_list; 
       priv && priv->config->irq != irq;
       priv = priv->flink);
  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */
  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, ism330dhcx_worker, priv, 0);
  if (ret < 0) {
    snerr("ERROR: Failed to queue work: %d\n", ret);
    return ret;
  }
  else {
    return OK;
  }
}

/****************************************************************************
 * Name: ism330dhcx_worker
 ****************************************************************************/

static void ism330dhcx_worker(FAR void *arg)
{
  FAR struct ism330dhcx_dev_s *priv = (FAR struct ism330dhcx_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */
  ism330dhcx_read_measurement_data(priv);
}

/****************************************************************************
 * Name: ism330dhcx_open
 ****************************************************************************/

static int ism330dhcx_open(FAR struct file *filep)
{
	sninfo("OPENING...\n");

	FAR struct inode *inode = filep->f_inode;
  	FAR struct ism330dhcx_dev_s *priv = inode->i_private;
  	uint8_t reg_content;

	ism330dhcx_read_register(priv, 0x19, &reg_content);
	sninfo("REG = %04x\n", reg_content);

  /* ConfigA
  Ac 01 00  FUNC_CFG_ACCESS (FUNC_CFG_ACCESS=0, SHUB_REG_ACCESS=0)
  Ac 02 3F  PIN_CTRL (OIS_PU_DIS=0 OCS_Aux and SDO_Aux pins with pull-up;)
                     (SDO_PU_EN=0 SDO pin pull-up disconnected (default))
  Ac 07 FF  FIFO_CTRL1 ()
  Ac 08 00  FIFO_CTRL2 ()
          FIFO watermark threshold = 2^8
          STOP_ON_WTM = 0: FIFO depth is not limited (default);
          FIFO_COMPR_RT_EN = 0: Disables compression algorithm runtime
          ODRCHG_EN = 0: Disable virtual sensor to be batched in FIFO
          UNCOPTR_RATE = 0: Non-compressed data writing is not forced;
  Ac 09 77  FIFO_CTRL3
          0b0111(BDR_GY_) 0111(BDR_XL_)
          BDR_GY_ = 833 Hz
          BDR_XL_ = 833 Hz
  Ac 0A 56 FIFO_CTRL4 ()
          0b01(DEC_TS_BATCH_) 01(ODR_T_BATCH_) 0() 110(FIFO_MODE)
          DEC_TS_BATCH_ = Selects decimation for timestamp batching 
          in FIFO. Write rate will be the maximum rate between XL 
          and GYRO BDR divided by decimation decoder.
          Decimation 1: max(BDR_XL[Hz],BDR_GY[Hz]) [Hz];
          ODR_T_BATCH_ = Selects batch data rate (write frequency in FIFO) 
          for temperature data
          01: 1.6 Hz;
          FIFO_MODE 110: Continuous mode: 
          if the FIFO is full, the new sample overwrites the older one;
  Ac 0B 00 COUNTER_BDR_REG1
  Ac 0C 00 COUNTER_BDR_REG2
  Ac 0D 00 INT1_CTRL
          DEN_DRDY_flag Sends DEN_DRDY (DEN stamped on Sensor Data flag) to INT1 pin.
          INT1_CNT_BDR Enables COUNTER_BDR_IA interrupt on INT1.
          INT1_FIFO_FULL Enables FIFO full flag interrupt on INT1 pin.
          INT1_FIFO_OVR Enables FIFO overrun interrupt on INT1 pin.
          INT1_FIFO_TH Enables FIFO threshold interrupt on INT1 pin.
          INT1_BOOT Enables boot status on INT1 pin.
          INT1_DRDY_G Enables gyroscope data-ready interrupt on INT1 pin.
          INT1_DRDY_XL Enables accelerometer data-ready interrupt on INT1 pin.
  Ac 0E 00 INT2_CTRL
          0
          NT2_CNT_BDR Enables COUNTER_BDR_IA interrupt on INT2 pin.
          INT2_FIFO_FULL Enables FIFO full flag interrupt on INT2 pin.
          INT2_FIFO_OVR Enables FIFO overrun interrupt on INT2 pin.
          INT_FIFO_TH Enables FIFO threshold interrupt on INT2 pin.
          INT2_DRDY_TEMP Enables temperature sensor data-ready interrupt on INT2 pin.
          INT2_DRDY_G Enables gyroscope data-ready interrupt on INT2 pin.
          INT2_DRDY_XL Enables accelerometer data-ready interrupt on INT2 pin.
  WHO_AM_I (0Fh)
    WHO_AM_I register (r). 
    This is a read-only register. Its value is fixed at 6Bh
  Ac 10 70  CTRL1_XL Accelerometer control register 1 (r/w)
          0b0111(ODR_XL) 00(FS_XL) 0(LPF2_XL_EN) 0()
          833 Hz (high performance) ±2 g
          output from first stage digital filtering selected (default);
  Ac 11 70  CTRL2_G  Gyroscope control register 2 (r/w)
          0b0111(ODR_G) 00(FS_G) 0(FS_125) 0(FS_4000)
          833 Hz (high performance) ±250 dps;
          FS selected through bits FS_G;
  Ac 12 04  CTRL3_C Control register 3 (r/w)
          0b0(BOOT) 0(BDU) 0(H_LACTIVE) 0(PP_OD) 0(SIM) 1(IF_INC) 0() 0(SW_RESET)
          normal mode
          continuous update
          interrupt output pins active high;
          INT1/INT2 push-pull mode
          4-wire interface
          Register address automatically incremented during a multiple byte 
          access with a serial interface
  Ac 13 00
  Ac 14 00
  Ac 15 00
  Ac 16 00 high-performance operating mode enabled
  Ac 17 00 ODR/2 
  Ac 18 E2 CTRL9_XL = 0b1110 0010
      DEN_X = 1 DEN stored in X-axis LSB
      DEN_Y = 1 DEN stored in Y-axis LSB
      DEN_Z = 1 DEN stored in Z-axis LSB
      DEN_XL_G = 0 DEN pin info stamped in the gyroscope axis selected by bits [7:5]
      DEN_XL_EN = 0 Extends DEN functionality to accelerometer sensor. (disabled)
      DEN_LH = DEN active level configuration. active high
  Ac 19 20 CTRL10_C = 0b0010 0000 (Enables timestamp counter.)
  Ac 56 00 
  Ac 57 00
  Ac 58 00
  Ac 59 00
  Ac 5A 00
  Ac 5B 00
  Ac 5C 00
  Ac 5D 00
  Ac 5E 00
  Ac 5F 00
  Ac 73 00
  Ac 74 00
  Ac 75 00
  */


  /* Read back the content of all control registers for debug purposes */
  /*
  reg_content = 0;
  for (reg_addr = ISM330DHCX_CTRL_REG_1;
       reg_addr <= ISM330DHCX_CTRL_REG_5;
       reg_addr++)
    {
      ism330dhcx_read_register(priv, reg_addr, &reg_content);
      sninfo("R#%04x = %04x\n", reg_addr, reg_content);
    }

  ism330dhcx_read_register(priv, ISM330DHCX_STATUS_REG, &reg_content);
  sninfo("STATUS_REG = %04x\n", reg_content);
 */
 	
	
	DEBUGASSERT(priv != NULL);

	if (!priv)
	{
		snerr("ERROR: Failed to open driver.");
		return -ENODEV;
	}

  	reg_content = 0;
  	ism330dhcx_read_register(priv, 0x0F, &reg_content);
  	spiinfo("STATUS_REG = %04x (expected 6B)\n", reg_content);

	return OK;
}

/****************************************************************************
 * Name: ism330dhcx_close
 ****************************************************************************/

static int ism330dhcx_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ism330dhcx_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */
  //ism330dhcx_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: ism330dhcx_read
 ****************************************************************************/

static ssize_t ism330dhcx_read(
  FAR struct file *filep, 
  FAR char *buffer,
  size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ism330dhcx_dev_s *priv = inode->i_private;
  FAR struct ism330dhcx_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(FAR struct ism330dhcx_sensor_data_s))
    {
      snerr("ERROR: "
            "Not enough memory for reading out a sensor data sample\n");
      return -ENOSYS;
    }

  ism330dhcx_read_measurement_data(priv);

  /* Acquire the semaphore before the data is copied */

  ret = nxsem_wait(&priv->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->datasem: %d\n", ret);
      return ret;
    }

  /* Copy the sensor data into the buffer */

  data = (FAR struct ism330dhcx_sensor_data_s *)buffer;
  memset(data, 0, sizeof(FAR struct ism330dhcx_sensor_data_s));

  data->x_acc = priv->data.x_acc;
  data->y_acc = priv->data.y_acc;
  data->z_acc = priv->data.z_acc;
  data->x_gyro = priv->data.x_gyro;
  data->y_gyro = priv->data.y_gyro;
  data->z_gyro = priv->data.z_gyro;
  data->temperature = priv->data.temperature;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(FAR struct ism330dhcx_sensor_data_s);
}

/****************************************************************************
 * Name: ism330dhcx_write
 ****************************************************************************/

static ssize_t ism330dhcx_write(
  FAR struct file *filep, 
  FAR const char *buffer,
  size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ism330dhcx_ioctl
 ****************************************************************************/

static int ism330dhcx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	int ret = OK;
	uint8_t read_data = 0;

  	switch (cmd)
    	{
	case SNIOC_SET_ACC_LOWPERF:
		ism330dhcx_read_register(g_ism330dhcx_list, 0x15, &read_data);
		switch (arg)
		{
		case 1:
			ism330dhcx_write_register(g_ism330dhcx_list, 0x15, 0x10 | read_data);
			break;
		case 0:
			ism330dhcx_write_register(g_ism330dhcx_list, 0x15, ~0x10 & read_data);
			break;
		default:
			snerr("ERROR: Unrecognized arg: %d\n", arg);
			ret = -ENOTTY;
			break;
		}
		break;
	// Accelerometer ODR (sampling rate)
	case SNIOC_SET_ACC_ODR:
		switch (arg)
		{
		case 1:
			ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0xB0);
			sninfo("ioctl: Accelerometer ODR: 1.6Hz\n");
			break;
		case 52:
			sninfo("ioctl: Accelerometer ODR: 52Hz\n");
			ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0x30);
			break;
		case 833:
			sninfo("ioctl: Accelerometer ODR: 833Hz\n");
			ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0x70);
			break;
		case 6660:
			sninfo("ioctl: Accelerometer ODR: 6.66kHz\n");
			ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0xA0);
			break;
		default:
			snerr("ERROR: Unrecognized arg: %d\n", arg);
			ret = -ENOTTY;
			break;
		}
		break;

	case SNIOC_SET_GYRO_LOWPERF:     
                ism330dhcx_read_register(g_ism330dhcx_list, 0x16, &read_data);
				switch (arg)
                {
                case 1:
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x16, 0x80 | read_data);
                        break;
                case 0:
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x16, ~0x80 & read_data);
                        break;
                default:
                        snerr("ERROR: Unrecognized arg: %d\n", arg);
                        ret = -ENOTTY;
                        break;
                }
                break;


		// Gyroscope ODR (sampling rate)
        case SNIOC_SET_GYRO_ODR:
                switch (arg)
                {
                case 52:  
			sninfo("ioctl: Accelerometer ODR: 52Hz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0x30);
                        break;
                case 833:
                        sninfo("ioctl: Accelerometer ODR: 833Hz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0x70);
                        break;
                case 6660:
                        sninfo("ioctl: Accelerometer ODR: 6.66kHz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0xA0);
                        break;
                default:
                        snerr("ERROR: Unrecognized arg: %d\n", arg);
                        ret = -ENOTTY;
                        break;
                }
                break;
		
		//Fifo watermark treshold
	case SNIOC_SET_FIFO_WM:
		switch (arg)
		{
                case 5:
                        sninfo("ioctl: FIFO treshold: 5\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x07, 0x1F);
                        break;
                case 6:
                        sninfo("ioctl: FIFO treshold: 6\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x07, 0x3F);
                        break;
                case 7:
                        sninfo("ioctl: FIFO treshold: 7\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x07, 0x4F);
                        break;
                default:
                        snerr("ERROR: Unrecognized arg: %d\n", arg);
                        ret = -ENOTTY;
                        break;
                }
                break;
		
		//Fifo batch rate (gyro & accelerometer)
		
	case SNIOC_SET_FIFO_BATCH_RATE:
                switch (arg)
                {
		case 0:
                        sninfo("ioctl: FIFO is not used\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x09, 0x00);
                        break;
                case 12:
                        sninfo("ioctl: FIFO BATCH_RATE: 12.5Hz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x09, 0x11);
                        break;
                case 52:
                        sninfo("ioctl: FIFO BATCH_RATE: 52Hz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x09, 0x33);
                        break;
                case 833:
                        sninfo("ioctl: FIFO BATCH_RATE: 833Hz\n");
                        ism330dhcx_write_register(g_ism330dhcx_list, 0x09, 0x77);
                        break;
                default:
                        snerr("ERROR: Unrecognized arg: %d\n", arg);
                        ret = -ENOTTY;
                        break;
                }
                break;

	
	/* Command was not recognized */
    default:
    	snerr("ERROR: Unrecognized cmd: %d\n", cmd);
    	ret = -ENOTTY;
    	break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  FAR struct spi_dev_s *spi)
{
  FAR struct ism330dhcx_config_s *config;
  FAR struct ism330dhcx_dev_s *priv;
  int ret;

  /* Sanity check */
  DEBUGASSERT(spi != NULL);

  /* Initialize the ISM330DHCX device config structure */
  config = (FAR struct ism330dhcx_config_s *)
          kmm_malloc(sizeof(struct ism330dhcx_config_s));
  config->spi_devid = 2;
  config->irq = 0;
  config->attach = NULL;

  /* Initialize the ISM330DHCX device structure */
  priv = (FAR struct ism330dhcx_dev_s *)
          kmm_malloc(sizeof(struct ism330dhcx_dev_s));
  if (priv == NULL) {
    snerr("ERROR: Failed to allocate instance\n");
    return -ENOMEM;
  }

  priv->spi = spi;
  priv->config = config;
  priv->work.worker = NULL;

  nxsem_init(&priv->datasem, 0, 1);  /* Initialize sensor data access
                                      * semaphore */

  /* Setup SPI frequency and mode */
  SPI_SETMODE(spi, ISM330DHCX_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, ISM330DHCX_SPI_FREQUENCY);

  /* Attach the interrupt handler */
  //ret = priv->config->attach(priv->config, &ism330dhcx_interrupt_handler);
  //if (ret < 0) {
  //  snerr("ERROR: Failed to attach interrupt\n");
  //  return -ENODEV;
  //}

  /* Register the character driver */
  ret = register_driver(devpath, &g_ism330dhcx_fops, 0666, priv);
  if (ret < 0) {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    kmm_free(priv);
    nxsem_destroy(&priv->datasem);
    return -ENODEV;
  }

  /* Since we support multiple ISM330DHCX devices are supported, we will need to
   * add this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */
  priv->flink = g_ism330dhcx_list;
  g_ism330dhcx_list = priv;

  return OK;
}
/************************************************************    ****************
  * Name: ism330dhcx_init
  *
  * Description:
  *   Initialize the ISM330DHCX with default configuration
  *
  * Input Parameters:
  *
  * Returned Value:
  *   Zero (OK) on success; a negated errno value on failure.
*************************************************************    ****************/
int ism330dhcx_init()
{
	//struct ism330dhcx_dev_s* priv = g_ism330dhcx_list;

	//sninfo("spidevid: %d\n", priv->config->spi_devid);

	// return -1 on memory allocation error
	DEBUGASSERT(g_ism330dhcx_list != NULL);
	if (!g_ism330dhcx_list)
	{
		return -1;
	}
	
	/* Perform a reset */
 	ism330dhcx_reset(g_ism330dhcx_list);

	sninfo("Configuration:\n");

  	ism330dhcx_write_register(g_ism330dhcx_list, 0x01, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x02, 0x3F);
	// FIFO treshold: 6 slots (sensor data + TAG)
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x07, 0x3F);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x08, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x09, 0x77);
  	// Enable continuous-FIFO mode (FIFO data is rewritten as new samples come in)
	ism330dhcx_write_register(g_ism330dhcx_list, 0x0A, 0x56);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x0B, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x0C, 0x00);
	// Enable FIFO threshold interrupt on pin INT1
	ism330dhcx_write_register(g_ism330dhcx_list, 0x0D, 0x08);
	ism330dhcx_write_register(g_ism330dhcx_list, 0x0E, 0x00);
  	
	// Accelerometer ODR (sampling rate)	

	#ifdef CONFIG_ISM330DHCX_ACC_ODR__1_6
	ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0xB0);
	sninfo("Accelerometer ODR: 1.6Hz\n");
	#endif

	#ifdef CONFIG_ISM330DHCX_ACC_ODR__52
	sninfo("Accelerometer ODR: 52Hz\n");
	ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0x30);
	#endif

	#ifdef CONFIG_ISM330DHCX_ACC_ODR__833
	sninfo("Accelerometer ODR: 833Hz\n");
	ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0x70);
	#endif

	#ifdef CONFIG_ISM330DHCX_ACC_ODR__6660
	sninfo("Accelerometer ODR: 6.66kHz\n");
	ism330dhcx_write_register(g_ism330dhcx_list, 0x10, 0xA0);
	#endif

	//Gyroscope ODR (sampling rate)

   	#ifdef CONFIG_ISM330DHCX_GYRO_ODR__52
	sninfo("Gyroscope ODR: 52Hz\n");
    ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0x30);
	#endif

	#ifdef CONFIG_ISM330DHCX_GYRO_ODR__833
	sninfo("Gyroscope ODR: 833Hz\n");
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0x70);
   	#endif

  	#ifdef CONFIG_ISM330DHCX_GYRO_ODR__6660
  	sninfo("Gyroscope ODR: 6.66kHz\n");
   	ism330dhcx_write_register(g_ism330dhcx_list, 0x11, 0xA0);
   	#endif



  	ism330dhcx_write_register(g_ism330dhcx_list, 0x12, 0x04);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x13, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x14, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x15, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x16, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x17, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x18, 0xE2);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x19, 0x20);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x56, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x57, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x58, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x59, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5A, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5B, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5C, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5D, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5E, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x5F, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x73, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x74, 0x00);
  	ism330dhcx_write_register(g_ism330dhcx_list, 0x75, 0x00);

	sninfo("ism330dhcx initialized\n");

	return 0;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_ISM330DHCX */

