#include <nuttx/config.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
//#include <nuttx/sensors/ism330dhcx.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/ioexpander/gpio.h>


struct sensor_data
{
  int16_t x_acc;        /* Measurement result for x axis */
  int16_t y_acc;        /* Measurement result for y axis */
  int16_t z_acc;        /* Measurement result for z axis */
  int16_t x_gyro;        /* Measurement result for x axis */
  int16_t y_gyro;        /* Measurement result for y axis */
  int16_t z_gyro;        /* Measurement result for z axis */
  int16_t temperature;  /* Measurement result for temperature sensor */
};

struct ism330dhcx_fifo_record_t
{
	uint8_t TAG;
	uint16_t DATA_X;
	uint16_t DATA_Y;
	uint16_t DATA_Z;
};

int main(int argc, FAR char *argv[])
{
  	printf("Running MEMS app\n");
	
	int ret, fd, ret2;
	int c = 0;
	//struct sensor_data sample;
    struct ism330dhcx_fifo_record_t sample[3];	
	fd = open("/dev/ism330dhcx", O_RDONLY);
    int fd2 = open("/dev/gpio2", O_RDWR);
    bool invalue;
	while(1)
	{
        
        ret2 = ioctl(fd2, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
              if (ret2 > -1) {printf("INT PING VALUE: %d\n", invalue);}
		c++;
		ret = read(fd, &sample, sizeof(struct ism330dhcx_fifo_record_t) * 3);
		if(ret != sizeof(struct ism330dhcx_fifo_record_t) * 3)
		{
            printf("Returned size wrong\n");
			break;
		}
		printf("x_acc: %12d\n", sample[0].DATA_X);
		usleep(200000);
		if (c == 15)
		{	
            /*
			// Set Acc to Lowperformance mode
			ioctl(fd, SNIOC_SET_ACC_LOWPERF, 1);
			// Set Acc ODR to 1.6Hz
			ioctl(fd, SNIOC_SET_ACC_ODR, 1);
			// Set Gyro to Lowperformance mode
			ioctl(fd, SNIOC_SET_GYRO_LOWPERF, 1);
			// Set Gyro ODR to 52.0Hz
			ioctl(fd, SNIOC_SET_GYRO_ODR, 52);
			// Set Fifo watermark treshold to 6 slots (sensor data + TAG)
			ioctl(fd, SNIOC_SET_FIFO_WM, 6);
			// Set Gyro Fifo batch rate 52Hz
			ioctl(fd, SNIOC_SET_FIFO_BATCH_RATE_GYRO, 52);
	 	    // Set Acc Fifo batch rate 52Hz
            ioctl(fd, SNIOC_SET_FIFO_BATCH_RATE_ACC, 52);
			// Set Temp Fifo batch rate 52Hz
            ioctl(fd, SNIOC_SET_FIFO_BATCH_RATE_TEMP, 0);
            */



		}	
	}
	return 0;
}
