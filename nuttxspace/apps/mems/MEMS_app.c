#include <nuttx/config.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <nuttx/sensors/ism330dhcx.h>
#include <nuttx/sensors/ioctl.h>


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

int main(int argc, FAR char *argv[])
{
  	printf("Running MEMS app\n");
	
	int ret, fd;
	int c = 0;
	struct sensor_data sample;	
	fd = open("/dev/ism330dhcx", O_RDONLY); 
	while(1)
	{
		c++;
		ret = read(fd, &sample, sizeof(sample));
		if(ret != sizeof(sample))
		{
			break;
		}
	printf("x_acc: %12d, y_acc: %12d\n, x_gyro: %12d, y_gyro: %12d\n", sample.x_acc, sample.y_acc, sample.x_gyro, sample.y_gyro);
	//printf("x_acc: %12d, %y_acc: %12d, z_acc: %12d, temp: %12d", sample.x_acc, sample.y_acc, sample.z_acc, sample.temperature);
	usleep(200000);
	if (c == 15)
	{	
		// Set Acc to Lowperformance mode
		ioctl(fd, SNIOC_SET_ACC_LOWPERF, 1);
		// Set Acc ODR to 1.6Hz
		ioctl(fd, SNIOC_SET_ACC_ODR, 1);
		// Set Gyro to Lowperformance mode
		ioctl(fd, SNIOC_SET_GYRO_LOWPERF, 1);
		// Set Gyro ODR to 52.0Hz
		ioctl(fd, SNIOC_SET_GYRO_ODR, 52);

	}
	}
	return 0;
}
