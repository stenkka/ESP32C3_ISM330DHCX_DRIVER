#include <nuttx/config.h>
#include <fcntl.h>
#include <stdio.h>
#include <nuttx/sensors/ism330dhcx.h>



struct sensor_data
{
  int16_t x_acc;        /* Measurement result for x axis */
  int16_t y_acc;        /* Measurement result for y axis */
  int16_t z_acc;        /* Measurement result for z axis */
  int16_t temperature;  /* Measurement result for temperature sensor */
};

int main(int argc, FAR char *argv[])
{
  	printf("Running MEMS app\n");
	
	int ret, fd;
	struct sensor_data sample;	
	fd = open("/dev/ism330dhcx", O_RDONLY); 
	while(1)
	{
		ret = read(fd, &sample, sizeof(sample));
		if(ret != sizeof(sample))
		{
			break;
		}
	printf("x_acc: %12d\n", sample.x_acc);
	//printf("x_acc: %12d, %y_acc: %12d, z_acc: %12d, temp: %12d", sample.x_acc, sample.y_acc, sample.z_acc, sample.temperature);
	usleep(500000);	
	}
	
	return 0;
}
