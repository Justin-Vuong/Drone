#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

int main(void)
{
	uint16_t readData[4];
	
	int fd = open("/dev/ttyACM0", O_RDONLY);
	if (fd < 0)
	{
		return 0;
	}	
	while(1)
	{
		read(fd, readData, 8);
		printf("count:%u X:%u Y:%u Z:%u\n", readData[0], readData[1], readData[2], readData[3]);
	}
	
	return 0;
}
