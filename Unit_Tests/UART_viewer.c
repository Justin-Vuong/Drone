#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

int main(void)
{
	char readData;
	
	int fd = open("/dev/ttyACM0", O_RDONLY);
	if (fd < 0)
	{
		fd = open("/dev/ttyACM1", O_RDONLY);
		if (fd < 0)
		{
			return 0;
		}
	}	
	while(1)
	{
		read(fd, &readData, 1);
		printf("%c", readData);
		
	}
	
	return 0;
}
