// Huynh Tan Luong   21146267
// Nguyen Chi Thanh  21146315
// Nguyen Anh Tuan   21146570
// Nguyen Minh Phuoc 21146297

#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> 

#include "tcs34725_library.h"

#define DEVICE_PATH "/dev/tcs34725"

int fd;
void turn_On_Sensor()
{
	fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) 
	{
        printf("loi roi\n");
    }
}
void Init_Enable(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_ENABLE, &value) < 0) {
        printf("loi roi ENABLE\n");       
    }
}
void Init_Atime(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_ATIME, &value) < 0) {
        printf("loi roi ATIME\n");       
    }
}
void Init_Wtime(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_WTIME, &value) < 0) {
        printf("loi roi WTIME\n");       
    }
}

void Init_Ailtl()
{
int value;
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_AILTL, &value) < 0) {
        printf("loi roi AILTL\n");       
    }
}

void Init_Ailth()
{
int value;
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_AILTH, &value) < 0) {
        printf("loi roi AILTH\n");       
    }
}
void Init_Aihtl()
{
int value;
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_AIHTL, &value) < 0) {
        printf("loi roi AIHTL\n");       
    }
}
void Init_Aihth()
{
int value;
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_AIHTH , &value) < 0) {
        printf("loi roi AIHTH\n");       
    }
}
void Init_Pers(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_PPERS, &value) < 0) {
        printf("loi roi PERS\n");       
    }
}
void Init_Config(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_CONFIG, &value) < 0) {
        printf("loi roi CONFIG\n");       
    }
}
void Init_Control(int value)
{
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_GAIN, &value) < 0) {
			printf("loi roi GAIN\n");       
		}
}
int Read_ID()
{
	int value;
	if (ioctl(fd, TCS34725_IOCTL_SET_UP_ID, &value) < 0) {
			printf("loi roi ID\n");       
		}
	return value;
}
int Read_CLEAR_data()
{
	int value;
	if (ioctl(fd, TCS34725_IOCTL_READ_CLEAR, &value) < 0) {
			printf("loi roi CLEAR\n");       
		}
	return value;
}

int Read_RED_data()
{
	int value;
	if (ioctl(fd, TCS34725_IOCTL_READ_RED, &value) < 0) {
			printf("loi roi RED\n");       
		}
	return value;
}

int Read_GREEN_data()
{
	int value;
	if (ioctl(fd, TCS34725_IOCTL_READ_GREEN, &value) < 0) {
			printf("loi roi GREEN\n");       
		}
	return value;
}
int Read_BLUE_data()
{
	int value;
	if (ioctl(fd, TCS34725_IOCTL_READ_BLUE, &value) < 0) {
			printf("loi roi BLUE\n");       
		}
	return value;
}

