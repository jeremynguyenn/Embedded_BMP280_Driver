


#include <stdio.h>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <string.h>

#include "tcs34725_library.h"



//define gia tri cho SPI
int fd_spi;
int bpw = 8;
int max_Speed = 8000000;

void Init_SPI()
{
	//load SPI dirver
    fd_spi = open("/dev/spidev0.0",O_RDWR);
    if (fd_spi<0) 
    {
        printf("Can't load SPI driver\n");
        
    }
    //set SPI operationanl mode
    int mode =0;
    if (ioctl(fd_spi,SPI_IOC_WR_MODE, &mode)<0)
    {
        printf("Can't get SPI mode\n");
        
    }
    int lsb_first = 0;
    if (ioctl(fd_spi,SPI_IOC_WR_LSB_FIRST, &lsb_first)<0)
    {
        printf("Can't set MSB mode\n");
        
    }
    int bpw = 8;
    if (ioctl(fd_spi,SPI_IOC_WR_BITS_PER_WORD,&bpw)<0)
    {
        printf("Can't set BIT per world\n");
        
    }
    int max_Speed = 8000000;
    if (ioctl(fd_spi,SPI_IOC_WR_MAX_SPEED_HZ, &max_Speed)<0)
    {
        printf("Can't set MSB mode\n");
        
    }
}
void sendData(uint8_t address, uint8_t value)
{
    struct spi_ioc_transfer spi;
    memset(&spi,0,sizeof(spi));
    uint8_t data[2];
    data[0] = address;
    data[1] = value;

    spi.tx_buf = (unsigned long)data;
    spi.len = 2;
    spi.delay_usecs   = 0 ;
    spi.bits_per_word = bpw;
    spi.speed_hz = max_Speed;

    ioctl(fd_spi, SPI_IOC_MESSAGE(1),&spi);

}
void setup_MAX7219()
{
	//setup max7219
	sendData(0x09, 0x00); 	//khong bat decode 
	sendData(0x0A, 0x0B);	// intensity 70
	sendData(0x0B, 0);		// scan limit
	sendData(0x0C, 0x01);	//shutdown (normal operation = 1) and shutdown = 0
	sendData(0x0F, 0);		//test led
}

void display_RED()
{
	sendData(0x0B, 2);
	sendData(1,0x7E);
	sendData(2,0x4F);
	sendData(3,0x77);
}
void display_BLUE()
{
	sendData(0x0B, 3);
	sendData(1,0x4F);
	sendData(2,0x3E);
	sendData(3,0x0E);
	sendData(4,0x7F);
}
void display_GREEN()
{
	sendData(0x0B, 4);
	sendData(1,0x76);
	sendData(2,0x4F);
	sendData(3,0x4F);
	sendData(4,0x77);
	sendData(5,0x5F);
	
}
int r,g,b,c;

int main()
{
	Init_SPI();
	setup_MAX7219();
	//display_GREEN();
	
	//Set up TCS34725
	turn_On_Sensor();
	Init_Enable(0x1B);
	Init_Atime(atime_24ms);
	Init_Control(gain_x4);

	while (1)
	{
		c = Read_CLEAR_data();
		r = Read_RED_data();
		g = Read_GREEN_data();
		b = Read_BLUE_data();
		
		//printf("c: %d, r: %d, g: %d, b: %d\n", c,r,g,b);
		int red, green, blue;
		red = r*255/c;
		green = g*255/c;
		blue = b*255/c;
		printf("r: %d, g: %d, b: %d\n",red,green,blue);
		
		if (red>100&&green<100&&blue<100)
		{
			display_RED();
		}
		
		if (red<100&&green>100&&blue<100)
		{
			display_GREEN();
		}
		
		if (red<100&&green<100&&blue>100)
		{
			display_BLUE();
		}
			
		sleep(1);
		
	}
	return 0;
	
}
