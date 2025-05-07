#include <stdio.h>
#include "GY_86.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"




int main()
{
	MPU6050_t MPU6050;
	MS5611_t MS5611;

	MPU6050_Init(2,2);
	MPU6050_Bypass(); // Turn on Bypass mode to setup HMC5883L
	HMC5883L_Setup(1);
	MPU6050_Master(5); // Turn off Bypass and turn on Master Mode to read value
	MPU6050_Slave_Read();

	MS5611_Init();
	MS5611_Reset();
	MS5611_ReadProm(&MS5611);
	
	MS5611.OFF2 = MS5611.T2 = MS5611.SENS2 = 0;

	printf("Done setting\n");

	while(1){
		MPU6050_Read_All_Kalman(&MPU6050);
		
		MS5611_RequestPressure(4096);
		delay(100);
		MS5611_ReadPressure(&MS5611);
		
		MS5611_RequestTemperature(4096);
		delay(100);
		MS5611_ReadTemperature(&MS5611);

		MS5611_CalculateTemperature(&MS5611);
		Temp_Com(&MS5611);
		MS5611_CalculatePressure(&MS5611);

		print_MS5611(&MS5611);
		delay(1000);
	}

	return 0;
}
