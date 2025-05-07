#include "GY_86.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>


int fd;
int fd_MS;
int fd_HMC;
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_rx;
uint8_t MPU6050_tx;
uint8_t MS5611_rx;
uint8_t MS5611_tx;
float MPU6050_Gyro_LSB;
float MPU6050_Acc_LSB;
float Mag_FS;

uint8_t MPU6050_Init(uint8_t Gyro_FS, uint8_t Acc_FS){
	switch(Gyro_FS)
	{
	case 0: //250dps
		MPU6050_Gyro_LSB = 131.0;
		break;
	case 1: //500dps
		MPU6050_Gyro_LSB = 65.5;
		break;
	case 2: //1000dps
		MPU6050_Gyro_LSB = 32.8;
		break;
	case 3: //2000dps
		MPU6050_Gyro_LSB = 16.4;
		break;
	default:
		break;
	}

	switch(Acc_FS)
	{
	case 0: //2g
		MPU6050_Acc_LSB = 16384.0;
		break;
	case 1: //4g
		MPU6050_Acc_LSB = 8192.0;
		break;
	case 2: //8g
		MPU6050_Acc_LSB = 4096.0;
		break;
	case 3: //16g
		MPU6050_Acc_LSB = 2048.0;
		break;
	default:
		break;
	}

	fd = wiringPiI2CSetup(MPU6050_ADDR);
	if(fd == -1){
		return 1;
	}

	MPU6050_rx = wiringPiI2CReadReg8(fd,WHO_AM_I_REG);
	MPU6050_tx = 0; //Will return this value if code ends here

	if(MPU6050_rx == 0x68){

		// Reset Device
		MPU6050_tx = 0x80;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(100);

		// Wake device up and Clock source = Internal 8MHz oscillator (default)
		MPU6050_tx = 0;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(10);

		// Set clock source to PLL with X-axis Gyro 8MHz
		MPU6050_tx = 1;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = 7; // Set SMPLRT_DIV = 0 -> sample rate = 1 KHz
		wiringPiI2CWriteReg8(fd, SMPLRT_DIV_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = 0x03; // Setting Digital Low-Pass Filter depending on sample rate above
		wiringPiI2CWriteReg8(fd, CONFIG_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = Gyro_FS << 3; // Gyro configure 
		wiringPiI2CWriteReg8(fd, GYRO_CONFIG_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = Acc_FS << 3; // Acc configure 
		wiringPiI2CWriteReg8(fd, ACCEL_CONFIG_REG, MPU6050_tx);
		delay(10);

		return 0;
	}
	return 1;
}

int MPU6050_Bypass(){
	MPU6050_tx = 0b00000000; // Precondition to enable Bypass Mode
	wiringPiI2CWriteReg8(fd, USER_CTRL_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00000010; // Enable Bypass ModeINT_PIN_CFG
	wiringPiI2CWriteReg8(fd, INT_PIN_CFG, MPU6050_tx);
	delay(10);
	return 0;
}

int MPU6050_Master(uint8_t clk_div) // range of clk_div from 0 to 15
{
	MPU6050_tx = 0x00; // Disable Bypass ModeINT_PIN_CFG
	wiringPiI2CWriteReg8(fd, INT_PIN_CFG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00100010; // Enable I2C Master Mode
	wiringPiI2CWriteReg8(fd, USER_CTRL_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = clk_div; // You should choose clk_div = 13 <=> I2C Master Clock Speed = 400kHz(fast mode of I2C)
	if(clk_div > 15 || clk_div < 0) return 1;
	wiringPiI2CWriteReg8(fd, I2C_MST_CTRL, MPU6050_tx);
	delay(10);

	MPU6050_tx = 1; // Set clock source to PLL with X-axis Gyro 8MHz
	wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
	delay(10);

	return 0;
}

int MPU6050_Slave_Read(){
	MPU6050_tx = HMC5883L_ADDRESS | 0x80; //Access Slave into read mode
	wiringPiI2CWriteReg8(fd, I2C_SLV0_ADDR, MPU6050_tx);
	delay(10);

	MPU6050_tx = HMC5883L_REG_OUT_X_M; // Address which HMC5883L's data transfer start
	wiringPiI2CWriteReg8(fd, I2C_SLV0_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0x80 | 0x06; 
	//Enable Slave 0 for data transfer operations and
	//specifies number of bytes transfered (6 bytes respectively x,y and z axis in this case)
	wiringPiI2CWriteReg8(fd, I2C_SLV0_CTRL, MPU6050_tx);
	delay(10);

	return 0;
}

int HMC5883L_Setup(int Mag_Range){
	// mGa/LSB 
	switch(Mag_Range){
	case 0: // 0.88 Ga
		Mag_FS = 0.73;
		break;
	case 1: // 1.3 Ga
		Mag_FS = 0.92;
		break;		
	case 2: // 1.9 Ga
		Mag_FS = 1.22;
		break;
	case 3: // 2.5 Ga
		Mag_FS = 1.52;
		break;
	case 4: // 4 Ga
		Mag_FS = 2.27;
		break;
	case 5: // 4.7 Ga
		Mag_FS = 2.56;
		break;		
	case 6: // 5.6 Ga
		Mag_FS = 3.03;
		break;
	case 7: // 8.1 Ga
		Mag_FS = 4.35;
		break;
	default:
		break;
	}
	fd_HMC = wiringPiI2CSetup(HMC5883L_ADDRESS);
	if(fd_HMC == -1){
		return 1;
	}

	MPU6050_tx = 0b00011000; // Data Output rate 75Hz
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_CONFIG_A, MPU6050_tx); 
	delay(10);

	MPU6050_tx = Mag_Range<<5; // Set Range of sensor
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_CONFIG_B, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00000000; // Continuous mode
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_MODE, MPU6050_tx);
	delay(10);

	return 0;
}

void MPU6050_Read_All_Raw(MPU6050_t *DataStruct){
	DataStruct->Accel_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_XOUT_L_REG));
	DataStruct->Accel_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_YOUT_L_REG));
	DataStruct->Accel_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L_REG));

	DataStruct->TEMP_RAW = (int16_t)((wiringPiI2CReadReg8(fd, TEMP_OUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, TEMP_OUT_L_REG));

	DataStruct->Gyro_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_XOUT_L_REG));
	DataStruct->Gyro_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_YOUT_L_REG));
	DataStruct->Gyro_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_ZOUT_L_REG));
	
	DataStruct->Mag_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_L));
	DataStruct->Mag_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_L));
	DataStruct->Mag_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_L));



	DataStruct->Gx = (DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gy = (DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gz = (DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB)* D2R;

	DataStruct->Temperature = (float)(DataStruct->TEMP_RAW / (float)340.0 + (float)36.53); // Unit is degrees

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

void MPU6050_Read_All_Kalman(MPU6050_t *DataStruct){

	DataStruct->Accel_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_XOUT_L_REG));
	DataStruct->Accel_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_YOUT_L_REG));
	DataStruct->Accel_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L_REG));

	DataStruct->TEMP_RAW = (int16_t)((wiringPiI2CReadReg8(fd, TEMP_OUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, TEMP_OUT_L_REG));

	DataStruct->Gyro_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_XOUT_L_REG));
	DataStruct->Gyro_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_YOUT_L_REG));
	DataStruct->Gyro_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_ZOUT_L_REG));
	
	DataStruct->Mag_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_L));
	DataStruct->Mag_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_L));
	DataStruct->Mag_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_L));

	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

	DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
	DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
	DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

	DataStruct->Gx = (DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gy = (DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gz = (DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB)* D2R;

	DataStruct->Temperature = (float)(DataStruct->TEMP_RAW / (float)340.0 + (float)36.53);

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;

	DataStruct->Mx = DataStruct->Mag_X_RAW * Mag_FS;
	DataStruct->My = DataStruct->Mag_Y_RAW * Mag_FS;
	DataStruct->Mz = DataStruct->Mag_Z_RAW * Mag_FS;


	// Kalman angle solve
    double dt = (double)(millis() - timer) / 1000.0;
    timer = millis(); // Update timestamp
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt){
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void print_MS5611(MS5611_t *data) {
    printf("MS5611 Sensor Data:\n");
    for (int i = 0; i < 8; i++) {
        printf("C[%d]: %u\n", i, data->C[i]);
    }
    printf("DigitalPressure_D1: %u\n", data->DigitalPressure_D1);
    printf("DigitalTemperature_D2: %u\n", data->DigitalTemperature_D2);
    printf("dT: %d\n", data->dT);
    printf("OFF: %lld\n", data->OFF);
    printf("SENS: %lld\n", data->SENS);
    printf("P: %d\n", data->P);
    printf("OFF2: %d\n", data->OFF2);
    printf("T2: %d\n", data->T2);
    printf("SENS2: %d\n", data->SENS2);
    printf("TEMP: %d\n", data->TEMP);
    printf("Altitude: %f\n", data->alt);
}

void print_MPU6050(MPU6050_t *data) {
    printf("MPU6050 Sensor Data:\n");
    printf("Accel_X_RAW: %d, Accel_Y_RAW: %d, Accel_Z_RAW: %d\n", data->Accel_X_RAW, data->Accel_Y_RAW, data->Accel_Z_RAW);
    printf("Ax: %.2f, Ay: %.2f, Az: %.2f\n", data->Ax, data->Ay, data->Az);
    printf("Gyro_X_RAW: %d, Gyro_Y_RAW: %d, Gyro_Z_RAW: %d\n", data->Gyro_X_RAW, data->Gyro_Y_RAW, data->Gyro_Z_RAW);
    printf("TEMP_RAW: %d\n", data->TEMP_RAW);
    printf("Gyro Offsets: X: %d, Y: %d, Z: %d\n", data->Gyro_X_Offset, data->Gyro_Y_Offset, data->Gyro_Z_Offset);
    printf("Gx: %.2f, Gy: %.2f, Gz: %.2f\n", data->Gx, data->Gy, data->Gz);
    printf("Temperature: %.2f\n", data->Temperature);
    printf("Mag_X_RAW: %d, Mag_Y_RAW: %d, Mag_Z_RAW: %d\n", data->Mag_X_RAW, data->Mag_Y_RAW, data->Mag_Z_RAW);
    printf("Mag Min: X: %d, Y: %d, Z: %d\n", data->Mag_X_Min, data->Mag_Y_Min, data->Mag_Z_Min);
    printf("Mag Max: X: %d, Y: %d, Z: %d\n", data->Mag_X_Max, data->Mag_Y_Max, data->Mag_Z_Max);
    printf("Mag Offsets: X: %d, Y: %d, Z: %d\n", data->Mag_X_Offset, data->Mag_Y_Offset, data->Mag_Z_Offset);
    printf("Mx: %.2f, My: %.2f, Mz: %.2f\n", data->Mx, data->My, data->Mz);
    printf("KalmanAngleX: %.2f, KalmanAngleY: %.2f\n", data->KalmanAngleX, data->KalmanAngleY);
}

int MS5611_Init(){
	fd_MS = wiringPiI2CSetup(MS5611_ADDR);
	if(fd_MS == -1){
		return 1;
	}
	return 0;
}

void Temp_Com(MS5611_t *DataStruct){
	if(DataStruct->TEMP > 2000){
		DataStruct->T2 = 0;
		DataStruct->OFF2 = 0;
		DataStruct->SENS2 = 0;
	}
	else{
		DataStruct->T2 = (DataStruct->dT * DataStruct->dT)>>31; // 2^31
		if(DataStruct->TEMP < -1500){
			DataStruct->OFF2 += 7 * (DataStruct->TEMP + 1500) * (DataStruct->TEMP + 1500);
			DataStruct->SENS2 += ((11 * (DataStruct->TEMP + 1500) * (DataStruct->TEMP + 1500))>>1);
		}
		else{
			DataStruct->OFF2 = 5 * (DataStruct->TEMP - 2000) * (DataStruct->TEMP - 2000) / 2;
			DataStruct->SENS2 = 5 * (DataStruct->TEMP - 2000) * (DataStruct->TEMP - 2000) / 4;
		}
	}
}

int MS5611_Reset(){
	MS5611_tx = CMD_RESET;
	wiringPiI2CWriteReg8(fd_MS, MS5611_ADDR, MS5611_tx);
	delay(10);
	return 0;
}

int MS5611_ReadProm(MS5611_t *DataStruct){
	DataStruct->C[0] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C0) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C0 + 1);
	delay(10);

	DataStruct->C[1] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C1) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C1 + 1);
	delay(10);
	
	DataStruct->C[2] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C2) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C2 + 1);
	delay(10);

	DataStruct->C[3] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C3) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C3 + 1);
	delay(10);

	DataStruct->C[4] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C4) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C4 + 1);
	delay(10);

	DataStruct->C[5] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C5) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C5 + 1);
	delay(10);

	DataStruct->C[6] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C6) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C6 + 1);
	delay(10);

	DataStruct->C[7] = wiringPiI2CReadReg8(fd_MS,CMD_PROM_C7) << 8 | wiringPiI2CReadReg8(fd_MS,CMD_PROM_C7 + 1);
	delay(10);
}

int MS5611_RequestPressure(int osr){
	switch(osr){
	case 256:
		MS5611_tx =PRESSURE_OSR_256; // tc = 1 ms
		break;
	case 512:
		MS5611_tx =PRESSURE_OSR_512; // tc = 2 ms
		break;
	case 1024:
		MS5611_tx =PRESSURE_OSR_1024; // tc = 3 ms
		break;
	case 2048:
		MS5611_tx =PRESSURE_OSR_2048; // tc = 5 ms
		break;
	case 4096:
		MS5611_tx =PRESSURE_OSR_4096; // tc = 10 ms
		break;
	default:
		return 1;
		break;
	}

	wiringPiI2CWrite(fd_MS, MS5611_tx);
	return 0;
}

int MS5611_RequestTemperature(int osr){
	switch(osr){
	case 256:
		MS5611_tx =TEMP_OSR_256; // tc = 1 ms
		break;
	case 512:
		MS5611_tx =TEMP_OSR_512; // tc = 2 ms
		break;
	case 1024:
		MS5611_tx =TEMP_OSR_1024; // tc = 3 ms
		break;
	case 2048:
		MS5611_tx =TEMP_OSR_2048; // tc = 5 ms
		break;
	case 4096:
		MS5611_tx =TEMP_OSR_4096; // tc = 10 ms
		break;
	default:
		return 1;
		break;
	}

	wiringPiI2CWrite(fd_MS, MS5611_tx);
	return 0;
}

int MS5611_ReadPressure(MS5611_t *DataStruct){
	uint8_t buffer_P[3];
	MS5611_tx = 0x00;
	wiringPiI2CWrite(fd_MS, MS5611_tx);
	read(fd_MS, buffer_P, 3);
	DataStruct->DigitalPressure_D1 = (buffer_P[0] << 16) | (buffer_P[1] << 8) | buffer_P[2];
	return 0;
}

int MS5611_ReadTemperature(MS5611_t *DataStruct){
	uint8_t buffer_T[3];
	MS5611_tx = 0x00;
	wiringPiI2CWrite(fd_MS,MS5611_tx);
	read(fd_MS, buffer_T, 3);
	DataStruct->DigitalTemperature_D2 = (buffer_T[0] << 16) | (buffer_T[1] << 8) | buffer_T[2];
	return 0;
}

int MS5611_CalculateTemperature(MS5611_t *DataStruct){
	DataStruct->dT = DataStruct->C[5];
	DataStruct->dT <<= 8; //Calculated up to C5 * 2^8
	DataStruct->dT *= -1; //Apply negative sign
	DataStruct->dT += DataStruct->DigitalTemperature_D2; // = D2 - C5 * 2^8

	DataStruct->TEMP = DataStruct->dT * DataStruct->C[6] / pow(2,23);
	DataStruct->TEMP += 2000;

	if(DataStruct->TEMP>TMAX*100) DataStruct->TEMP = TMAX*100;
	if(DataStruct->TEMP <TMIN*100) DataStruct->TEMP = TMIN*100;

	return 0;
}

int MS5611_CalculatePressure(MS5611_t *DataStruct){
	DataStruct->OFF = DataStruct->C[2];
	DataStruct->OFF <<= 16; //Calculated up to C2 * 2^16
	DataStruct->OFF += (DataStruct->C[4] * DataStruct->dT) / pow(2,7);


	DataStruct->SENS = DataStruct->C[1];
	DataStruct->SENS <<= 15; // Calculated up to C1 * 2^15
	DataStruct->SENS += (DataStruct->C[3] * DataStruct->dT) / pow(2,8);

	DataStruct->P = ((DataStruct->DigitalPressure_D1 * DataStruct->SENS) / pow(2,21) - DataStruct->OFF) / pow(2,15);
	if(DataStruct->P>PMAX*100) DataStruct->P = PMAX*100;
	if(DataStruct->P<PMIN*100) DataStruct->P = PMIN*100;

	DataStruct-> alt = MS5611_getAltitude1(DataStruct->P/100);

	return 0;
}

#define SEA_PRESSURE 1013.25f  // Standard atmospheric pressure in hPa (mbar)


float MS5611_getAltitude1(float pressure){
	return (44330.0f * (1.0f - powf(pressure / SEA_PRESSURE, 0.1902949f)));
}




