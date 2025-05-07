#ifndef GY_86_H
#define GY_86_H

#include <stdint.h>

#define D2R 0.01745329252 
#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDR                0x68
#define CONFIG_REG                  0x1A
#define GYRO_CONFIG_REG             0x1B
#define ACCEL_CONFIG_REG            0x1C
#define SMPLRT_DIV_REG              0x19
#define I2C_MST_CTRL                0x24
#define I2C_SLV0_ADDR               0x25
#define I2C_SLV0_REG                0x26
#define I2C_SLV0_CTRL               0x27
#define INT_PIN_CFG                 0x37
#define INT_ENABLE_REG              0x38
#define INT_STATUS_REG              0x3A

#define ACCEL_XOUT_H_REG            0x3B
#define ACCEL_XOUT_L_REG            0x3C
#define ACCEL_YOUT_H_REG            0x3D
#define ACCEL_YOUT_L_REG            0x3E
#define ACCEL_ZOUT_H_REG            0x3F
#define ACCEL_ZOUT_L_REG            0x40

#define TEMP_OUT_H_REG              0x41
#define TEMP_OUT_L_REG              0x42

#define GYRO_XOUT_H_REG             0x43
#define GYRO_XOUT_L_REG             0x44
#define GYRO_YOUT_H_REG             0x45
#define GYRO_YOUT_L_REG             0x46
#define GYRO_ZOUT_H_REG             0x47
#define GYRO_ZOUT_L_REG             0x48

// Using Master Mode to read HMC5883L through MPU6050

#define REG_OUT_MAG_X_M          (0x03)
#define REG_OUT_MAG_X_L          (0x04)
#define REG_OUT_MAG_Z_M          (0x05)
#define REG_OUT_MAG_Z_L          (0x06)
#define REG_OUT_MAG_Y_M          (0x07)
#define REG_OUT_MAG_Y_L          (0x08)


#define USER_CTRL_REG               0x6A
#define PWR_MGMT_1_REG              0x6B
#define WHO_AM_I_REG                0x75


#define HMC5883L_ADDRESS              (0x1E) 
#define HMC5883L_ADDRESS_WRITE        (0x3C) 
#define HMC5883L_ADDRESS_READ         (0x3D) 
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

#define MS5611_ADDR 0x77

#define ADC_READ      0x00

#define CONVERSION_OSR_256  1
#define CONVERSION_OSR_512  2
#define CONVERSION_OSR_1024 3
#define CONVERSION_OSR_2048 5
#define CONVERSION_OSR_4096 10

#define CMD_RESET 0x1E
#define CMD_PROM_C0 0xA0
#define CMD_PROM_C1 0xA2
#define CMD_PROM_C2 0xA4
#define CMD_PROM_C3 0xA6
#define CMD_PROM_C4 0xA8
#define CMD_PROM_C5 0xAA
#define CMD_PROM_C6 0xAC
#define CMD_PROM_C7 0xAE

#define PRESSURE_OSR_256 0x40
#define PRESSURE_OSR_512 0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256 0x50
#define TEMP_OSR_512 0x52
#define TEMP_OSR_1024 0x54
#define TEMP_OSR_2048 0x56
#define TEMP_OSR_4096 0x58

#define PMAX 1200
#define PMIN 10

#define TMAX 85
#define TMIN -40






typedef struct _MPU6050_t
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    int16_t TEMP_RAW;

    int16_t Gyro_X_Offset;
    int16_t Gyro_Y_Offset;
    int16_t Gyro_Z_Offset;

    float Gx;
    float Gy;
    float Gz;

    float Temperature;


    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;

    int16_t Mag_X_Min;
	int16_t Mag_Y_Min;
	int16_t Mag_Z_Min;

    int16_t Mag_X_Max;
	int16_t Mag_Y_Max;
	int16_t Mag_Z_Max;

    int16_t Mag_X_Offset;
	int16_t Mag_Y_Offset;
	int16_t Mag_Z_Offset;

    float Mx;
    float My;
    float Mz;

    double KalmanAngleX; // roll angle
    double KalmanAngleY; // pitch angle

}MPU6050_t;

typedef struct _Kalman_t
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

typedef struct _MS5611{
    uint16_t C[8];
    uint32_t DigitalPressure_D1;
    uint32_t DigitalTemperature_D2;
    int32_t dT;
    int32_t TEMP;
    int64_t OFF;
    int64_t SENS;
    int32_t P;

    float alt;
    

    int OFF2;
    int T2;
    int SENS2;

}MS5611_t;

uint8_t MPU6050_Init(uint8_t Gyro_FS, uint8_t ACC_FS);
int MPU6050_Bypass();
int MPU6050_Master(uint8_t clk_div);
int MPU6050_Slave_Read();
void MPU6050_Read_All_Raw(MPU6050_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void MPU6050_Read_All_Kalman(MPU6050_t *DataStruct);
int HMC5883L_Setup(int Mag_Range);
int MS5611_Init();
int MS5611_Reset();
void Temp_Com(MS5611_t *DataStruct);
int MS5611_ReadProm(MS5611_t *DataStruct);
int MS5611_RequestPressure(int osr);
int MS5611_RequestTemperature(int osr);
int MS5611_ReadPressure(MS5611_t *DataStruct);
int MS5611_ReadTemperature(MS5611_t *DataStruct);
int MS5611_CalculatePressure(MS5611_t *DataStruct);
int MS5611_CalculateTemperature(MS5611_t *DataStruct);
void print_MS5611(MS5611_t *data);
void print_MPU6050(MPU6050_t *data);
float MS5611_getAltitude1(float pressure);













#endif
