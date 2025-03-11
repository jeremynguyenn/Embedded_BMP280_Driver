#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> // Include errno header
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
// address register
#define BMP280_ADDR             0x76
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_xLSB   0xF9
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_xLSB    0xFC
#define BMP280_REG_CALIB        0x88

// mode register
#define BMP280_FORCED_MODE   0x01
#define BMP280_SLEEP_MDOE    0x02
#define BMP280_NORMAL_MODE   0x03

//OVERSAMPLING DEFINITION
#define BMP280_OVERSAMP_SKIPPED   0x00
#define BMP280_OVERSAMP_1X        0x01
#define BMP280_OVERSAMP_2X        0x02
#define BMP280_OVERSAMP_4X        0x03
#define BMP280_OVERSAMP_8X        0x04
#define BMP280_OVERSAMP_16X       0x05

//FILTER DEFINITION
#define BMP280_FILTER_COEFF_OFF     0x00
#define BMP280_FILTER_COEFF_2       0x01
#define BMP280_FILTER_COEFF_4       0x02
#define BMP280_FILTER_COEFF_8       0x03
#define BMP280_FILTER_COEFF_16      0x04

//STANDBY TIME DEFINITION
#define BMP280_STANDBY_TIME_1_MS      0x00
#define BMP280_STANDBY_TIME_6_MS     0x01
#define BMP280_STANDBY_TIME_125_MS    0x02
#define BMP280_STANDBY_TIME_250_MS    0x03
#define BMP280_STANDBY_TIME_500_MS    0x04
#define BMP280_STANDBY_TIME_1000_MS   0x05
#define BMP280_STANDBY_TIME_2000_MS   0x06
#define BMP280_STANDBY_TIME_4000_MS   0x07

typedef enum {
    I2C_mode,
    SPI_mode
} mode_interface;

typedef enum {
    HANDHELD_DEVICE_LOW_POWER=3,
    WEATHER_MONITORING,
    HANDHELD_DEVICE_DYNAMIC,
    EVELATOR_FLOOR_CHANGE,
    DROP_DETECTION,
    INDOOR_NAVIGATION,
    CONFIG_SETUP_0xF4,
    CONFIG_SETUP_0xF5
} MODE_ACTIVE;



int BMP_mode(mode_interface mode) {
    int fd;
    if (mode == I2C_mode) {
        fd = open("/dev/bmp280_device_I2C",O_RDWR);
        if (fd<0) {
        perror("Failed to open I2C device...\n");
        return errno;
    }
    }
    else if (mode == SPI_mode) {
        fd = open("/dev/bmp280_device_spi",O_RDWR);
        if (fd<0) {
        perror("Failed to open SPI device...\n");
        return errno;
    }   
    }
    return fd;
}

void BMP_config_0xF5(int fd,uint8_t standby_time,uint8_t filter){
    char data_config;
    data_config = standby_time<<5 | filter<<2 | 0x00;
    if (ioctl(fd,CONFIG_SETUP_0xF5,data_config) <0 ){
        perror("Failed to set up 0xF5...");
        close(fd);
        return errno;
    };
}

void BMP_ctrl_meas_0xF4(int fd,uint8_t ovs_t,uint8_t ovs_p,uint8_t mode){
    char data_ctrl_meas;
    data_ctrl_meas = ovs_t<<5 | ovs_p<<2 | mode;
    if (ioctl(fd,CONFIG_SETUP_0xF4,data_ctrl_meas) <0 ){
        perror("Failed to set up 0xF4...");
        close(fd);
        return errno;
    };
}

uint8_t BMP_mode_active(int fd,MODE_ACTIVE cmd) {
    char data_config,data_ctrl_meas;

    switch (cmd) {
        case HANDHELD_DEVICE_LOW_POWER:
            if (ioctl(fd,HANDHELD_DEVICE_LOW_POWER,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        case WEATHER_MONITORING:
            if (ioctl(fd,WEATHER_MONITORING,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        case HANDHELD_DEVICE_DYNAMIC:
            if (ioctl(fd,HANDHELD_DEVICE_DYNAMIC,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        case EVELATOR_FLOOR_CHANGE:
            if (ioctl(fd,EVELATOR_FLOOR_CHANGE,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        case DROP_DETECTION:
            if (ioctl(fd,DROP_DETECTION,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        case INDOOR_NAVIGATION:
            if (ioctl(fd,INDOOR_NAVIGATION,0) <0 ){
                perror("Failed to set up...");
                close(fd);
                return errno;
            };
            break;
        default:
            return -1;
    }
}

float read_temperature(int fd) {
    float T;
    char data[100];
    if (read(fd,data,sizeof(data)) < 0) {
            perror("Failed to read Temperature...");
            close(fd);
            return errno;
    }
    strtok(data,"\n");
    T =atof(data);
    return T;
}

float read_pressure(int fd) {
    float P;
    char data[100];
    char *token;
    if (read(fd,data,sizeof(data)) < 0) {
            perror("Failed to read Pressure...");
            close(fd);
            return errno;
    }
    strtok(data,"\n");
    token = strtok(NULL,"\n");
    P =atof(token);
    return P;
}