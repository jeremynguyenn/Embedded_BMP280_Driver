#include "BMP280.h" 
#include <wiringPi.h>
#include <math.h>
#define sea_level_pressure 1013.25
int main() {
    wiringPiSetup();
    int fd;
    float altitude = 0;
    fd = BMP_mode(I2C_mode);
    BMP_config_0xF5(fd,BMP280_STANDBY_TIME_500_MS,BMP280_FILTER_COEFF_16);
    BMP_ctrl_meas_0xF4(fd,BMP280_OVERSAMP_2X,BMP280_OVERSAMP_16X,BMP280_NORMAL_MODE);
    while (1) {
        altitude = 44330.0 * (1.0 - pow(read_pressure(fd) / sea_level_pressure, 0.1903));
        printf("Altitude: %f\n", altitude);
        delay(200);
    }
    close(fd);
    return 0;
}