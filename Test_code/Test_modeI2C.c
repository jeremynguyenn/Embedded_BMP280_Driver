#include "BMP280.h" 
#include <wiringPi.h>

char data[100];
int32_t TEM;
float PER;



int main() {
    wiringPiSetup();
    int fd;
    fd = BMP_mode(I2C_mode);
    BMP_config_0xF5(fd,BMP280_STANDBY_TIME_1000_MS,BMP280_FILTER_COEFF_OFF);
    BMP_ctrl_meas_0xF4(fd,BMP280_OVERSAMP_1X,BMP280_OVERSAMP_1X,BMP280_NORMAL_MODE);
    // BMP_mode_active(fd,WEATHER_MONITORING); 
    while (1) {
        printf("nhiet do: %f\n",read_temperature(fd));
        printf("ap suat: %f\n",read_pressure(fd));
        delay(200);
    }
    close(fd);
    return 0;
}
