#include "BMP280.h" 
#include <wiringPi.h>
// measuare temperature and pressure by using Weather_monitoring mode
int main() {
    wiringPiSetup();
    int fd;
    fd = BMP_mode(I2C_mode);
    BMP_mode_active(fd,WEATHER_MONITORING);    
    while (1) {
        printf("Temperature: %f\n",read_temperature(fd));
        printf("Pressure: %f\n",read_pressure(fd));
        delay(60000); // 60s update 
    }
    close(fd);
    return 0;
}