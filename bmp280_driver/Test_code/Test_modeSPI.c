#include "BMP280.h" 
#include <wiringPi.h>
int main() {
    wiringPiSetup();
    int fd;
    float altitude = 0;
    fd = BMP_mode(SPI_mode);
    BMP_mode_active(fd,HANDHELD_DEVICE_DYNAMIC);
    while (1) {
        printf("Temperature: %f\n",read_temperature(fd));
        printf("Pressure: %f\n",read_pressure(fd));
        delay(200);
    }
    close(fd);
    return 0;
}