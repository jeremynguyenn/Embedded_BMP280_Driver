

#define TCS34725_IOCTL_MAGIC 'm'
#define TCS34725_IOCTL_READ_CLEAR        _IOR(TCS34725_IOCTL_MAGIC, 1, int)
#define TCS34725_IOCTL_READ_RED          _IOR(TCS34725_IOCTL_MAGIC, 2, int)
#define TCS34725_IOCTL_READ_GREEN        _IOR(TCS34725_IOCTL_MAGIC, 3, int)
#define TCS34725_IOCTL_READ_BLUE         _IOR(TCS34725_IOCTL_MAGIC, 4, int)
#define TCS34725_IOCTL_SET_UP_ID         _IOR(TCS34725_IOCTL_MAGIC, 5, int)

#define TCS34725_IOCTL_SET_UP_ENABLE     _IOW(TCS34725_IOCTL_MAGIC, 0, int)
#define TCS34725_IOCTL_SET_UP_ATIME      _IOW(TCS34725_IOCTL_MAGIC, 1, int)
#define TCS34725_IOCTL_SET_UP_WTIME      _IOW(TCS34725_IOCTL_MAGIC, 2, int)
//Khong truyen gia tri-----------------------------------------------------
#define TCS34725_IOCTL_SET_UP_AILTL      _IOW(TCS34725_IOCTL_MAGIC, 3, int)
#define TCS34725_IOCTL_SET_UP_AILTH      _IOW(TCS34725_IOCTL_MAGIC, 4, int)
#define TCS34725_IOCTL_SET_UP_AIHTL      _IOW(TCS34725_IOCTL_MAGIC, 5, int)
#define TCS34725_IOCTL_SET_UP_AIHTH      _IOW(TCS34725_IOCTL_MAGIC, 6, int)
//-------------------------------------------------------------------------
#define TCS34725_IOCTL_SET_UP_PPERS      _IOW(TCS34725_IOCTL_MAGIC, 7, int)
#define TCS34725_IOCTL_SET_UP_CONFIG     _IOW(TCS34725_IOCTL_MAGIC, 8, int)
#define TCS34725_IOCTL_SET_UP_GAIN       _IOW(TCS34725_IOCTL_MAGIC, 9, int)
#define TCS34725_IOCTL_SET_UP_STATUS     _IOW(TCS34725_IOCTL_MAGIC, 10, int)

//----------------------------------Giá tri cho thanh ghi ATIME-------------------
#define atime_2phay4ms				0xFF
#define atime_24ms       			0xF6
#define atime_101ms       			0xD5
#define atime_154ms       			0xC0
#define atime_700ms       			0x00
//---------------------------------Giá trị thanh ghi WTIME------------------------
#define wtime_2phay4ms					0xFF
#define wtime_204ms       			0xAB
#define wtime_614ms       			0x00
//-----------------------------------Giá trị thanh ghi PERS-----------------------
#define pers_every					0x00
#define pers_1						0x01
#define pers_2						0x02
#define pers_3						0x03
#define pers_5						0x04
#define pers_10						0x05
#define pers_15						0x06
#define pers_20						0x07
#define pers_25						0x08
#define pers_30						0x09
#define pers_35						0x0A
#define pers_40						0x0B
#define pers_45						0x0C
#define pers_50						0x0D
#define pers_55						0x0E
#define pers_60						0x0F
//----------------------------------Gia tri cho thanh ghi Config------------------------
#define config_off					0x00
#define config_on					0x02
//----------------------------------Gia tri cho thanh ghi Control (Gain)-----------
#define gain_x1       				0x00
#define gain_x4       				0x01
#define gain_x16       				0x10
#define gain_x60       				0x11
//-----------------------------------
void turn_On_Sensor();
void Init_Enable(int value);
void Init_Atime(int value);
void Init_Wtime(int value);
void Init_Ailtl();
void Init_Ailth();
void Init_Aihtl();
void Init_Aihtl();
void Init_Pers(int value);
void Init_Config(int value);
void Init_Control(int value);

int Read_ID();
int Read_CLEAR_data();
int Read_RED_data();
int Read_GREEN_data();
int Read_BLUE_data();


