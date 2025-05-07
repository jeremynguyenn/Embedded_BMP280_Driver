#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>


#define DEVICE_NAME "bmp280_device_I2C" 
#define CLASS_NAME "bmp280_device_I2C"

/* Variables for device and device class */
static struct class *bmp280_class=NULL;
static struct device *bmp280_device=NULL;

static int major_number;
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
#define BMP280_NORMAL_MODE   0x03
#define BMP280_SLEEP_MODE    0x00

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
#define BMP280_STANDBY_TIME_63_MS     0x01
#define BMP280_STANDBY_TIME_125_MS    0x02
#define BMP280_STANDBY_TIME_250_MS    0x03
#define BMP280_STANDBY_TIME_500_MS    0x04
#define BMP280_STANDBY_TIME_1000_MS   0x05
#define BMP280_STANDBY_TIME_2000_MS   0x06
#define BMP280_STANDBY_TIME_4000_MS   0x07

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

int data[26];
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t adc_T,adc_P;
int32_t t_fine,T;
int32_t P;
static int device_open_count=0;

static struct i2c_client *bmp280_client;

static void read_calib(struct i2c_client *client){
    for (int i = 136;i <=161;i++) { 
        data[i-136] = i2c_smbus_read_byte_data(client,i);    
    }
    dig_T1 = (data[1]<<8) | data[0];
    dig_T2 = (data[3]<<8) | data[2];
    dig_T3 = (data[5]<<8) | data[4];
    dig_P1 = (data[7]<<8) | data[6];
    dig_P2 = (data[9]<<8) | data[8];
    dig_P3 = (data[11]<<8) | data[10];
    dig_P4 = (data[13]<<8) | data[12];
    dig_P5 = (data[15]<<8) | data[14];
    dig_P6 = (data[17]<<8) | data[16];
    dig_P7 = (data[19]<<8) | data[18];
    dig_P8 = (data[21]<<8) | data[20];
    dig_P9 = (data[23]<<8) | data[22];

    for (int i =0; i<=23;i++) {
        printk("data thu %d:%d",i,data[i]);
    }
}

int32_t bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    T = var1 + var2;
    return T;
}

int32_t bmp280_compensate_P(int32_t adc_P,int32_t t_fine)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}



static int driver_open(struct inode *device_file, struct file *instance) {
	if (device_open_count>0){
        return -EBUSY;
    }
    printk("dev_nr - open was called!\n");
    device_open_count++;
    try_module_get(THIS_MODULE);
	return 0;
}

static int driver_close(struct inode *device_file, struct file *instance) {
    printk("dev_nr - close was called!\n");
    device_open_count--;
    module_put(THIS_MODULE);
	return 0;   
}


static long bmp280_ioctl(struct file *file, MODE_ACTIVE cmd, unsigned long arg)
{
    char data_config,data_ctrl_meas;

    if (!bmp280_client) {
        printk(KERN_ERR "bmp280_client is NULL\n");
        return -EFAULT;
    }
    switch (cmd) {
        case HANDHELD_DEVICE_LOW_POWER:
            data_config = BMP280_STANDBY_TIME_63_MS<<5 | BMP280_FILTER_COEFF_4<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_2X<<5 | BMP280_OVERSAMP_16X<<2 | BMP280_NORMAL_MODE; 
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas);
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case WEATHER_MONITORING:
            data_config = BMP280_STANDBY_TIME_1000_MS<<5 | BMP280_FILTER_COEFF_OFF<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_1X<<5 | BMP280_OVERSAMP_1X<<2 | BMP280_FORCED_MODE;
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas); 
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case HANDHELD_DEVICE_DYNAMIC:
            data_config = BMP280_STANDBY_TIME_1_MS<<5 | BMP280_FILTER_COEFF_16<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_1X<<5 | BMP280_OVERSAMP_4X<<2 | BMP280_NORMAL_MODE;
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas); 
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case EVELATOR_FLOOR_CHANGE:
            data_config = BMP280_STANDBY_TIME_125_MS<<5 | BMP280_FILTER_COEFF_4<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_1X<<5 | BMP280_OVERSAMP_4X<<2 | BMP280_NORMAL_MODE;
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas); 
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case DROP_DETECTION:
            data_config = BMP280_STANDBY_TIME_1_MS<<5 | BMP280_FILTER_COEFF_OFF<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_1X<<5 | BMP280_OVERSAMP_2X<<2 | BMP280_NORMAL_MODE;
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas); 
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case INDOOR_NAVIGATION:
            data_config = BMP280_STANDBY_TIME_1_MS<<5 | BMP280_FILTER_COEFF_16<<2 | 0x00;
            data_ctrl_meas = BMP280_OVERSAMP_2X<<5 | BMP280_OVERSAMP_16X<<2 | BMP280_NORMAL_MODE;
            printk("data_config: %x\n", data_config);
            printk("data_ctrl: %x\n", data_ctrl_meas); 
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, data_config);
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, data_ctrl_meas);
            break;
        case CONFIG_SETUP_0xF4:
            i2c_smbus_write_byte_data(bmp280_client, 0xF4, arg);
            break;
        case CONFIG_SETUP_0xF5:
            i2c_smbus_write_byte_data(bmp280_client, 0xF5, arg);
            break;
        default:
            printk("MODE: %d\n", cmd);
            return -EINVAL;
    }
    return 0;   
}

static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
    int32_t TEM;
    char out_string[100];

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(out_string));
    // Read data from MPU6050 sensor

    adc_T = (i2c_smbus_read_byte_data(bmp280_client, 0xFA) << 12) | (i2c_smbus_read_byte_data(bmp280_client, 0xFB) << 4) | (i2c_smbus_read_byte_data(bmp280_client, 0xFC) >> 4);
    adc_P = (i2c_smbus_read_byte_data(bmp280_client, 0xF7) << 12) | (i2c_smbus_read_byte_data(bmp280_client,0xF8) << 4) | (i2c_smbus_read_byte_data(bmp280_client, 0xF9) >> 4);

    t_fine = bmp280_compensate_T(adc_T);
    P = bmp280_compensate_P(adc_P, t_fine);

    TEM = (int32_t)((t_fine * 5 + 128) >> 8);
    snprintf(out_string, sizeof(out_string),"%d.%d\n%d.%d\n", TEM/100, TEM%100, P/100, P%100);
	/* Copy data to user */
	not_copied = copy_to_user(user_buffer, out_string, to_copy);
	/* Calculate data */
	delta = to_copy - not_copied;

	return delta;
}


static struct file_operations fops = {
	.open = driver_open,
	.release = driver_close,
    .unlocked_ioctl = bmp280_ioctl,
    .read = driver_read
};

static int bmp280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    // Set power management register to wake up device
    ret = i2c_smbus_read_byte_data(client, BMP280_REG_ID);
    printk("Chip ID: 0x%x\n",ret);
    if (ret != 0x58) {
        printk(KERN_ERR "Failed to wake up BMP\n");
        return -1;
    }
    
    bmp280_client = client;
    
    /*Register a major number*/
	major_number = register_chrdev(0,DEVICE_NAME,&fops);
    if (major_number<0) {
        printk(KERN_ALERT"Fail to register a major number\n");
        return -1;
    }
    /*create a class*/
	bmp280_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp280_class)){
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ALERT"fail to register device class\n");
        return PTR_ERR(bmp280_class);
    }
    /*BMP280 device*/
	bmp280_device = device_create(bmp280_class,NULL,MKDEV(major_number,0),NULL,DEVICE_NAME);
    if (IS_ERR(bmp280_device)){
        class_destroy(bmp280_class);
        unregister_chrdev(major_number,DEVICE_NAME);
        printk(KERN_ALERT"fail to create BMP280 device\n");
        return PTR_ERR(bmp280_device);
    }
    printk(KERN_INFO "BMP280 driver probed\n");
    read_calib(bmp280_client);
    return 0;
}

static void bmp280_remove(struct i2c_client *client)
{
    device_destroy(bmp280_class,MKDEV(major_number,0));
    class_unregister(bmp280_class);
    class_destroy(bmp280_class);
    unregister_chrdev(major_number,DEVICE_NAME);
    printk(KERN_INFO "BMP280 driver removed\n");

    // Clean up
}

// static const struct i2c_device_id bmp280_id[] = {
//     { "bmp280", 0 },
//     { }
// };
// MODULE_DEVICE_TABLE(i2c, bmp280_id);

// static struct i2c_driver bmp280_driver = {
//     .driver = {
//         .name   = DEVICE_NAME,
//         .owner  = THIS_MODULE,
//     },
//     .probe      = bmp280_probe,
//     .remove     = bmp280_remove,
//     .id_table   = bmp280_id,
// };

static const struct of_device_id bmp280_of_match[] = {
    { .compatible = "bmp280_1", },
    { },
};
MODULE_DEVICE_TABLE(of, bmp280_of_match);

static struct i2c_driver bmp280_driver = {
    .driver = {
        .name   = DEVICE_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp280_of_match),
    },
    .probe      = bmp280_probe,
    .remove     = bmp280_remove,
};


static int __init bmp280_Init(void) {
    printk("BMP280 I2C driver initialized\n");
    return i2c_add_driver(&bmp280_driver);
}

static void __exit bmp280_Exit(void) {

    printk(KERN_INFO"BMP280 I2C driver exited\n");
    i2c_del_driver(&bmp280_driver);
}

module_init(bmp280_Init);
module_exit(bmp280_Exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nhom bat on");
MODULE_DESCRIPTION("BMP280_set");

// bmp280@76 {
// 					compatible = "invensense,bmp280";
// 					reg = <0x76>;
// 				};