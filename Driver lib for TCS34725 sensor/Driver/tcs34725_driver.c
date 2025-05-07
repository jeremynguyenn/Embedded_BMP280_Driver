// Huynh Tan Luong   21146267
// Nguyen Chi Thanh  21146315
// Nguyen Anh Tuan   21146570
// Nguyen Minh Phuoc 21146297

// khai bao thu vien
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

// dinh nghia ten driver, class, dev
#define DRIVER_NAME "tcs34725_driver"
#define CLASS_NAME "tcs34725"
#define DEVICE_NAME "tcs34725"

// dinh nghia dia chi i2c
#define tcs34725_ADDR 0x29 // khong sai

// // dinh nghia thanh ghi
#define command_bit				0x80

#define tcs_enable       			0x00
#define tcs_atime         			0x01
#define tcs_wtime         			0x03
// 4 thanh ghi khong truyen gia tri
#define tcs_AILTL         			0x04
#define tcs_AILTH         			0x05
#define tcs_AIHTL         			0x06
#define tcs_AIHTH         			0x07

#define tcs_PPERS         			0x0C
#define tcs_CONFIG         			0x0D
#define tcs_control       			0x0F
#define tcs_ID       				0x12
#define tcs_status       			0x13

#define tcs_clear_low_data        		0x14
#define tcs_red_low_data        		0x16
#define tcs_green_low_data        		0x18
#define tcs_blue_low_data        		0x1A

// dinh nghi ky hieu qua lai giua kernal va user space
#define TCS34725_IOCTL_MAGIC 'm'
#define TCS34725_IOCTL_READ_CLEAR        _IOR(TCS34725_IOCTL_MAGIC, 1, int)
#define TCS34725_IOCTL_READ_RED          _IOR(TCS34725_IOCTL_MAGIC, 2, int)
#define TCS34725_IOCTL_READ_GREEN        _IOR(TCS34725_IOCTL_MAGIC, 3, int)
#define TCS34725_IOCTL_READ_BLUE         _IOR(TCS34725_IOCTL_MAGIC, 4, int)
#define TCS34725_IOCTL_SET_UP_ID         _IOR(TCS34725_IOCTL_MAGIC, 5, int)

#define TCS34725_IOCTL_SET_UP_ENABLE     _IOW(TCS34725_IOCTL_MAGIC, 0, int)
#define TCS34725_IOCTL_SET_UP_ATIME      _IOW(TCS34725_IOCTL_MAGIC, 1, int)
#define TCS34725_IOCTL_SET_UP_WTIME      _IOW(TCS34725_IOCTL_MAGIC, 2, int)
#define TCS34725_IOCTL_SET_UP_AILTL      _IOW(TCS34725_IOCTL_MAGIC, 3, int)
#define TCS34725_IOCTL_SET_UP_AILTH      _IOW(TCS34725_IOCTL_MAGIC, 4, int)
#define TCS34725_IOCTL_SET_UP_AIHTL      _IOW(TCS34725_IOCTL_MAGIC, 5, int)
#define TCS34725_IOCTL_SET_UP_AIHTH      _IOW(TCS34725_IOCTL_MAGIC, 6, int)
#define TCS34725_IOCTL_SET_UP_PPERS      _IOW(TCS34725_IOCTL_MAGIC, 7, int)
#define TCS34725_IOCTL_SET_UP_CONFIG     _IOW(TCS34725_IOCTL_MAGIC, 8, int)
#define TCS34725_IOCTL_SET_UP_GAIN       _IOW(TCS34725_IOCTL_MAGIC, 9, int)
#define TCS34725_IOCTL_SET_UP_STATUS     _IOW(TCS34725_IOCTL_MAGIC, 10, int)

static struct i2c_client *tcs34725_client;
static struct class* tcs34725_class = NULL;
static struct device* tcs34725_device = NULL;

static int major_number;
static int device_open_count = 0;

// ham doc gia tri duoi lop kernal
static int tcs34725_read_color(struct i2c_client *client, int color)
{
    s32 res,low,high;

	low = i2c_smbus_read_byte_data(client, command_bit | color);
    if (low < 0) {
        printk(KERN_ERR "Failed to read16 low\n");
        return -EIO;
    }
	high = i2c_smbus_read_byte_data(client, command_bit | ( color + 1));
	if (high < 0) {
        printk(KERN_ERR "Failed to read16 high\n");
        return -EIO;
    }
	res = (high << 8) | low ;

    return res;
}
// ham cau hinh thanh ghi
static int tcs34725_cau_hinh_enable(struct i2c_client *client, int val_enable)
{
	int ret;
    // cau hinh thanh ghi
    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_enable , val_enable);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up enable\n");
        return ret;
    }
    printk(KERN_INFO "TCS34725 enable set up successful\n");
	return 0;
}

static int tcs34725_cau_hinh_atime(struct i2c_client *client,int val_atime )
{
	int ret;
    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_atime , val_atime);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up atime\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 atime set up successful\n");
	return 0;
}

static int tcs34725_cau_hinh_wtime(struct i2c_client *client,int val_wtime )
{
	int ret;

    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_wtime , val_wtime);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up wtime\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 wtime set up successful\n");
	return 0;
}

//-------------------------------chu y kho viet nho doi ham cho 4 thanh ghi 0x04-0x07 do khong co truyen gia tri--------------
static int tcs34725_cau_hinh_ailtl(struct i2c_client *client)
{
	int ret;

    ret = i2c_smbus_write_byte(client,command_bit | tcs_AILTL);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up ailtlt\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 ailtl set up successful\n");
	return 0;
}
static int tcs34725_cau_hinh_ailth(struct i2c_client *client)
{
	int ret;

    ret = i2c_smbus_write_byte(client,command_bit | tcs_AILTH);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up ailtlh\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 ailth set up successful\n");
	return 0;
}
static int tcs34725_cau_hinh_aihtl(struct i2c_client *client)
{
	int ret;

    ret = i2c_smbus_write_byte(client,command_bit | tcs_AIHTL);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up aihtl\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 aihtl set up successful\n");
	return 0;
}
static int tcs34725_cau_hinh_aihth(struct i2c_client *client)
{
	int ret;

    ret = i2c_smbus_write_byte(client,command_bit | tcs_AIHTH);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up aihth\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 aihth set up successful\n");
	return 0;
}
//-----------------------------------------------------------------------------------

static int tcs34725_cau_hinh_ppers(struct i2c_client *client,int val_ppers )
{
	int ret;

    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_PPERS , val_ppers);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up ppers\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 ppers set up successful\n");
	return 0;
}


static int tcs34725_cau_hinh_config(struct i2c_client *client,int val_config )
{
	int ret;

    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_CONFIG  , val_config);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up config\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 config set up successful\n");
	return 0;
}

static int tcs34725_cau_hinh_gain(struct i2c_client *client,int val_gain)
{
	int ret;
    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_control, val_gain);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up gain\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 gain set up successful\n");
	return 0;
}

// thanh ghi doc
static int tcs34725_cau_hinh_id(struct i2c_client *client)
{
	int ret;
	
	ret = i2c_smbus_read_byte_data(client, command_bit | tcs_ID );
    if (ret < 0) {
        printk(KERN_ERR "Read ID ERROR\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 ID TRUE\n");
	return ret;
}

static int tcs34725_cau_hinh_status(struct i2c_client *client,int val_status)
{
	int ret;
    ret = i2c_smbus_write_byte_data(client,command_bit | tcs_status , val_status);
    if (ret < 0) {
        printk(KERN_ERR "Failed to wake up status\n");
        return ret;
    }
	printk(KERN_INFO "TCS34725 status set up successful\n");
	return 0;
}

static long tcs34725_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int data;

    printk(KERN_INFO "tcs34725_ioctl called with cmd=%u\n", cmd);

    if (!tcs34725_client) {
        printk(KERN_ERR "tcs34725_client is not initialized\n");
        return -ENODEV;
    }

    switch (cmd) {
        case TCS34725_IOCTL_READ_CLEAR:
			printk(KERN_INFO "Reading CLEAR\n");
			data = tcs34725_read_color(tcs34725_client, tcs_clear_low_data);
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				return -EFAULT;}
			break;
        case TCS34725_IOCTL_READ_RED:
			printk(KERN_INFO "Reading RED\n");
			data = tcs34725_read_color(tcs34725_client, tcs_red_low_data);
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				return -EFAULT;}
			break;
        case TCS34725_IOCTL_READ_GREEN:
			printk(KERN_INFO "Reading GREEN\n");
			data = tcs34725_read_color(tcs34725_client, tcs_green_low_data);
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				return -EFAULT;}
			break;
        case TCS34725_IOCTL_READ_BLUE:
			printk(KERN_INFO "Reading BLUE\n");	
			data = tcs34725_read_color(tcs34725_client, tcs_blue_low_data);
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				return -EFAULT;}
			break;

		case TCS34725_IOCTL_SET_UP_ID:
			printk(KERN_INFO "Reading id\n");      
			data = tcs34725_cau_hinh_id(tcs34725_client);
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				return -EFAULT;}
			break;


		case TCS34725_IOCTL_SET_UP_ENABLE:
		    printk(KERN_INFO "Sending enable\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_enable(tcs34725_client,data);
			break;
		case TCS34725_IOCTL_SET_UP_ATIME:
		    printk(KERN_INFO "Sending atime\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_atime(tcs34725_client,data);
			break;
		case TCS34725_IOCTL_SET_UP_WTIME:
		    printk(KERN_INFO "Sending wtime\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_wtime(tcs34725_client,data);
			break;
			   
		case TCS34725_IOCTL_SET_UP_AILTL:
		    printk(KERN_INFO "Sending AILTL\n");
			tcs34725_cau_hinh_ailtl(tcs34725_client);
			   break;
		case TCS34725_IOCTL_SET_UP_AILTH:
		    printk(KERN_INFO "Sending AILTL\n");
			   tcs34725_cau_hinh_ailth(tcs34725_client);
			   break;
		case TCS34725_IOCTL_SET_UP_AIHTL:
		    printk(KERN_INFO "Sending AIHTL\n");
			tcs34725_cau_hinh_aihtl(tcs34725_client);
			break;
		case TCS34725_IOCTL_SET_UP_AIHTH:
		    printk(KERN_INFO "Sending AIHTH\n");
			tcs34725_cau_hinh_aihth(tcs34725_client);
			break;
			   
		case TCS34725_IOCTL_SET_UP_PPERS:
		    printk(KERN_INFO "Sending ppers\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_ppers(tcs34725_client,data);
			break;
		case TCS34725_IOCTL_SET_UP_CONFIG:
		    printk(KERN_INFO "Sending config\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_config(tcs34725_client,data);
			break;
		case TCS34725_IOCTL_SET_UP_GAIN:
		    printk(KERN_INFO "Sending gain\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_gain(tcs34725_client,data);
			break;
		case TCS34725_IOCTL_SET_UP_STATUS:
		    printk(KERN_INFO "Sending status\n");
		    if(copy_from_user(&data, (int __user *)arg, sizeof(data)))	return -EFAULT;
			tcs34725_cau_hinh_status(tcs34725_client,data);
			break;
        default:
            return -EINVAL;
    }

    return 0;
}
static int tcs34725_open(struct inode *inodep, struct file *filep)
{
	// dam bao chi co the 1 file code c load dc driver
    if (device_open_count > 0)
        return -EBUSY;
    device_open_count++;
    try_module_get(THIS_MODULE);
    printk(KERN_INFO "tcs34725 device opened\n");
    return 0;
}
static int tcs34725_release(struct inode *inodep, struct file *filep)
{
	device_open_count--;
    module_put(THIS_MODULE);
    printk(KERN_INFO "tcs34725 device closed\n");
    return 0;
}
// dang ky giao tiep giua user space vs kernal
static struct file_operations fops = {
    .open = tcs34725_open,
    .unlocked_ioctl = tcs34725_ioctl,
    .release = tcs34725_release,
};
static int tcs34725_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    tcs34725_client = client;
    // dang ky driver voi cai tro character device
    // nhap 0 de cap so major_number tu dong khac voi cac so da co trong linux( tranh xung dot major_number)
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    // khai bao driver duoi dang class
    tcs34725_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(tcs34725_class)) {
        unregister_chrdev(major_number, DEVICE_NAME); // xay ra loi huy dang ky character divece
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(tcs34725_class);
    }

    // tao devive moi khong can tao devive truc tiep nua (tao trong dev)
    tcs34725_device = device_create(tcs34725_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(tcs34725_device)) {
        class_destroy(tcs34725_class); // huy dang ky class
        unregister_chrdev(major_number, DEVICE_NAME); // huy dang ky character device
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(tcs34725_device);
    }
    printk(KERN_INFO "tcs34725 driver installed\n");
    return 0;
}
static void tcs34725_remove(struct i2c_client *client)
{
// huy device
    device_destroy(tcs34725_class, MKDEV(major_number, 0));
// huy dang ky class
    class_unregister(tcs34725_class);
    class_destroy(tcs34725_class);
// huy character device
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "tcs34725 driver removed\n");
}
// dang ky tren he thong
static const struct of_device_id tcs34725_of_match[] = {
    { .compatible = "taos,tcs34725", },
    { },
};
MODULE_DEVICE_TABLE(of, tcs34725_of_match);
// khi sudo se tu dong chay het cac dong nay
static struct i2c_driver tcs34725_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(tcs34725_of_match),
    },
    .probe      = tcs34725_probe,
    .remove     = tcs34725_remove,
};
// ham ket noi driver voi i2c
static int __init tcs34725_init(void)
{
    printk(KERN_INFO "Initializing tcs34725driver\n");
    return i2c_add_driver(&tcs34725_driver);
}

// ham ngat ket noi
static void __exit tcs34725_exit(void)
{
    printk(KERN_INFO "Exiting tcs34725 driver\n");
    i2c_del_driver(&tcs34725_driver);
}

module_init(tcs34725_init);
module_exit(tcs34725_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("tcs34725 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");






















