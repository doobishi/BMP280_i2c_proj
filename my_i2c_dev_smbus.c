#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#define DRIVER_NAME "my_i2c_BMP280"

#define BMP280_REG_CHIP_ID   0xD0
#define BMP280_CHIP_ID_VAL   0x58
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_TEMP_MSB  0xFA

/* 
 * Configuration Byte Breakdown for 0xF4:
 * osrs_t (Temperature oversampling x1) : 001 (Bits 7:5)
 * osrs_p (Pressure skipped)            : 000 (Bits 4:2)
 * mode   (Forced mode, single measure) : 01  (Bits 1:0)
 * Binary: 00100001 -> Hex: 0x21
 */
#define BMP280_MEASURE_TEMP_ONLY 0x21

static dev_t bmp280_dev_num;
static struct class *bmp280_class;

/* Private device context structure */
struct bmp280_data {
    struct i2c_client *client;
    struct cdev cdev_bmp280;
};

static int trigger_and_wait_for_temperature(struct i2c_client *client);
static s32 get_temperature_alg(struct bmp280_data *data);
static s32 read_and_compensate_temp(struct bmp280_data *data);


static int bmp280_open(struct inode *inode, struct file *file)
{
    /* Map the private context structure to the file handle */
    struct bmp280_data *bmp280data = container_of(inode->i_cdev, struct bmp280_data, cdev_bmp280);
    file->private_data = bmp280data;
    return 0;
}

static int bmp280_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t bmp280_read(struct file *file, char __user *user_buffer, size_t count, loff_t *offset)
{
    struct bmp280_data *data = file->private_data;
    char temp_str[16];
    s32 temperature;
    int len;

    if (*offset > 0) return 0; /* Handle EOF */

    temperature = read_and_compensate_temp(data) ; 
    if (temperature == -EIO)
        return -EIO;

    len = snprintf(temp_str, sizeof(temp_str), "%d.%02d\n", temperature / 100, temperature % 100);

    if (copy_to_user(user_buffer, temp_str, len))
        return -EFAULT;

    *offset += len;
    return len;
}

static const struct file_operations bmp280_fops = {
    .owner   = THIS_MODULE,
    .open    = bmp280_open,
    .read    = bmp280_read,
    .release = bmp280_release,
};

static int my_i2c_probe(struct i2c_client *client)
{
    struct bmp280_data *bmp280data;
    s32 chip_id;
    int ret;

    /* Verify hardware presence */
    chip_id = i2c_smbus_read_byte_data(client, BMP280_REG_CHIP_ID);
    if (chip_id != BMP280_CHIP_ID_VAL ) {
        dev_err(&client->dev, "Invalid Chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }

    /* Allocate memory for the private context structure */
    bmp280data = devm_kzalloc(&client->dev, sizeof(struct bmp280_data), GFP_KERNEL);
    if (!bmp280data) return -ENOMEM;

    bmp280data->client = client;
    

    /* Initialize and add the character device */
    cdev_init(&bmp280data->cdev_bmp280, &bmp280_fops);
    bmp280data->cdev_bmp280.owner = THIS_MODULE;
    ret = cdev_add(&bmp280data->cdev_bmp280, bmp280_dev_num, 1);
    if (ret) {
        dev_err(&client->dev, "BMP280 Failed to add cdev\n");
        return ret;
    }

    /* Create the /dev/my_sensor node */
    device_create(bmp280_class, &client->dev, bmp280_dev_num, NULL, "my_bmp280");
    
    i2c_set_clientdata(client, bmp280data);
    dev_info(&client->dev, "BMP280 Probed and /dev/my_bmp280 created\n");

    return 0;
}

static int trigger_and_wait_for_temperature(struct i2c_client *client)
{
    s32 status;
    int loop_counts = 20; /* Maximum loops before failing */

    /* 1. Instruct the sensor to measure temperature */
    i2c_smbus_write_byte_data(client, BMP280_REG_CTRL_MEAS, BMP280_MEASURE_TEMP_ONLY);

    /* 2. Poll the status register until the measuring bit (Bit 3) clears */
    while (loop_counts > 0)
    {
        /* Sleep for 2 milliseconds to avoid locking the CPU */
        usleep_range(2000, 3000); 

        status = i2c_smbus_read_byte_data(client, BMP280_REG_STATUS);
        if (status < 0) {
            dev_err(&client->dev, "Failed to read status register\n");
            return status;
        }

        /* Bitwise AND with 0x08 (binary 00001000) to check Bit 3 */
        if ((status & 0x08) == 0) {
            dev_info(&client->dev, "Measurement finished!\n");
            return 0; /* Success */
        }

        loop_counts--;
    } 

    dev_err(&client->dev, "Sensor measurement timed out\n");
    return -ETIMEDOUT;
}

static s32 get_temperature_alg(struct bmp280_data *data) 
{
    struct i2c_client *client = data->client;
    s32 var1, var2, t_fine, temperature;
    s32 adc_T;
    int ret;
    
    u8 calib_buf[6]; /* FIX: Must be 6 bytes to hold T1, T2, and T3 */
    u8 raw_buf[3];   /* FIX: Separate buffer for the actual temperature reading */

    /* 1. Read Calibration Data (6 bytes starting at 0x88) */
    ret = i2c_smbus_read_i2c_block_data(client, 0x88, 6, calib_buf);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read sensor calibration data\n");
        return ret;
    }

    /* Convert Little-Endian bytes to CPU-endian 16-bit integers */
    data->dig_T1 = (u16)((calib_buf[1] << 8) | calib_buf[0]);
    data->dig_T2 = (s16)((calib_buf[3] << 8) | calib_buf[2]);
    data->dig_T3 = (s16)((calib_buf[5] << 8) | calib_buf[4]);

    /* 2. Read Raw Temperature Data (3 bytes starting at 0xFA) */
    ret = i2c_smbus_read_i2c_block_data(client, 0xFA, 3, raw_buf);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read raw temperature data\n");
        return ret;
    }

    /* 3. Reconstruct the 20-bit raw temperature value using raw_buf */
    adc_T = (raw_buf[0] << 12) | (raw_buf[1] << 4) | (raw_buf[2] >> 4);

    /* 4. Apply the Bosch Compensation Formula */
    var1 = ((((adc_T >> 3) - ((s32)data->dig_T1 << 1))) * ((s32)data->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((s32)data->dig_T1)) * ((adc_T >> 4) - ((s32)data->dig_T1))) >> 12) * ((s32)data->dig_T3)) >> 14;
    t_fine = var1 + var2;
    
    /* Final temperature scaled by 100 */
    temperature = (t_fine * 5 + 128) >> 8;
    
    dev_info(&client->dev, "Calculated Temperature: %d.%02d C\n",  temperature / 100, temperature % 100);
    return temperature;
}

static s32 read_and_compensate_temp(struct bmp280_data *data)
{
    int ret;
    
    ret = trigger_and_wait_for_temperature(data->client);
    if (ret < 0)
        return -EIO; /* Return I/O error if timeout or I2C fails */
        
    return get_temperature_alg(data);
}

static void my_i2c_remove(struct i2c_client *client)
{
    struct bmp280_data *data = i2c_get_clientdata(client);

    /* Teardown in reverse order of creation */
    device_destroy(bmp280_class, bmp280_dev_num);
    cdev_del(&data->cdev_bmp280);
    
    dev_info(&client->dev, "BMP280 Removed\n");
}

/* Device Tree matching table */
static const struct of_device_id my_i2c_of_match[] = {
    { .compatible = "custom,my-sensor" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, my_i2c_of_match);

static struct i2c_driver my_i2c_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = my_i2c_of_match,
    },
    .probe = my_i2c_probe,
    .remove = my_i2c_remove,
};

static int __init my_i2c_init(void)
{
    /* Allocate a major number dynamically for the character device */
    if (alloc_chrdev_region(&bmp280_dev_num, 0, 1, "my_sensor_dev") < 0)
        return -1;

    bmp280_class = class_create("my_sensor_class");
    if (IS_ERR(bmp280_class)) {
        unregister_chrdev_region(bmp280_dev_num, 1);
        return PTR_ERR(bmp280_class);
    }

    return i2c_add_driver(&my_i2c_driver);
}

static void __exit my_i2c_exit(void)
{
    i2c_del_driver(&my_i2c_driver);
    class_destroy(bmp280_class);
    unregister_chrdev_region(bmp280_dev_num, 1);
}

module_init(my_i2c_init);
module_exit(my_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("doobi-shih");
MODULE_DESCRIPTION("BMP280 I2C Character Device Driver");