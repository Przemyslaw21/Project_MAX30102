#include "max30102.h"
#include <stdio.h>
#include <zephyr/drivers/i2c.h>




// int MAX30102_Init(const struct device *i2c0_dev)
// {
//     uint8_t BufToRead[1] = {0x0};
//     int err = i2c_burst_read(i2c0_dev, MAX30102_ADDRESS, MAX30102_PARTID, BufToRead, 1);


//     if (err)
//     {
//         printk("MAX30102 disconnected!!!\n");
//     }
//     else
//     {
//         if (BufToRead[0] == 0x15)
//         {
//             printk("MAX30102 connected! Part ID: 0x%x\n", BufToRead[0]);
//         }
//         else
//         {
//             printk("MAX30102 wrong ID! Part ID: %dd\n", BufToRead[0]);
//         }
//     }
//     return err;

// }

// static int MAX30102_Reset(const struct device *dev_i2c)// Reset MAX30102
// {
//     uint8_t BufToWrite[1] = {MAX30102_RESET};
//     uint8_t err = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, BufToWrite[0]);

//     if (err)
//     {
//         printk("Reset error!\n");
//         return MAX30102_ERROR;
//     }
//     else
//     {
//         printk("Reset done!\n");
//         return MAX30102_OK;
//     }
// }

// static int MAX30102_Clear(const struct device *dev_i2c) // Reads/clears the interrupt status register
// {
//     uint8_t IntStatus1[1] = {0x0};
//     int ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, MAX30102_IRQ_STATUS_1, IntStatus1, 1);

//     if (ret)
//         printk("MAX30102: Interrupt Status 1: Error!\n");
//     else
//         printk("MAX30102: Interrupt Status 1: Ok (%d)\n", IntStatus1[0]);
//     return ret;
// }

// static int MAX30102_Config(const struct device *dev_i2c)
// {
//     uint8_t buf[1] = {0x00};
//     int ret, ret1 = 0;

//     buf[0] = 0xC0;
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_IRQ_ENABLE_1, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x00;
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_IRQ_ENABLE_2, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x00;
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_WRITE_PTR, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x00;
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_OVERFLOW_COUNT, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x00;
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_READ_PTR, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x0F; // sample avg = 1, fifo rollover=false, fifo almost full = 17
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, buf[0]);

//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x03; // 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x27; // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x24; // LED1 current ~ 7mA
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_LED1_PULSE_AMPLITUDE, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }

//     buf[0] = 0x24; // LED2 current ~ 7mA
//     ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_LED2_PULSE_AMPLITUDE, buf[0]);
//     if (ret)
//     {
//         ret1 = 0;
//     }
//     return ret1;
// }

// static int MAX30102_Read_Reg(const struct device *dev_i2c, uint8_t uch_addr, uint8_t *puch_data)
// {
//     int ret;
//     uint8_t buf[1] = {0x00};

//     ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, uch_addr, buf, 1);
//     if (ret)
//     {
//         printk("MAX30102: Error!\n");
//         return false;
//     }
//     else
//     {
//         *puch_data = buf[0];
//         return true;
//     }
// }

// static int MAX30102_Read_Fifo(const struct device *dev_i2c, uint32_t *pun_red_led, uint32_t *pun_ir_led)
// {
//     int ret;
//     uint32_t un_temp;
//     unsigned char uch_temp;
//     *pun_red_led = 0;
//     *pun_ir_led = 0;
//     char ach_i2c_data[6];

//     // read and clear status register
//     maxim_max30102_read_reg(dev_i2c, MAX30102_IRQ_STATUS_1, &uch_temp);
//     maxim_max30102_read_reg(dev_i2c, MAX30102_IRQ_STATUS_2, &uch_temp);

//     ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA, ach_i2c_data, 6);
//     if (ret)
//         printk("MAX30102: maxim_max30102_read_fifo Error!\n");

//     un_temp = (unsigned char)ach_i2c_data[0];
//     un_temp <<= 16;
//     *pun_red_led += un_temp;
//     un_temp = (unsigned char)ach_i2c_data[1];
//     un_temp <<= 8;
//     *pun_red_led += un_temp;
//     un_temp = (unsigned char)ach_i2c_data[2];
//     *pun_red_led += un_temp;

//     un_temp = (unsigned char)ach_i2c_data[3];
//     un_temp <<= 16;
//     *pun_ir_led += un_temp;
//     un_temp = (unsigned char)ach_i2c_data[4];
//     un_temp <<= 8;
//     *pun_ir_led += un_temp;
//     un_temp = (unsigned char)ach_i2c_data[5];
//     *pun_ir_led += un_temp;
//     *pun_red_led &= 0x03FFFF;
//     *pun_ir_led &= 0x03FFFF;

//     return true;
// }