
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "mpu6050.h"
#include "hardware/i2c.h"
#include "mpu6050_i2c.h"



static int addr = 0x68;

int I2C_WByte(uint8_t reg, uint8_t value)
{
  uint8_t buf[2];
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  buf[0]=reg;
  buf[1]=value;
  return i2c_write_blocking (i2c_default, addr, buf,2, false);
}

int  I2C_RByte(uint8_t reg, uint8_t * value)
{
  uint8_t buf[1];
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  buf[0]=reg;
  i2c_write_blocking (i2c_default, addr, buf,1, true);
  return i2c_read_blocking(i2c_default,addr,value,1,false);
}


uint8_t mpu6050_status()
{
  uint8_t value;
  I2C_RByte(MPU6050_RA_INT_STATUS,&value);
  return value;
}


void mpu6050_reset() {
    int loop;
    int TheReg;

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t tbuf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, tbuf, 2, false);


    sleep_ms(1000);
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    //Sets sample rate to 8000/1+15 = 500Hz
    //I2C_WByte(MPU6050_RA_SMPLRT_DIV,15);
    //Sets sample rate to 8000/1+7 = 1000Hz
    I2C_WByte(MPU6050_RA_SMPLRT_DIV,(8000/FSAMP) - 1);
    //Disable gyro self tests, scale of 500 degrees/s
    I2C_WByte(MPU6050_RA_GYRO_CONFIG, 0b00001000);
//    //Disable accel self tests, scale of +-2g, no DHPF
    I2C_WByte(MPU6050_RA_ACCEL_CONFIG, 0x0);
    // set +/- 4g
    // I2C_WByte(MPU6050_RA_ACCEL_CONFIG, 0x08);


    //Sets clock source to gyro reference w/ PLL
    I2C_WByte(MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    I2C_WByte(MPU6050_RA_PWR_MGMT_2, 0x00);
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address
    I2C_WByte(MPU6050_RA_INT_ENABLE, 0x01);

    printf("\r\nMPU6050 Setup Complete\r\n");
}

void mpu6050_read_acc(int16_t accel[3])
{
   uint8_t buffer[6];
   uint8_t val = 0x3B;
   i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
   i2c_read_blocking(i2c_default, addr, buffer, 6, false);
   for(int i=0;i<3;i++)
       accel[i] = buffer[i*2]<<8 | buffer[(i*2)+1];
}


