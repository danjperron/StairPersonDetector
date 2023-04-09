

#ifndef FLASH_SETTINGS
#define FLASH_SETTINGS

#include "pico/stdlib.h"
#include "mpu6050_i2c.h"



typedef struct{
  uint64_t ValidateFlash;
  float Gx_offset;
  float Gy_offset;
  float Gz_offset;
  float mpuThreshold;
  float udpThreshold;
  float peakThreshold;
  int lightDelay;
  uint8_t Mask[NSAMP/2];
} mpuSettingsStruct;

extern mpuSettingsStruct  mpuSettings;


void Write_Settings(void);
int Load_Settings(void);

#endif

