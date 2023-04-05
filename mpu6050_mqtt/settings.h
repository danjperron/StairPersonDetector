

#ifndef FLASH_SETTINGS
#define FLASH_SETTINGS

#include "pico/stdlib.h"



typedef struct{
  uint64_t ValidateFlash;
  float Gx_offset;
  float Gy_offset;
  float Gz_offset;
  float mpuThreshold;
  float udpThreshold;
  float peakThreshold;
  int lightDelay;
} mpuSettingsStruct;

extern mpuSettingsStruct  mpuSettings;


void Write_Settings(void);
int Load_Settings(void);

#endif

