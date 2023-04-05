#include "settings.h"

#include <hardware/flash.h>
#include <string.h>
#include "hardware/sync.h"


#define VALIDATE_FLASH_ID  0x18812442E77EC33C
// use the last 4K of flash memory
#define SETTINGS_FLASH_OFFSET 0x1ff000
#define SETTINGS_FLASH_SIZE 4096


union _FlashSettings
{
  mpuSettingsStruct  settings;
  uint8_t block[SETTINGS_FLASH_SIZE];
}FlashSettings;


mpuSettingsStruct  mpuSettings;


void Write_Settings(void)
{
  int loop=0;
  uint32_t ints = save_and_disable_interrupts();

  // first erase flash
   for(loop=0;loop<SETTINGS_FLASH_SIZE;loop+= FLASH_SECTOR_SIZE)
     flash_range_erase(SETTINGS_FLASH_OFFSET+loop,FLASH_SECTOR_SIZE);
  // ok write by page size
  memcpy(FlashSettings.block,&mpuSettings, SETTINGS_FLASH_SIZE);
  for(loop=0;loop<SETTINGS_FLASH_SIZE;loop+= FLASH_PAGE_SIZE)
     flash_range_program(SETTINGS_FLASH_OFFSET+loop,&FlashSettings.block[loop],FLASH_PAGE_SIZE);

  restore_interrupts (ints);
}



// set default settings
int Load_Settings(void)
{
  // load flash
  // we will use the last 4K memory
  char * Pt = (char *) (XIP_BASE + SETTINGS_FLASH_OFFSET);

  // transfer the flash memory to the FlashSettings union
  memcpy(FlashSettings.block, Pt, SETTINGS_FLASH_SIZE);

  // ok is the flash Valid

  if(FlashSettings.settings.ValidateFlash != VALIDATE_FLASH_ID)
  {
   // ok it is invalid then put default
   mpuSettings.ValidateFlash = VALIDATE_FLASH_ID;
   mpuSettings.Gx_offset = 0.0;
   mpuSettings.Gy_offset = 0.0;
   mpuSettings.Gz_offset = 0.0;
   mpuSettings.mpuThreshold = 20.0;
   mpuSettings.udpThreshold = 20.0;
   mpuSettings.peakThreshold = 150.0;
   mpuSettings.lightDelay = 30;
   Write_Settings();
   return 1;
  }
  // ok data is valid t
  memcpy(&mpuSettings,FlashSettings.block, sizeof(mpuSettingsStruct));
  return 1;
}

