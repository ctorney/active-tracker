/**************************************************************************/

*/
/**************************************************************************/

#ifndef _STORAGE_H
#define _STORAGE_H

#include "tracker.h"

const byte PIN_FLASH_CS = 32; // 
#include <SPI.h>
#include <SparkFun_SPI_SerialFlash.h>

// flags to indicate whether the board had to reboot from the watchdog
#define WD_FLAG_TRUE 0xAA   
#define WD_FLAG_FALSE 0x00   


#define WD_FLAG_PTR 0       // location of wd flag
#define SAVE_COUNT_PTR 1    // location of counter for saved messages
#define SEND_COUNT_PTR 5    // location of counter for sent messages
#define TELEMETRY_PTR 9     // start of telemetry data storage

#define MAX_STORAGE = 

/// 8760 entries in flash - a full year 365*24 12 bytes for GPS, 49 bytes for imu

class Storage {
public:
  
  bool begin();

  Storage(); //
  virtual ~Storage();

  void wd_shutdown();

  void read_next_message();
  void write_next_message();

  

private:
  SFE_SPI_FLASH flashDrive;

  long save_count;
  long send_count;

  size_t location_size;
  size_t activity_size;

};
/**************************************************************************/

#endif