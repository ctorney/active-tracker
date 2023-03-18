/**************************************************************************/


/**************************************************************************/

#ifndef _STORAGE_H
#define _STORAGE_H
#include <Arduino.h>
#include "tracker.h"

const byte PIN_FLASH_CS = 32; // 
#include <SPI.h>
#include <SparkFun_SPI_SerialFlash.h>
#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization

//// flags to indicate whether the board had to reboot from the watchdog
//#define WD_FLAG_TRUE 0xAA   
//#define WD_FLAG_FALSE 0x00   
//
//
//#define WD_FLAG_PTR 0       // location of wd flag
//#define SAVE_COUNT_PTR 1    // location of counter for saved messages
//#define SEND_COUNT_PTR 5    // location of counter for sent messages
//#define TELEMETRY_PTR 9     // start of telemetry data storage

#define MAX_STORAGE 365*24*2*2      // 2 measurements per hour so when we hit MAX_STORAGE we've been running for two years with a deficit between saved messages and transmitted messages
#define RESET_THRESHOLD 365*24*2    // if over RESET_THRESHOLD we'll clear the storage whenever we've sent all messages

/// 8760 entries in flash - a full year 365*24 12 bytes for GPS, 49 bytes for imu

class Storage {
public:
  
  bool begin();

//  Storage(); //
//  virtual ~Storage();

  void wd_shutdown();

  LoraMessage read_next_message();
  void write_next_message(location_reading location, activity_reading activity);
  void send_successful();
  bool anything_to_send();

  

private:
  SFE_SPI_FLASH flashDrive;

  long save_count;
  long send_count;


  int size_of_location = 12; 
  int size_of_activity = 49; 
  

};
/**************************************************************************/

#endif
