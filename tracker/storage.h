/**************************************************************************/


/**************************************************************************/

#ifndef _STORAGE_H
#define _STORAGE_H
#include <Arduino.h>
#include "tracker.h"


#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization


class Storage {
public:
  
  void begin();
  void sleep();
  LoraMessage read_next_message();
  void write_next_message(location_reading location, activity_reading activity);
  void send_successful();
  bool anything_to_send();

  uint8_t message_buffer[12+49];
  

private:

  int size_of_location = 12; 
  int size_of_activity = 49; 
  
  int record_size = 12+49;


  uint16_t currentSectorIndex;
  uint16_t currentRecordIndex; 

};
/**************************************************************************/

#endif
