#pragma once


#ifndef _LORA_H
#define _LORA_H
#include <Arduino.h>
#include "tracker.h"
#include "storage.h"
#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization



class Lora {
public:
  
  bool begin();
  bool update(Storage* storage);

  bool join();
  bool send_message(LoraMessage message);
    


private:
  void sendQuery(String atstring);
  bool sendCommand(String atstring);
  bool lora_active = false;
  bool join_success = false;
  bool activate();
  void deactivate();
  String WBEEST_APP_KEY="434F4C494E5357494C44454245455354";

};


#endif
