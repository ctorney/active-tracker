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

  
  
  void deactivate();


private:


};


#endif
