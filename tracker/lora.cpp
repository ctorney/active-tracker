#include "lora.h"


bool Lora::begin() {


return true;
}



bool Lora::update(Storage* storage) {

 if (!storage->anything_to_send())
 {
  deactivate();
  return false;
 }

 LoraMessage message = storage->read_next_message();
 Serial.println("Start sending..");   
  for (int i=0;i<message.getLength();i++){
    Serial.println(message.getBytes()[i],BIN);   
  }
 Serial.println("End of message");   

 storage->send_successful();
return true;
}

void Lora::deactivate() {


return;
}
