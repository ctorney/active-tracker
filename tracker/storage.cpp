#include "storage.h"

//Storage::Storage() {
//  // I guess nothing ...
//  
//}

bool Storage::begin() {
  
  if (flashDrive.begin(PIN_FLASH_CS, 2000000, SPI1) == false)
  {
    Serial.println(F("SPI Flash not detected.."));
    return false;
  }

  Serial.println(F("SPI Flash detected"));

  sfe_flash_manufacturer_e mfgID = flashDrive.getManufacturerID();
  if (mfgID != SFE_FLASH_MFG_UNKNOWN)
  {
    Serial.print(F("Manufacturer: "));
    Serial.println(flashDrive.manufacturerIDString(mfgID));
  }
  else
  {
    uint8_t unknownID = flashDrive.getRawManufacturerID(); // Read the raw manufacturer ID
    Serial.print(F("Unknown manufacturer ID: 0x"));
    if (unknownID < 0x10) Serial.print(F("0")); // Pad the zero
    Serial.println(unknownID, HEX);
  }

  Serial.print(F("Device ID: 0x"));
  Serial.println(flashDrive.getDeviceID(), HEX);

    // can't keep anything after a reboot unfortunately
    flashDrive.erase();
//
//    for (int x = 0 ; x < 10 ; x++)
//    {
//      
//
//      byte val = flashDrive.readByte(x);
//      if (val < 0x10) Serial.print("0");
//      Serial.println(val, HEX);
//      
//    }
//
//    Serial.println("============");
//    uint8_t myVal[4] = {0xDE, 0xAD, 0xBE, 0xEF};
//    for (int x = 0 ; x < 10 ; x += 4)
//    {
//      flashDrive.writeBlock(x, myVal, 4); //Address, pointer, size
//    }
//
//    for (int x = 0 ; x < 10 ; x++)
//    {
//      byte val = flashDrive.readByte(x);
//      if (val < 0x10) Serial.print("0");
//      Serial.println(val, HEX);
//    }
//
////    uint8_t myVal2[4] = {0x00, 0x00, 0x00, 0x00};
////    for (int x = 0 ; x < 0x0004 ; x += 4)
////    {
////      flashDrive.writeBlock(x, myVal2, 4); //Address, pointer, size
////    }
//  byte wd_flag = flashDrive.readByte(WD_FLAG_PTR);
//      Serial.print("flag: ");
//  Serial.println(wd_flag);
//
//  if (wd_flag==WD_FLAG_TRUE)
//  {
//
//      Serial.println("reading from flash");
//
////    flashDrive.writeByte((uint32_t)WD_FLAG_PTR, )
//    flashDrive.readBlock(SAVE_COUNT_PTR,(uint8_t *)&save_count,sizeof(save_count));
//    flashDrive.readBlock(SEND_COUNT_PTR,(uint8_t *)&send_count,sizeof(send_count));
//  }
//  else
//  {
//    save_count=0;
//    send_count=0;
//  }
//  flashDrive.writeByte(WD_FLAG_PTR, WD_FLAG_FALSE);
//  Serial.print("save count : ");
//  Serial.println(save_count);
//Serial.print("send count : ");
//  Serial.println(send_count);
  return true;
}

void Storage::wd_shutdown(){
//  flashDrive.writeByte(WD_FLAG_PTR, WD_FLAG_TRUE);
//
//  flashDrive.writeBlock(SAVE_COUNT_PTR,(uint8_t *)&save_count,sizeof(save_count));
//  flashDrive.writeBlock(SEND_COUNT_PTR,(uint8_t *)&send_count,sizeof(send_count));
    
}


void Storage::write_next_message()
{

    if (save_count==MAX_STORAGE){
      save_count=0;
      send_count=0;
      flashDrive.erase();
    }

//  location_address=   (save_count/2)*sizeof(page)
//  acivity_address=   (save_count/2)*sizeof(page) + sizeof(location)
    // write location at loc address
    
    save_count+=2;
    

    
}

bool Storage::anything_to_send()
{
  return (send_count<save_count)
}

void Storage::send_successful()
{
  send_count++;
  if ((send_count==save_count)&&(send_count==RESET_THRESHOLD))
  {
      save_count=0;
      send_count=0;
      flashDrive.erase();
  }

}

void Storage::read_next_message()
{

//  location_address=   save_count*sizeof(page)
//  acivity_address=   save_count*sizeof(page) + sizeof(location)
    // write location at loc address
    if (send_count % 2 == 0)
    {
        // sending location 
        //  location_address=   (send_count/2)*sizeof(page)
        // build lora message
    }
    else{

              // sending location 
        //  activity_address=   (send_count/2)*sizeof(page) + sizeof(location)
    
    }
    

    
}
