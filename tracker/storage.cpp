#include "storage.h"


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

  return true;
}

void Storage::wd_shutdown(){
   
}

void Storage::checkStart(){

    if (flashDrive.begin(PIN_FLASH_CS, 2000000, SPI1) == false)
    {
      Serial.println(F("SPI Flash not detected.."));
    }
}  

void Storage::write_next_message(location_reading location, activity_reading activity)
{

    checkStart();
    if (save_count==MAX_STORAGE){
      save_count=0;
      send_count=0;
      flashDrive.erase();
    }


    int location_address = (save_count/2)*(size_of_location + size_of_activity);

    flashDrive.writeBlock(location_address,(uint8_t *)&location.start_time,4);
    flashDrive.writeBlock(location_address+4,(uint8_t *)&location.lat,4);
    flashDrive.writeBlock(location_address+8,(uint8_t *)&location.lon,4);

    int activity_address=  (save_count/2)*(size_of_location + size_of_activity) + (size_of_location);

    flashDrive.writeBlock(activity_address,(uint8_t *)&activity.start_time,4);
    flashDrive.writeBlock(activity_address+4,(uint8_t *)&activity.activities,sizeof(activity.activities));
    
    save_count+=2;

}

bool Storage::anything_to_send()
{
 
  if (send_count<save_count)
    return true;
  else
    return false;

}

void Storage::send_successful()
{
  send_count++;
   
  if ((send_count==save_count)&&(send_count>=RESET_THRESHOLD))
  {  
      save_count=0;
      send_count=0;
      checkStart();
      flashDrive.erase();
  }

}

LoraMessage Storage::read_next_message()
{

  checkStart();
  
  LoraMessage message;

  // delay a different amount of time between 0 and 30 seconds
  // so that collars don't try to send at the same moment
//  delay(1000*((save_count+send_count)%30));
  delay(500);
  

  if (send_count % 2 == 0)
  {
    int location_address = (send_count/2)*(size_of_location + size_of_activity);

    long location_time;
    float location_data[2];
    flashDrive.readBlock(location_address,(uint8_t *)&location_time,sizeof(location_time));
    flashDrive.readBlock(location_address + 4,(uint8_t *)&location_data,sizeof(location_data));
    
    message.addUnixtime(location_time);
    message.addLatLng(location_data[0],location_data[1]);
    
  }
  else
  {
    int activity_address = (send_count/2)*(size_of_location + size_of_activity) + (size_of_location);
    
    long start_time;
    byte activities[45];
    
    flashDrive.readBlock(activity_address,(uint8_t *)&start_time,sizeof(start_time));
    flashDrive.readBlock(activity_address+4,(uint8_t *)&activities,sizeof(activities));
    
    message.addUnixtime(start_time);
    for (int i=0;i<45;i++)
      message.addUint8(activities[i]);
  }
    
  return message;
    
}
