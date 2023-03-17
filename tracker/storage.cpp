#include storage.h

storage::storage() {
  // I guess nothing ...
  
}

bool storage::begin() {
  
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


  byte wd_flag = flashDrive.readByte(WD_FLAG_PTR);

  if (wd_flag==WD_FLAG_TRUE)
  {
    flashDrive.writeByte(WD_FLAG_PTR)
    flashDrive.readBlock(SAVE_COUNT_PTR,&save_count,sizeof(save_count))
    flashDrive.readBlock(SEND_COUNT_PTR,&send_count,sizeof(send_count))
  }
  else
  {
    save_count=0;
    send_count=0;
  }
  flashDrive.writeByte(WD_FLAG_PTR, WD_FLAG_FALSE);

  return true;
}

bool storage::wd_shutdown(){
  flashDrive.writeByte(WD_FLAG_PTR, WD_FLAG_TRUE);

  flashDrive.writeBlock(SAVE_COUNT_PTR,&save_count,sizeof(save_count))
  flashDrive.writeBlock(SEND_COUNT_PTR,&send_count,sizeof(send_count))
    
}
