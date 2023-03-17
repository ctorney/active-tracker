

#include <SerialFlash.h>
#include <SPI.h>
#include <MKRWAN.h>


// #include <MKRWAN_v2.h>

LoRaModem modem;

const byte PIN_FLASH_CS = 32; // Change this to match the Chip Select pin on your board
#include <SPI.h>
#include <SparkFun_SPI_SerialFlash.h>
SFE_SPI_FLASH myFlash;


int FCstate = 8;  //to see the state of the function


int sendLORA(float *pdata)
{
/* This sketch is intended to use with arduino MKR1310 boards only
 *  To test the initialization of the 2MB flash memory
 */



  pinMode(LORA_RESET, OUTPUT);  //LORA reset pin declaration as output
  digitalWrite(LORA_RESET, LOW);  //turn off LORA module
  myFlash.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // Begin the flash using the chosen CS pin. Default to: spiPortSpeed=2000000, spiPort=SPI and spiMode=SPI_MODE0
  if (myFlash.begin(PIN_FLASH_CS, 2000000, SPI1) == false)
  {
    Serial.println(F("SPI Flash not detected. Check wiring. Maybe you need to pull up WP/IO2 and HOLD/IO3? Freezing..."));
    while (1);
  }

  Serial.println(F("SPI Flash detected"));

  sfe_flash_manufacturer_e mfgID = myFlash.getManufacturerID();
  if (mfgID != SFE_FLASH_MFG_UNKNOWN)
  {
    Serial.print(F("Manufacturer: "));
    Serial.println(myFlash.manufacturerIDString(mfgID));
  }
  else
  {
    uint8_t unknownID = myFlash.getRawManufacturerID(); // Read the raw manufacturer ID
    Serial.print(F("Unknown manufacturer ID: 0x"));
    if (unknownID < 0x10) Serial.print(F("0")); // Pad the zero
    Serial.println(unknownID, HEX);
  }

  Serial.print(F("Device ID: 0x"));
  Serial.println(myFlash.getDeviceID(), HEX);

  
  return 0;

  /*while sent_count<fix_count
     if (millis() - hour_timer > lora_timeout)
      {
          break;
      }

*/

}
