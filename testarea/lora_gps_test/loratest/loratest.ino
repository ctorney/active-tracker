

//#define LORA_DEBUG Serial
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#include <RTCZero.h>


#define GPS_I2C_ADDRESS 0x10

#include <MKRWAN_v2.h>

LoRaModem modem;


Adafruit_GPS GPS(&Wire);


String devAddr;
String nwkSKey;
String appSKey;

// 
void setup() {

  
  Serial.begin(115200);
//  while (!Serial);

  delay(1000);  
  if (!GPS.begin(GPS_I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");

    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
      break;
    }
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  Serial.println("GPS setup");

  
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  modem.dataRate(0);
  modem.setADR(false);
  Serial.print("Your device data rate is: ");
  Serial.println(modem.getDataRate());
  Serial.print("Your device ADR is: ");
  Serial.println(modem.getADR());

//  modem.sleep();

  
}



unsigned long previousConnectionTime = 0;  
const unsigned long connectionInterval = 1000*60*10  ; // 10 minute intervals try and send

void loop() 
{
  if (GPS.available()) {
    char c = GPS.read();
  }
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
  
  
  unsigned long currentTime = millis();  

  if (currentTime - previousConnectionTime >= connectionInterval) 
  {
    send_lora();
    previousConnectionTime=millis();
  }
      
}

void send_lora()
{

  Serial.println("Connecting....");

     
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  
  if (!modem.begin(EU868)) {
    return;
  }
  
  String appEui = "0101010101010101";
  String appKey = "283E729D21445192C9A0E4B844D97838";
  String devEui = modem.deviceEUI();
  Serial.println("Sending message....");

  int connected = modem.joinOTAA(appEui, appKey, devEui);
  
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    
    return;
  }


    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
  int err;

  

  modem.beginPacket();

  if (GPS.hour < 10) { modem.print('0'); }
  modem.print(GPS.hour, DEC); modem.print(':');
  if (GPS.minute < 10) { modem.print('0'); }
  modem.print(GPS.minute, DEC); modem.print(':');
  if (GPS.seconds < 10) { modem.print('0'); }
  modem.print(GPS.seconds, DEC); modem.print(',');
  modem.print(GPS.latitude,6); modem.print(GPS.lat);
  modem.print(",");
  modem.print(GPS.longitude,6); modem.print(GPS.lon);
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly!");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    
  } else {
    Serial.println("Error sending message :(");
    Serial.println(err);
  }
  
  while (modem.available()) {
    Serial.write(modem.read());
  }
  modem.poll();

//  modem.sleep();

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    



}
