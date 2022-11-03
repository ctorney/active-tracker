


#include <RTCZero.h>
#include <Adafruit_GPS.h>

RTCZero rtc;
Adafruit_GPS GPS(&Wire);

#define I2C_ADDRESS 0x10

#define PMTK_ALWAYS_LOCATE "$PMTK225,9*22" ///< 115200 bps
#define PMTK_FULL_POWER "$PMTK225,0*2B" ///< 115200 bps
#define PMTK_ALWAYS_LOCATE_8 "$PMTK225,8*23" ///< 115200 bps



#define PMTK_LOG_INTERVAL "$PMTK187,1,5*38" // logger interval

#include <SPI.h>
#include <SD.h>


#include <Wire.h>

bool LED_BLINK = 0; 
const int chipSelect = SDCARD_SS_PIN;


unsigned long previousGPSTime = 0;  
const unsigned long GPSEventInterval = 10*1000; // 10 seconds
#define GPSECHO  true

#define filename "gpsdatalog.txt"

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)


  Serial.begin(9600);
  delay(2000);

  Serial.print("Initializing SD card...");
//*
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  
 
    


  GPS.begin(9600);

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
  //GPS.sendCommand(PMTK_LOG_INTERVAL);

  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);

  //Serial.println(filename);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //Serial.println(filename);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  //Serial.println(filename);
  set_time_from_gps();
  //rtc.begin(); // initialize RTC
  //rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
  //rtc.setAlarmTime(0, 0, 10);
  //rtc.enableAlarm(rtc.MATCH_HHMMSS);
  
  //rtc.attachInterrupt(daily_alarm);

    
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  //delay(100000);
  //GPS.standby();
  //rtc.standbyMode();
  //Serial.println("\nSTARTING LOGGING....");
  //if (GPS.LOCUS_StartLogger())
  //  Serial.println(" STARTED!");
  //else
  //  Serial.println(" no response :(");
  GPS.sendCommand(PMTK_FULL_POWER);
    GPS.sendCommand(PMTK_ALWAYS_LOCATE_8);

  
}


void set_time_from_gps()
{
  unsigned long startGPSTime = millis();  
  unsigned long scanGPSTime = 10*60*1000; // 10 minute scan time
  
  while (true)
  {

    char c = GPS.read();
    Serial.write(c);
    if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
    
    if (GPS.fix) {

      Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);


    
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      return;
    }

    unsigned long currentTime = millis();  
    if (currentTime - startGPSTime >= scanGPSTime) 
    {    
        GPS.standby();

        delay(1000*60*50); // wait 50 minutes for the next scan
        GPS.wakeup();

        unsigned long startGPSTime = millis();  

    
    }
  }
}




void loop() {
  
  
  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  unsigned long currentTime = millis();  

  if (currentTime - previousGPSTime >= GPSEventInterval) 
  {
    previousGPSTime = millis();
    output_gps();
  }
  
  

}


void output_gps()
{
    File dataFile = SD.open("gpslog.txt", FILE_WRITE);

    if (dataFile) {

    Serial.print("Writing to gpslog.txt...");

    

  
    dataFile.print("GPS,");  
    if (GPS.hour < 10) { dataFile.print('0'); }
    dataFile.print(GPS.hour, DEC); dataFile.print(':');
    if (GPS.minute < 10) { dataFile.print('0'); }
    dataFile.print(GPS.minute, DEC); dataFile.print(':');
    if (GPS.seconds < 10) { dataFile.print('0'); }
    dataFile.print(GPS.seconds, DEC); dataFile.print('.');
    if (GPS.milliseconds < 10) {
      dataFile.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      dataFile.print("0");
    }
    dataFile.print(GPS.milliseconds);
    dataFile.print(",");
    dataFile.print(GPS.latitude, 4); dataFile.print(GPS.lat);
    dataFile.print(",");
    dataFile.print(GPS.longitude, 4); dataFile.println(GPS.lon);
    dataFile.close();
    
    Serial.println("done.");

  } else {

    // if the file didn't open, print an error:

    Serial.println("error opening test.txt");

  }
      
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);


    
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}

void daily_alarm()
{
  
    digitalWrite(LED_BUILTIN, HIGH);
    GPS.wakeup();
    previousGPSTime = 0;  
    //previousWakeTime = millis();  

}
