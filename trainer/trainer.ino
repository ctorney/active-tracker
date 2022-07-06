


#include <RTCZero.h>
#include <Adafruit_GPS.h>

RTCZero rtc;
Adafruit_GPS GPS(&Wire);

#define I2C_ADDRESS 0x10


#include <SPI.h>
#include <SD.h>




#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;



#include <Wire.h>

bool LED_BLINK = 0; 
const int chipSelect = SDCARD_SS_PIN;


unsigned long previousIMUTime = 0;  
const unsigned long IMUEventInterval = 200; // 200 milliseconds

unsigned long previousGPSTime = 0;  
const unsigned long GPSEventInterval = 10*1000; // 10 seconds

unsigned long previousWakeTime = 0;  
const unsigned long wakeInterval = 12*60*60*1000; // 12 hours

String filename = "datalog";

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)


  Serial.begin(9600);
  delay(2000);

  Serial.print("Initializing SD card...");
/*
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  // create the filename
  File root = SD.open("/");
  int fileCountOnSD = 0;
  while (true) {

    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    
    // for each file count it
    fileCountOnSD++;
  }
  Serial.println(fileCountOnSD);
  filename += String(fileCountOnSD);
  filename += String(".txt");
  Serial.println(filename);
*/
  if (!icm.begin_I2C()) {
      Serial.println("ERROR: ICM ");
      return;
    }


  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);

  rtc.begin(); // initialize RTC

  rtc.setAlarmTime(0, 15, 0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  
  rtc.attachInterrupt(daily_alarm);

    
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  
}

void loop() {
  
  
  unsigned long currentTime = millis();  
  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  /* This is the event */
  if (currentTime - previousIMUTime >= IMUEventInterval) 
  {
    previousIMUTime = millis();
    output_imu();
  }
  if (currentTime - previousGPSTime >= GPSEventInterval) 
  {
    previousGPSTime = millis();
    output_gps();
  }
  if (currentTime - previousWakeTime >= wakeInterval) 
  {
    GPS.standby();
    rtc.standbyMode();
  }
  

}

void output_imu()
{

    icm.getEvent(&accel, &gyro, &temp);
    float gx = float(gyro.gyro.x);
    float gy = float(gyro.gyro.y); 
    float gz = float(gyro.gyro.z); 
    float ax = float(accel.acceleration.x);
    float ay = float(accel.acceleration.y);
    float az = float(accel.acceleration.z);
    

    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);

 /*   File dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",");
    dataFile.print(pitch);
    dataFile.print(",");
    dataFile.print(x);
    dataFile.print(",");
    dataFile.print(y);
    dataFile.print(",");
    dataFile.println(z);

    dataFile.close();*/
//    LED_BLINK = !LED_BLINK;
//    digitalWrite(LED_BUILTIN, LED_BLINK);    // turn the LED off by making the voltage LOW

}

void output_gps()
{
      
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
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}

void daily_alarm()
{
    previousIMUTime = 0;  
    previousGPSTime = 0;  
    previousWakeTime = millis();  

}
