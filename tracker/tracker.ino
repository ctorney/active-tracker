
#include <Wire.h>
#include <RTCZero.h>
#include "WDTZero.h"

#include "storage.h"
#include "classifier.h"
#include "lora.h"

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);


#define I2C_ADDRESS 0x10

#define PMTK_ALWAYS_LOCATE "$PMTK225,9*22" 
#define PMTK_BACKUP_MODE "$PMTK225,4*2F" 
#define PMTK_ALWAYS_LOCATE_8 "$PMTK225,8*23" 
#define GPS_WAKE_PIN 0

#define MAX_HDOP 1.0


RTCZero rtc;
WDTZero wdt; 

Storage storage;
Classifier classifier;
Lora lora;

bool GPS_ACTIVE = false;
bool IMU_ACTIVE = false;
bool LORA_ACTIVE = false;


unsigned long gps_run_time = 1000*60*10;  // gps will run for up to 10 minutes to get a fix
unsigned long lora_run_time = 1000*60*20; // lora will broadcast for up to 20 minutes every hour


unsigned long imu_interval = 200; // update imu every 200ms for 5hz readings
unsigned long last_imu_time = 0;

unsigned long gps_check_interval = 60*1000; // check the gps fix every minute  
unsigned long gps_last_check_time = 0;  

unsigned long gps_start_time = 0;  
unsigned long lora_start_time = 0; 

void setup() 
{
  
  Serial.begin(115200);
  delay(4000);
  Serial.println("\n\n*****************\n*****************\n");
  
  if (!lora.begin()) 
  {
    Serial.println("ERROR: lora");
  }
  delay(500);
  
  if (!storage.begin()) 
  {
    Serial.println("ERROR: storage");
  }
  delay(500);
      
  if (!classifier.begin()) 
  {
    Serial.println("ERROR: classifier");
  }
  delay(500);

  if (!GPS.begin(I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");
  }
  delay(500);
  
  pinMode(GPS_WAKE_PIN, OUTPUT);
  digitalWrite(GPS_WAKE_PIN, HIGH);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  
 
  rtc.begin(); // initialize RTC
  rtc.setAlarmTime(0, 0, 0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  delay(500);

  for (int i = 0; i < 10; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);   // 10 flashes indicate success
    delay(200);                       
    digitalWrite(LED_BUILTIN, LOW);    
    delay(200);
  }
  
  
 }



void wake_gps()
{
   Serial.println("waking up gps");
   gps_start_time = millis();
   gps_last_check_time = millis(); 
   digitalWrite(GPS_WAKE_PIN, HIGH);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
   GPS.fix = false;
   GPS.HDOP = 2.0*MAX_HDOP;
   GPS.satellites = 0;
}

void pause_gps()
{
   Serial.println("pausing gps");
   digitalWrite(GPS_WAKE_PIN, LOW);    
   GPS.sendCommand(PMTK_BACKUP_MODE);
}

bool waiting_on_first_fix = true;

void update_time()
{

  if ((GPS.satellites > 10)||(waiting_on_first_fix))
  {
      rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
      rtc.setDate(GPS.day, GPS.month, GPS.year);
      waiting_on_first_fix = false;
  }
}

unsigned int imu_count;

location_reading latest_location;

void loop() 
{

  
    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE))
    {
      // if nothing active then we are at the top of the hour so wake up the gps and imu and reset variables
      wake_gps();
      GPS_ACTIVE=true;
      IMU_ACTIVE=true;
      gps_start_time = millis();
      latest_location.lat = 0.0f;
      latest_location.lon = 0.0f;
      lora.session_success = false;
      
      wdt.setup(WDT_SOFTCYCLE8M);  // initialize WDT-softcounter refesh cycle on 8m interval
      classifier.activate(rtc.getEpoch());

    }


    if (GPS_ACTIVE)
    {
      char c = GPS.read();
      if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

      if ((millis() - gps_last_check_time) >= gps_check_interval) // Check fix quality every minute
      {
        if ( ( (GPS.fix) && (GPS.HDOP < MAX_HDOP) ) || ( millis() - gps_start_time >= gps_run_time) )
        {
  
           // either we got a fix or we're out of time
           if (GPS.fix)
           {
              latest_location.lat = GPS.latitudeDegrees;
              latest_location.lon = GPS.longitudeDegrees;
              update_time();
           }
  
           
           latest_location.start_time = rtc.getEpoch();
           GPS_ACTIVE=false;
           pause_gps();
        }
        gps_last_check_time = millis(); 

      }
    }

    if (IMU_ACTIVE)
    {
      if ((millis() - last_imu_time) >= imu_interval) //To stream at 5 Hz without using additional timers
      {



        last_imu_time = millis();
        IMU_ACTIVE = classifier.update();
      }
    }


    if ((!IMU_ACTIVE) && (!LORA_ACTIVE))
    {
      // add the readings to storage
      Serial.println("writing to storage...");

      storage.write_next_message(latest_location, classifier.latest_activity);
      Serial.println("write complete.");

      // turn on lora
      LORA_ACTIVE=true;
      lora_start_time = millis();
     
    }
    
    if (LORA_ACTIVE)
    {

     LORA_ACTIVE = lora.update(&storage);
     if (millis() - lora_start_time >= lora_run_time) 
     {
      LORA_ACTIVE=false;
     }

    }
    
    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {      
      Serial.println("going to sleep...");
      wdt.setup(WDT_OFF);  //watchdog 
      rtc.standbyMode();
    }

    wdt.clear();
   
  


    
}
