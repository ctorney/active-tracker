

#include <Wire.h>
#include <RTCZero.h>
#include "WDTZero.h"

#include "storage.h"
#include "classifier.h"
#include "lora.h"
#include "ArduinoLowPower.h"


#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);


#define I2C_ADDRESS 0x10

#define PMTK_BACKUP_MODE "$PMTK225,4*2F" 
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

unsigned long start_time = 0;  
unsigned long lora_start_time = 0; 

void setup() 
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.begin(115200);
  delay(4000);
  Serial.println("\n\n*****************\n*****************\n");

  if (!lora.begin()) 
  {
    Serial.println("ERROR: lora");
  }
  delay(500);


  delay(500);

  if (!classifier.begin()) 
  {
    Serial.println("ERROR: classifier");
  }
  delay(500);



  pinMode(GPS_WAKE_PIN, OUTPUT);
  digitalWrite(GPS_WAKE_PIN, HIGH);
  delay(1000); 
  digitalWrite(GPS_WAKE_PIN, LOW);

  if (!GPS.begin(I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");
  }
  delay(500);
    
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);

  rtc.begin(); // initialize RTC
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
  gps_last_check_time = millis(); 
  digitalWrite(GPS_WAKE_PIN, HIGH);
  delay(1000);
  digitalWrite(GPS_WAKE_PIN, LOW);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  GPS.fix = false;
  GPS.HDOP = 2.0*MAX_HDOP;
  GPS.satellites = 0;
}

void pause_gps()
{
  GPS.sendCommand(PMTK_BACKUP_MODE);
}

bool waiting_on_first_fix = true;

void update_time()
{

  if ((GPS.satellites > 10)||(waiting_on_first_fix))
  {
    if((GPS.year > 2020) && (GPS.year < 2079)) {                                       
      rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
      rtc.setDate(GPS.day, GPS.month, GPS.year);
      waiting_on_first_fix = false;
    }
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
      start_time = millis();
      latest_location.lat = 0.0f;
      latest_location.lon = 0.0f;
      lora.session_success = false;
      
      wdt.setup(WDT_SOFTCYCLE16M);  // initialize WDT-softcounter refesh cycle on 16 minute interval
      classifier.activate(rtc.getEpoch());

    }


    if (GPS_ACTIVE)
    {
      char c = GPS.read();
      if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

      if ((millis() - gps_last_check_time) >= gps_check_interval) // Check fix quality every minute
      {
        if ( ( (GPS.fix) && (GPS.HDOP < MAX_HDOP) ) || ( millis() - start_time >= gps_run_time) )
        {

//          Serial.print("\nTime: ");
//          if (GPS.hour < 10) { Serial.print('0'); }
//          Serial.print(GPS.hour, DEC); Serial.print(':');
//          if (GPS.minute < 10) { Serial.print('0'); }
//          Serial.print(GPS.minute, DEC); Serial.print(':');
//          if (GPS.seconds < 10) { Serial.print('0'); }
//          Serial.println(GPS.seconds, DEC);
//          Serial.print("Location: ");
//          Serial.print(GPS.latitudeDegrees); Serial.print(GPS.lat);
//          Serial.print(", ");
//          Serial.print(GPS.longitudeDegrees); Serial.println(GPS.lon);
       
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
      classifier.deactivate();
      Serial.println("writing to storage...");
      lora.deactivate();
      storage.begin();
      storage.write_next_message(latest_location, classifier.latest_activity);
      storage.sleep();
      lora.activate();
      Serial.println("write complete.");

      // turn on lora
      LORA_ACTIVE=true;
      lora_start_time = millis();
     
    }
    
    if (LORA_ACTIVE)
    {
      LORA_ACTIVE = lora.update(&storage);
      if (millis() - lora_start_time >= lora_run_time) 
        LORA_ACTIVE=false;
    }

    wdt.clear();

    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {      
      

      
      wdt.setup(WDT_OFF);  //watchdog 
      unsigned long run_time = millis() - start_time;
      Serial.println("going to sleep...");
      Serial.print("back in ");
      Serial.print(60*60*1000 - run_time);
      Serial.println(" milliseconds.");

      LowPower.deepSleep(60*60*1000 - run_time);
      
    }

   
  


    
}
