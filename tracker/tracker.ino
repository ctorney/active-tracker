
#include <Wire.h>



#include "storage.h"
#include "classifier.h"

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);


#define SECRET_APP_EUI "xxxxxxxxxxxxx"
#define SECRET_APP_KEY "yyyyyyyyyyyyyyyyyyyyyyy"

#define I2C_ADDRESS 0x10

uint32_t timer = millis();

#define PMTK_ALWAYS_LOCATE "$PMTK225,9*22" 
#define PMTK_BACKUP_MODE "$PMTK225,4*2F" 
#define PMTK_ALWAYS_LOCATE_8 "$PMTK225,8*23" 
#define GPS_WAKE_PIN 0


unsigned long lastTime = 0;

unsigned int segment_counter = 0;

float predict_data[N_INPUTS];
float prediction[N_OUTPUTS];

#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization


//record_type record[8];
// 30 minutes of activities with 2 bits every 10 seconds gives 45 bytes


#include <RTCZero.h>

/* Create an rtc object */
RTCZero rtc;

#include "WDTZero.h"

WDTZero WatchDogTimer; 

Storage storage;

bool GPS_ACTIVE = false;
bool IMU_ACTIVE = false;
bool LORA_ACTIVE = false;


unsigned long gps_run_time = 1000*60*10;  // gps will run for up to 10 minutes to get a fix
unsigned long lora_run_time = 1000*60*20; // lora will broadcast for up to 20 minutes every hour

unsigned long gps_start_time = 0;  
unsigned long lora_start_time = 0; 



bool GPS_SLEEP = true;
unsigned int imu_counter = 0;
unsigned int bit_counter = 0;

unsigned long gps_timer = 0;
unsigned long gps_time_out = 1000*60*10;

void setup() 
{
    Serial.begin(115200);
    delay(4000);
    
  if (!storage.begin()) 
   {
      Serial.println("ERROR: storage");
    
    }
   

   if (!GPS.begin(I2C_ADDRESS)) 
   {
      Serial.println("ERROR: GPS");
    
    }

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  GPS.sendCommand(PMTK_ALWAYS_LOCATE_8);
  

//   if (!initialiseIMU())  {Serial.println("ERROR: ICM ");}
   Serial.println("initialised...");

//  if (!modem.begin(EU868)) {
//    Serial.println("Failed to start module");
//    while (1) {}
//  };
//  Serial.print("Your module version is: ");
//  Serial.println(modem.version());
//  Serial.print("Your device EUI is: ");
//  Serial.println(modem.deviceEUI());
  delay(1);
  rtc.begin(); // initialize RTC

  rtc.setAlarmTime(0, 0, 0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  
//  rtc.attachInterrupt(quarter_hour_alarm);

  Serial.print("\nWDTZero-Demo : Setup Soft Watchdog at 32S interval"); 
 WatchDogTimer.attachShutdown(wd_shutdown);
 WatchDogTimer.setup(WDT_SOFTCYCLE16M);  // initialize WDT-softcounter refesh cycle on 16m interval
}

void wd_shutdown()
{
  Serial.print("\nshutting down ...");
}


void wake_gps()
{
   Serial.println("waking up gps");
   gps_start_time = millis();
   digitalWrite(GPS_WAKE_PIN, HIGH);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
   GPS.sendCommand(PMTK_ALWAYS_LOCATE);
}

void pause_gps()
{
   Serial.println("pausing gps");
   digitalWrite(GPS_WAKE_PIN, LOW);    
//      GPS.sendCommand(PMTK_BACKUP_MODE);
}

bool waiting_on_first_fix = true;

void update_time()
{

  if ((GPS.satellites>4)||(waiting_on_first_fix))
  {
      rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
      rtc.setDate(GPS.day, GPS.month, GPS.year);
      waiting_on_first_fix = false;
  }
}

unsigned int imu_count;


void loop() 
{

    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE))
    {
      // if nothing active then we are at the top of the hour so wake up the gps and imu
      wake_gps();
      GPS_ACTIVE=true;
      IMU_ACTIVE=true;
      gps_start_time = millis();

    }


    if (GPS_ACTIVE)
    {
      char c = GPS.read();
      if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

      if ((GPS.fix)&&(GPS.HDOP<1.0)|| ( millis() - gps_start_time >= gps_run_time) 
      {
         // either we got a fix or we're out of time
         if (GPS.fix)
         {
             // TODO add an entry for lora broadcasting
             update_time();
         }
         GPS_ACTIVE=false;
         pause_gps();
      }

    }

    if (IMU_ACTIVE)
    {
      if ((millis() - lastTime) >= 200) //To stream at 5 Hz without using additional timers
      {
        lastTime = millis();
        IMU_ACTIVE = updateIMU(&predict_data[segment_counter*N_CHANNELS]);
      }
    }


    if ((!IMU_ACTIVE) && (!LORA_ACTIVE))
    {
      // add the readings to storage
      // turn on lora
      LORA_ACTIVE=true;
      lora_start_time = millis();
     
    }
    
    if (LORA_ACTIVE)
    {

     // try_send();
     if (millis() - lora_start_time >= lora_run_time) 
     {
      LORA_ACTIVE=false;
     }

    }
    
    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) rtc.standbyMode();

    
}
