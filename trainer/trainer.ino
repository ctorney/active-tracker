

uint8_t COLLAR_ID = 1;

#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include "WDTZero.h"
#include <Adafruit_GPS.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

#include <RTCZero.h>
#include <imuFilter.h>


#define GPS_I2C_ADDRESS 0x10
#define PMTK_PERIODIC "$PMTK225,1,4000,120000,4000,120000*16"
#define SECRET_SSID "WCOLLAR"
#define SECRET_PASS "wcollar"

Adafruit_GPS GPS(&Wire);

Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

RTCZero rtc;
WDTZero wdt; 

WiFiServer server(23);

constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.
imuFilter <&GAIN> filter;

float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float angle_bias[2] = {0.0f, 0.0f};

float imu_data[5];

constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.


char ssid[9];
char pass[9];

int status = WL_IDLE_STATUS;


bool alreadyConnected = false; // whether or not a client has connected recently
bool active_mode = false;
bool first_fix = false;
bool gps_on=true; //switch off the gps once we have the time


unsigned long previousIMUTime = 0;  
unsigned long previousConnectionTime = 0;  
const unsigned long connectionWait = 1000*60*10; // switch off after 10 minutes of no connection

const unsigned long IMUEventInterval = 200; // 200 milliseconds so we record the IMU data at 5hz
unsigned short imu_counter = 0; // keep track of the sequence of readings in a 10s batch
const unsigned short imu_length = 50; // 10s batch at 5hz gives use 50 readings in a batch

void setup() 
{
  String strSSID = SECRET_SSID + String(COLLAR_ID,DEC);
  String strPASS = SECRET_PASS + String(COLLAR_ID,DEC);

  strSSID.toCharArray(ssid, 9);
  strPASS.toCharArray(pass, 9);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  delay(2000);
  Serial.println("starting setup");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) 
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
    }
  }

  
  WiFi.config(IPAddress(192, 168, 4, 1));
  status = WiFi.beginAP(ssid, pass);


  if (status != WL_AP_LISTENING) 
  {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
    }
  }

  WiFi.end();

  // wait a second for connection:
  Serial.println("wifi setup");
  delay(500);
  
  if (!icm.begin_I2C()) 
  {
    Serial.println("ERROR: ICM ");
    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
    }
  }
  
  icm.setAccelRateDivisor(225);
  icm.setGyroRateDivisor(225);


  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);
  delay(500);

  icm.getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
    
  acc_bias[2] = pow( ax*ax + ay*ay + az*az,0.5) ;
  filter.setup( ax,ay,az);     

  Serial.println("IMU setup");
  
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
    }
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
//  GPS.sendCommand(PMTK_PERIODIC);

  Serial.println("GPS setup");


  rtc.begin();
  rtc.setAlarmTime(00,00,00);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  const byte seconds = 00;
  const byte minutes = 45;
  const byte hours = 4;

  const byte day = 30 - COLLAR_ID;

  rtc.setDay(day);
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

//  deactivate();
  for (int i = 0; i < 10; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);   // 10 flashes indicate success
    delay(200);                       
    digitalWrite(LED_BUILTIN, LOW);    
    delay(200);
  }

    Serial.println("RTC setup");

  rtc.standbyMode();
  
}

void loop() 
{
  
  if (active_mode==false)
    if (check_time())  // between operating hours and right day 7am to 7pm 
      activate();
    else
    {
      rtc.standbyMode();
      return;
    }

  wdt.clear();
  
  unsigned long currentTime = millis();  

  if (currentTime - previousConnectionTime >= connectionWait) 
  {
    deactivate();
    rtc.standbyMode();
    return;
  }
  
  bool send_imu = false;
  
  if (currentTime - previousIMUTime >= IMUEventInterval) 
  {
    previousIMUTime = millis();
    update_imu();
    send_imu = true;
  }

  if (gps_on) process_gps();
  
  // check for a new client:
  WiFiClient client = server.available();
  
   
  if (client) 
  {
    client.flush();

    if (!alreadyConnected) 
      alreadyConnected = true;

    if (client.status() > 0) 
    {
      if (send_imu) 
        output_imu();
      previousConnectionTime = millis();
    }
    else   
    {
      client.flush();
      client.stop();
      alreadyConnected = false;
    }
  }
  

  if ( WiFi.status() != WL_AP_CONNECTED) 
  {
    if (alreadyConnected) 
    {
      client.flush();
      client.stop();
      alreadyConnected = false;
    }
  }
  
}


void process_gps()
{
  if (GPS.available()) {
    char c = GPS.read();
//    Serial.write(c);
  }
//  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
//  if (!first_fix) 
    if (GPS.fix)
    {
      first_fix=true;
      // set to UTC
      rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
      rtc.setDate(GPS.day, GPS.month, GPS.year);
      //GPS.standby();
      //gps_on=false;
    }
  
      
   
}

bool check_time()
{
  uint8_t hour = rtc.getHours();
  uint8_t day = rtc.getDay();

  Serial.println("checking time");
  // each collar is active once every 3 days
  if ((day + COLLAR_ID) % 3 != 0) return false;

  // adjust for UTC - TZ is 3 hours ahead
  if (hour==7-3) return true;
  if (hour==8-3) return true;
  if (hour==9-3) return true;
  if (hour==10-3) return true;
  if (hour==12-3) return true;
  if (hour==14-3) return true;
  if (hour==16-3) return true;
  
  return false;
}


void deactivate()
{

  
  WiFi.end();
  active_mode=false;
  wdt.setup(WDT_OFF);  //watchdog off

  if (first_fix) 
    {
      GPS.standby();
      gps_on=false;
    }
  
}

void activate()
{
  digitalWrite(LED_BUILTIN, HIGH);   // 10 flashes indicate success
  
  active_mode=true;

  // start the access point
  status = WiFi.beginAP(ssid, pass);

  // start the server
  server.begin();
  
  icm.getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
    
  filter.setup( ax,ay,az);   
  imu_counter=0;
  
  previousConnectionTime = millis();
  previousIMUTime = millis();

  // watchdog on
  wdt.setup(WDT_SOFTCYCLE8S);  // initialize WDT-softcounter refesh cycle on 8sec interval

}


void update_imu()
{
  icm.getEvent(&accel, &gyro, &temp);
  float gx = float(gyro.gyro.x);
  float gy = float(gyro.gyro.y); 
  float gz = float(gyro.gyro.z); 
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
  filter.update(gx, gy, gz, ax, ay, az);

  float pitch = float(filter.pitch());
  float roll = float(filter.roll());
  float v[3] = { ax, ay, az };

  filter.projectVector( true, v );

  acc_bias[0] = (1.0-bias_gain)*acc_bias[0] + bias_gain*v[0];
  acc_bias[1] = (1.0-bias_gain)*acc_bias[1] + bias_gain*v[1];
  acc_bias[2] = (1.0-bias_gain)*acc_bias[2] + bias_gain*v[2];

  angle_bias[0] = (1.0-bias_gain)*angle_bias[0] + bias_gain*pitch;
  angle_bias[1] = (1.0-bias_gain)*angle_bias[1] + bias_gain*roll;

  imu_data[0] = v[0]-acc_bias[0];
  imu_data[1] = v[1]-acc_bias[1];
  imu_data[2] = v[2]-acc_bias[2];
  imu_data[3] = pitch-angle_bias[0];
  imu_data[4] = roll-angle_bias[1];

  imu_counter++;
  imu_counter = imu_counter % imu_length;

}

void output_imu()
{
  
  uint8_t hour = rtc.getHours();
  uint8_t minute = rtc.getMinutes();
  uint8_t second = rtc.getSeconds();
    

  if (hour < 10) { server.print('0'); }
  server.print(hour, DEC); server.print(':');
  if (minute < 10) { server.print('0'); }
  server.print(minute, DEC); server.print(':');
  if (second < 10) { server.print('0'); }
  server.print(second, DEC); server.print(',');
  server.print(imu_counter); server.print(","); server.print(imu_data[0]); server.print(","); server.print(imu_data[1]);
  server.print(","); server.print(imu_data[2]); server.print(","); server.print(imu_data[3]); server.print(",");
  server.print(imu_data[4]);server.print(",");server.println(imu_data[0]+imu_data[1]+imu_data[2]+imu_data[3]+imu_data[4]);

}
