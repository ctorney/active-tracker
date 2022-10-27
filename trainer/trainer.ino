

#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include "ArduinoLowPower.h"
#include "WDTZero.h"

#define GPS_I2C_ADDRESS 0x10

#define PMTK_ALWAYS_LOCATE "$PMTK225,9*22" ///< 115200 bps
#define PMTK_FULL_POWER "$PMTK225,0*2B" ///< 115200 bps
#define PMTK_ALWAYS_LOCATE_8 "$PMTK225,8*23" ///< 115200 bps


#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

#include <RTCZero.h>

#include <imuFilter.h>

constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.
imuFilter <&GAIN> filter;

float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float angle_bias[2] = {0.0f, 0.0f};

float imu_data[5];

constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.

uint8_t COLLAR_ID = 1;

#define SECRET_SSID "WILDEBEEST_"
#define SECRET_PASS "WILDEBEEST_"

char ssid[13];
char pass[13];


RTCZero rtc;
WDTZero wdt; 

int status = WL_IDLE_STATUS;

WiFiServer server(23);

bool alreadyConnected = false; // whether or not a client has connected recently
bool active_mode = false;
bool first_fix = false;

void setup() 
{
  String strSSID = SECRET_SSID + String(COLLAR_ID,DEC);
  String strPASS = SECRET_PASS + String(COLLAR_ID,DEC);

  strSSID.toCharArray(ssid, 13);
  strPASS.toCharArray(pass, 13);
  
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
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);



  Serial.println("GPS setup");


  


   rtc.begin();
   rtc.setAlarmTime(00,00,10);
   rtc.enableAlarm(rtc.MATCH_SS);


  



  deactivate();
    
}

unsigned long previousIMUTime = 0;  
unsigned long previousConnectionTime = 0;  
const unsigned long connectionWait = 1000*30;// 30 seconds for DEBUGGING!!! 1000*60*10; // switch off after 10 minutes of no connection

const unsigned long IMUEventInterval = 200; // 200 milliseconds so we record the IMU data at 5hz
unsigned short imu_counter = 0; // keep track of the sequence of readings in a 10s batch
const unsigned short imu_length = 50; // 10s batch at 5hz gives use 50 readings in a batch

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
  
  //Serial.println(WiFi.status());

  bool send_imu = false;
  
  if (currentTime - previousIMUTime >= IMUEventInterval) 
  {
    previousIMUTime = millis();
    update_imu();
    send_imu = true;
  }
  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
  if (!first_fix)
    if (GPS.fix)
    {
      first_fix=true;
      update_time();
    }
  
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
      Serial.println("We have lost the client");
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

bool check_time()
{
  uint8_t hour = rtc.getHours();
  uint8_t day = rtc.getDay();
  if ((rtc.getMinutes()) % 2 != 0) return true;

  // each collar is active once every 3 days
  if ((day + COLLAR_ID) % 3 != 0) return false;

  if (hour==8) return true;
  if (hour==9) return true;
  if (hour==10) return true;
  if (hour==12) return true;
  if (hour==14) return true;
  if (hour==16) return true;
  
  return false;
}

void update_time()
{
  // adjust for UTC
  rtc.setTime(GPS.hour+3, GPS.minute, GPS.seconds);
  rtc.setDate(GPS.day, GPS.month, GPS.year);
}


void deactivate()
{
  WiFi.end();
  GPS.standby();
  active_mode=false;
  wdt.setup(WDT_OFF);  //watchdog off
}

void activate()
{
  active_mode=true;

  // start the access point
  status = WiFi.beginAP(ssid, pass);

  // start the server
  server.begin();
  
  printWifiStatus();

  // watchdog on
  wdt.setup(WDT_SOFTCYCLE8S);  // initialize WDT-softcounter refesh cycle on 8sec interval
  icm.getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
    
  filter.setup( ax,ay,az);   
  imu_counter=0;

  first_fix = false;
  previousConnectionTime = millis();
  previousIMUTime = millis();

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

  if (GPS.hour < 10) { server.print('0'); }
  server.print(GPS.hour, DEC); server.print(':');
  if (GPS.minute < 10) { server.print('0'); }
  server.print(GPS.minute, DEC); server.print(':');
  if (GPS.seconds < 10) { server.print('0'); }
  server.print(GPS.seconds, DEC); server.print(',');
  server.print(GPS.latitude); server.print(",");
  server.print(GPS.longitude); server.print(",");
  server.print(imu_counter); server.print(","); server.print(imu_data[0]); server.print(","); server.print(imu_data[1]);
  server.print(","); server.print(imu_data[2]); server.print(","); server.print(imu_data[3]); server.print(",");
  server.println(imu_data[4]);

}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
