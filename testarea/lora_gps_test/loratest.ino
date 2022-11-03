


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#include <RTCZero.h>


#define GPS_I2C_ADDRESS 0x10

#include <MKRWAN.h>

LoRaModem modem;


Adafruit_GPS GPS(&Wire);

unsigned long previousConnectionTime = 0;  
const unsigned long connectionInterval = 1000*60*10; // 10 minute intervals try and send

void setup() 
{
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  delay(2000);
  Serial.println("starting setup");

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
  Serial.println("GPS setup");


  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  if (modem.version() != ARDUINO_FW_VERSION) {
    Serial.println("Please make sure that the latest modem firmware is installed.");
    Serial.println("To update the firmware upload the 'MKRWANFWUpdate_standalone.ino' sketch.");
  }
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  
}

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
  }
  
  delay(500);
  
}


void send_lora()
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

  if (first_fix && gps_on)
    {
      GPS.standby();
      gps_on=false;
    }
  
}

void activate()
{
  
  
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
