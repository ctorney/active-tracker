

#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>

#define GPS_I2C_ADDRESS 0x10

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;


#include <imuFilter.h>

constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.
imuFilter <&GAIN> filter;

float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float angle_bias[2] = {0.0f, 0.0f};

float imu_data[5];

constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.



#include "arduino_secrets.h" 

char ssid[] = SECRET_SSID;  
char pass[] = SECRET_PASS;  


int status = WL_IDLE_STATUS;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  // while (true);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

    delay(5000);
      Serial.println("starting setup");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
    
    ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }


  WiFi.config(IPAddress(192, 168, 4, 1));
  status = WiFi.beginAP(ssid, pass);


  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");


    // don't continue
    while (true);


  }
if (!icm.begin_I2C()) {
      Serial.println("ERROR: ICM ");
        while (true)
        {
digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
        }
    }
      Serial.println("I2C setup");
  
    

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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);


  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
  


  icm.setAccelRateDivisor(225);
  icm.setGyroRateDivisor(225);


  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);
  delay(100);

  icm.getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
//
  acc_bias[0] = ax;
  acc_bias[1] = ay;
  acc_bias[2] = az;
  filter.setup( ax,ay,az);     



  // wait a second for connection:
Serial.println("wifi setup");

  delay(1000);




  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();

  //WiFi.lowPowerMode();
  //GPS.standby();

}
unsigned long previousIMUTime = 0;  
const unsigned long IMUEventInterval = 200; // 200 milliseconds

bool GPS_ON = true;

//WiFiClient client;
void loop() {

  //Serial.println(WiFi.status());

  bool send_imu = false;
  unsigned long currentTime = millis();  
  if (currentTime - previousIMUTime >= IMUEventInterval) 
  {
    previousIMUTime = millis();
    update_imu();
    send_imu = true;
  }
  if (GPS_ON)
  {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
  }
  
  // check for a new client:
  WiFiClient client = server.available();
  
   
  if (client) 
  {
    client.flush();

    if (!alreadyConnected) 
    {
    
      Serial.println("We have a new client");
      client.print("Connected to server: ");
      client.println(WiFi.SSID());
      alreadyConnected = true;
    }

    if (client.status() > 0) 
    {
      if (send_imu) 
          output_imu();
    }
    else
    {
      Serial.println("We have lost the client");
      client.flush();
      client.stop();
      alreadyConnected = false;
    }
  }
  
  int status = WiFi.status();

  if (status == WL_AP_CONNECTED) 
  {
  // a device has connected to the AP
  // Serial.println("Device connected to AP");
  } 
  else 
  {
  
    if (alreadyConnected) 
    {
      Serial.println("We have lost the client");
      client.flush();
      client.stop();
      alreadyConnected = false;
    }
  }
    // client.stop();
  
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
  imu_data[4] = pitch-angle_bias[0];
  imu_data[5] = roll-angle_bias[0];

}
void output_imu()
{

    

    
    if (GPS.hour < 10) { server.print('0'); }
    server.print(GPS.hour, DEC); server.print(':');
    if (GPS.minute < 10) { server.print('0'); }
    server.print(GPS.minute, DEC); server.print(':');
    if (GPS.seconds < 10) { server.print('0'); }
    server.print(GPS.seconds, DEC); server.print('.');
    if (GPS.milliseconds < 10) {
      server.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      server.print("0");
    }
    server.print(GPS.milliseconds);
    server.print(",");
    server.print(GPS.latitude);
    server.print(",");
    server.print(GPS.longitude);
    server.print(",");
    server.print(imu_data[0]);
    server.print(",");
    server.print(imu_data[1]);
    server.print(",");
    server.print(imu_data[2]);
    server.print(",");
    server.print(imu_data[3]);
    server.print(",");
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
