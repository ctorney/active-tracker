

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





#include "arduino_secrets.h" 

char ssid[] = SECRET_SSID;  
char pass[] = SECRET_PASS;  


int status = WL_IDLE_STATUS;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
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
      while (true);
    }

  if (!GPS.begin(GPS_I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");
        while (true);
  }

  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
  


  // wait a second for connection:


  delay(1000);




  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
}
unsigned long previousIMUTime = 0;  
const unsigned long IMUEventInterval = 200; // 200 milliseconds


//WiFiClient client;
void loop() {

  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
  // check for a new client:
  WiFiClient client = server.available();
   Serial.println(client.status());
  // when the client sends the first byte, say hello:
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

      unsigned long currentTime = millis();  
      if (currentTime - previousIMUTime >= IMUEventInterval) 
      {
          previousIMUTime = millis();
          output_imu();
      }
    }
    else
    {
      Serial.println("We have lost the client");
      client.flush();
      client.stop();
      alreadyConnected = false;
    }
  }
  else{
    client.stop();
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
    server.print(gx);
    server.print(",");
    server.print(gy);
    server.print(",");
    server.print(gz);
    server.print(",");
    server.print(ax);
    server.print(",");
    server.print(ay);
    server.print(",");
    server.println(az);


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
