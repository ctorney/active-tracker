#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>
#include <Wire.h>

#include "tf_model.h"

#define N_CHANNELS 5
#define SEG_LENGTH 50

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 4*1024

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;



#include "imu.h"

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

#include <MKRWAN_v2.h>

LoRaModem modem;

#define SECRET_APP_EUI "xxxxxxxxxxxxx"
#define SECRET_APP_KEY "yyyyyyyyyyyyyyyyyyyyyyy"

#define I2C_ADDRESS 0x10

uint32_t timer = millis();



unsigned long lastTime = 0;

unsigned int segment_counter = 0;

float predict_data[N_INPUTS];
float prediction[N_OUTPUTS];


void setup() 
{
    Serial.begin(115200);
    delay(4000);
    tf.begin(model_tflite);

    // check if model loaded fine
    if (!tf.isOk()) {
      Serial.print("ERROR: ");
      Serial.println(tf.getErrorMessage());
      while (true) delay(1000);
    }

   

   if (!GPS.begin(I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");
    
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
  

   if (!initialiseIMU())  {Serial.println("ERROR: ICM ");}
   Serial.println("initialised...");

if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  delay(1);

}

void loop() 
{
  
if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
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

  if ((millis() - lastTime) >= 200) //To stream at 5 Hz without using additional timers
  {
    lastTime = millis();
     //if (segment_counter==SEG_LENGTH) segment_counter=0;

  
    updateIMU(&predict_data[segment_counter*N_CHANNELS]);
    segment_counter++;
    if (segment_counter==SEG_LENGTH)
    {
      tf.predict(predict_data, prediction);
      int activity = tf.probaToClass(prediction);
      if (activity == 0) Serial.println("Walking");
      if (activity == 1) Serial.println("Standing");
      if (activity == 2) Serial.println("Sitting");
      if (activity == 3) Serial.println("Lying");
      segment_counter=0;
    }
  }
}
