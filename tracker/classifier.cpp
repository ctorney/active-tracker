
#include "classifier.h"

#include <imuFilter.h>

#include "tf_model.h"


Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines response of heading correction with respect to gravity.
imuFilter <&GAIN> fusion;

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

/*
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#include "tf_model.h"

#define N_CHANNELS 5
#define SEG_LENGTH 50

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 5432

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
  
unsigned int segment_counter = 0;

float predict_data[N_INPUTS];
float prediction[N_OUTPUTS];


Adafruit_ICM20649 icm;
*/
int initialiseIMU()
{
/*
    tf.begin(model_tflite);

    // check model
    if (!tf.isOk()) {
      Serial.print("ERROR: ");
      Serial.println(tf.getErrorMessage());
      return 0;
    }
*/

tf.begin(model_tflite);

    // check if model loaded fine
    if (!tf.isOk()) {
      Serial.print("ERROR: ");
      Serial.println(tf.getErrorMessage());
      while (true) delay(1000);
    }
    if (!icm.begin_I2C()) {
      Serial.println("ERROR: ICM ");
      return 0;
    }
    return 1;
  
}

void activate()
{

      int  imu_count = 0;
      int bit_count = 0;
//      memset(activities,0,sizeof(activities));

}

bool updateIMU(float *pdata)
{
  /*
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
          
          switch (activity){
              case 0:
                break;
              case 1:
                bitWrite(activities[imu_counter], 2*bit_counter, 1);
                break;
              case 2:
                bitWrite(activities[imu_counter], 2*bit_counter+1, 1);
                break;
              case 3:
                bitWrite(activities[imu_counter], 2*bit_counter, 1);
                bitWrite(activities[imu_counter], 2*bit_counter+1, 1);
                break;
            }

          bit_counter++;

          if (bit_counter == 4)
          {
           bit_counter = 0;
           imu_counter++;
          }

          if (imu_counter == 45) 
            IMU_ACTIVE = false
        }
  icm.getEvent(&accel, &gyro, &temp);
  

  // reset the segment counter
  pdata[0] = float(gyro.gyro.x) / 16.00;
  pdata[1] = float(gyro.gyro.y) / 16.00;
  pdata[2] = float(accel.acceleration.x);
  pdata[3] = float(accel.acceleration.y);
  pdata[4] = float(accel.acceleration.z);
/*  
  segment_counter++;
  if (segment_counter==SEG_LENGTH)
  {
    segment_counter=0;
    
    tf.predict(predict_data, prediction);
    int activity = tf.probaToClass(prediction);
    Serial.println(activity);
  /*    if (activity == 0) Serial.println("Walking");
      if (activity == 1) Serial.println("Standing");
      if (activity == 2) Serial.println("Sitting");
      if (activity == 3) Serial.println("Lying"); * /
     segment_counter=0;
     return activity;
   }
   */
   return true;
}
