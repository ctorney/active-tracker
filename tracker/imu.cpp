
#include "imu.h"


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
    if (!icm.begin_I2C()) {
      Serial.println("ERROR: ICM ");
      return 0;
    }
    return 1;
  
}


void updateIMU(float *pdata)
{

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
   return;
}
