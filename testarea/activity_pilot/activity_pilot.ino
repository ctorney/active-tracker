#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>
#include "BNO055_support.h"   
#include <Wire.h>

#include "tf_model.h"

#define N_CHANNELS 5
#define SEG_LENGTH 50

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 4*1024

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;


struct bno055_t accel;
struct bno055_euler euler_data; //Structure to hold the Euler data
struct bno055_linear_accel accel_data; //Structure to hold the accelerometer data

unsigned long lastTime = 0;

unsigned int segment_counter = 0;

float predict_data[N_INPUTS];
float prediction[N_OUTPUTS];


void get_accel_data()
{
  bno055_read_linear_accel_xyz(&accel_data);
  bno055_read_euler_hrp(&euler_data);     

  // reset the segment counter
  if (segment_counter==SEG_LENGTH) segment_counter=0;
  predict_data[segment_counter*N_CHANNELS] = float(euler_data.p) / 16.00;
  predict_data[segment_counter*N_CHANNELS + 1] = float(euler_data.r) / 16.00;
  predict_data[segment_counter*N_CHANNELS + 2] = float(accel_data.x);
  predict_data[segment_counter*N_CHANNELS + 3] = float(accel_data.y);
  predict_data[segment_counter*N_CHANNELS + 4] = float(accel_data.z);

  segment_counter++;
}

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
    
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&accel); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

}

void loop() 
{
  if ((millis() - lastTime) >= 200) //To stream at 5 Hz without using additional timers
  {
    lastTime = millis();
    get_accel_data();
  
    if (segment_counter==SEG_LENGTH)
    {
      tf.predict(predict_data, prediction);
      int activity = tf.probaToClass(prediction);
      if (activity == 0) Serial.println("Walking");
      if (activity == 1) Serial.println("Standing");
      if (activity == 2) Serial.println("Sitting");
      if (activity == 3) Serial.println("Lying");
      //Serial.println(activity);
      //Serial.print("Predicted class is: ");
      //Serial.println(tf.probaToClass(prediction));
      
    }
  }
}
