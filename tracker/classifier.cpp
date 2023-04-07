#include "classifier.h"
#include <imuFilter.h>
#include "tf_model.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>


Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;






Adafruit_ICM20649 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;



constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.
imuFilter <&GAIN> filter;

float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float angle_bias[2] = {0.0f, 0.0f};


constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.



bool Classifier::begin() {
  
  tf.begin(model_tflite);

  // check if model loaded fine
  if (!tf.isOk()) {
      Serial.print("ERROR: ");
      Serial.println(tf.getErrorMessage());
      return false;
    }

  if (!icm.begin_I2C()) 
  {
    Serial.println("ERROR: ICM ");
    return false;
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

  return true;

}


void Classifier::activate(long unixtime){

  latest_activity.start_time = unixtime;

  memset(latest_activity.activities,0,sizeof(latest_activity.activities));

  if (!icm.begin_I2C()) 
  {
    Serial.println("ERROR: ICM ");
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
    
  filter.setup( ax,ay,az);   
  imu_counter=0;
  bit_counter=0;
  segment_counter=0;
}


void Classifier::deactivate(){

  icm.reset();
  return;
}

bool Classifier::update(){

  update_imu(&predict_data[segment_counter*N_CHANNELS]);
  segment_counter++;
  if (segment_counter==SEG_LENGTH)
  {

    tf.predict(predict_data, prediction);
    int activity = tf.probaToClass(prediction);

    // FOR DEBUGGING ONLY!!!!!!!!!!
    // WE'LL SET ACTIVITY TO BE EQUAL TO THE BIT_COUNTER FOR THE FIRST HALF
    // THEN EQUAL TO 3 - BIT_COUNTER FOR THE SECOND HALF
    activity = 3 - bit_counter;
    if (imu_counter<23)
    {
      activity = bit_counter;
    }
    // END OF DEBUGGING CODE 
    // DELETE BEFORE DEPLOYMENT TO USE THE REAL CNN PREDICTIONS

    segment_counter=0;

    // set so that the first entry goes into the most significant bit and we read
    // right to left in order of the temporal sequence
    int msb = 3 - bit_counter;

    switch (activity){
        case 0:
          break;
        case 1:
          bitWrite(latest_activity.activities[imu_counter], 2*msb, 1);
          break;
        case 2:
          bitWrite(latest_activity.activities[imu_counter], 2*msb+1, 1);
          break;
        case 3:
          bitWrite(latest_activity.activities[imu_counter], 2*msb, 1);
          bitWrite(latest_activity.activities[imu_counter], 2*msb+1, 1);
          break;
      }

    bit_counter++;

    if (bit_counter == BIT_LENGTH)
    {
     bit_counter = 0;
     imu_counter++;
    }

    if (imu_counter == SERIES_LENGTH) 
      return false;
    
  }
  return true;
}

void Classifier::update_imu(float *pdata){
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

  pdata[0] = v[0]-acc_bias[0];
  pdata[1] = v[1]-acc_bias[1];
  pdata[2] = v[2]-acc_bias[2];
  pdata[3] = pitch-angle_bias[0];
  pdata[4] = roll-angle_bias[1];

  
}
