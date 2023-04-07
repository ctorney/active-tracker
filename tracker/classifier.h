
#pragma once
#ifndef _CLASSIFIER_H
#define _CLASSIFIER_H
#include <Arduino.h>
#include "tracker.h"


#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#define N_CHANNELS 5
#define SEG_LENGTH 50


#define BIT_LENGTH 4        // we write 4 predictions to a single byte as we need 2 bits per prediction
#define SERIES_LENGTH 45    // each byte is 40 seconds (4 predictions x 10s per prediction) 30 minutes is 45 bytes

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 4*1024

class Classifier {
public:
  
  bool begin();
  bool update();

  
  
  void activate(long unixtime);
  void deactivate();

  activity_reading latest_activity;

private:
    void update_imu(float *pdata);
    unsigned int imu_counter = 0;
    unsigned int bit_counter = 0;
    unsigned int segment_counter = 0;
    float predict_data[N_INPUTS];
    float prediction[N_OUTPUTS];
};


#endif
