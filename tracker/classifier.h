
#pragma once

#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#define N_CHANNELS 5
#define SEG_LENGTH 50

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 4*1024

#include "tf_model.h"


Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;


int initialiseIMU();
bool updateIMU(float *pdata);


void activate();
