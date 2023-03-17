
#pragma once

#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#define N_CHANNELS 5
#define SEG_LENGTH 50

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

#define TENSOR_ARENA_SIZE 4*1024




int initialiseIMU();
bool updateIMU(float *pdata);


void activate();
