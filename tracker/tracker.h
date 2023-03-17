#pragma once



typedef struct
{
      long start_time;
      byte activities[45];
}  activity_reading;

typedef struct
{
      long start_time;
      float lat;
      float lon;
}  location_reading;
