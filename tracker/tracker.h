#pragma once



typedef struct
{
      uint32_t start_time;
      byte activities[45];
}  activity_reading;

typedef struct
{
      uint32_t start_time;
      float lat;
      float lon;
}  location_reading;
