#include "Encoder.h"

#define MINUTE_TO_SECOND 60
#define MS_TO_S          1000
#define RPS_TO_RAD (2.0f*PI)

 
Encoder::Encoder(float resolution){
  counts=0;
  prevcount=0;
  prevtime=0;
  this->resolution=resolution;
}


float Encoder::calcspeed(){
  
  float current_time=millis();
  long  newcount=counts;
  int   dp=newcount-prevcount;
  float dt=current_time- prevtime;

        /*calc speed*/
  wheel_speed=(MS_TO_S/resolution)*(dp/dt)*RPS_TO_RAD;//in Rad,{speed=dp/dt in counts/ms}

    //update postition
  prevcount=counts;

    //update time
  prevtime=current_time;

  return wheel_speed;
 
}
