#include "Encoder.h"
Encoder::Encoder(float resolution){
  counts=0;
  prevcount=0;
  prevtime=0;
  this->resolution=resolution;
}


float Encoder::calcspeed(){
  
  float current_time=millis();
  long newcount=counts;
  int dp=newcount-prevcount;
  float dt=current_time- prevtime;

        /*calc speed*/
  Espeed=(60*1000/resolution)*(dp/dt);//in RPM,{speed=dp/dt in counts/ms}

    //update postition
  prevcount=counts;

    //update time
  prevtime=current_time;

  return Espeed;
 
}
