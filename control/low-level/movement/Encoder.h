#ifndef ENCODER_H   
#define ENCODER_H
#include"Arduino.h"

/**
 * @class Encoder
 * @brief This class is parent for encoder interfacing
 * 
*/
class Encoder{
 
  public:
    long encodercount; ///<encoder counts

  /**
   * @param resolution the resolution of the encoder
   * @brief the constructor of the parent class
  */
    Encoder(float resolution);
  
  /**
   * @brief abstracted setup method for the parent class
  */
    void setup();
  
  /**
   * @brief method for speed calculation in RPM
   * @return the speed of the motor
  */
    float calcspeed();

  private :
  
     int prevcount = 0;     ///<previous counts of the encoder
     float prevtime;        ///<previous time of the calculation
     float Espeed = 0 ;     ///<speed of the motor
     float  resolution = 0; ///<the resolution of the encoder
};
#endif
