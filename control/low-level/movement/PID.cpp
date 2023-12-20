#include "PID.h"
#include "Config.h"
#include <Arduino.h>


PIDControl::PIDControl(float Kp,float Ki,float Kd,float sample_time) 
{
      this->Kp = Kp; 
      this->Ki = Ki; 
      this->Kd = Kd;
      this->sample_time = sample_time; 
      
      previous_error = 0; 
      integralTerm = 0; 
}

void PIDControl :: set_setpoint(float setpoint)
{
    this->setpoint = setpoint;
    //integralTerm = 0;
}

float  PIDControl::calculateOutput(float feedBack) 
{
    //if((abs(setpoint-feedBack)>DEAD_ZONE))
   // {

        error = setpoint - feedBack; 

        integralTerm += Ki * (error * sample_time); 
        integralTerm = constrain(integralTerm,minIntegral,maxIntegral);
        derivativeTerm = Kd * (error - previous_error) / sample_time; 
        output = Kp * error + integralTerm +  derivativeTerm; 
        previous_error = error; 
    //}
    /*if ((abs(setpoint-feedBack)<DEAD_ZONE)&& (0 == setpoint))
    {
      output = 0;
      integralTerm = 0;
    }*/
    return output;
      
}
      
    void PIDControl::set_parameters(float Kp, float Ki, float Kd){

      this->Kp = Kp;
      this->Ki = Ki; 
      this->Kd = Kd; 
    }
