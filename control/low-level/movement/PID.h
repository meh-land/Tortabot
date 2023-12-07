#ifndef PID_H
#define PID_H

#define minIntegral -50000
#define maxIntegral 50000

class PIDControl
{
    public:
        PIDControl(float Kp,float Ki,float Kd, float sample_time);
        float calculateOutput(float feedBack);
        void set_parameters(float Kp, float Ki, float Kd);
        void set_setpoint(float setpoint);
    private:
        float Kp = 0; 
        float Ki = 0; 
        float Kd = 0; 
        float sample_time; 
        float error;
        float previous_error; 
        float integralTerm; 
        float derivativeTerm;
        float output;
        float setpoint;
};
#endif