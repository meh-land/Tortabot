
#include <Servo.h>
#define SERVO1 PA3 // A descriptive name for D6 pin of Arduino to provide PWM signal

#define DAMPING 10

Servo MG995_Servo;  // Define an instance of of Servo with the name of "MG995_Servo"
  
int servo_angle = 90;


void damp_write(int setpoint_angle)
{
  if (setpoint_angle > servo_angle)
  {
    while(setpoint_angle > servo_angle)
    {
      servo_angle++;
      MG995_Servo.write(servo_angle);
      delay(DAMPING);
    }
  }

  else if (setpoint_angle < servo_angle)
  {
    while(setpoint_angle < servo_angle)
    {
      servo_angle--;
      MG995_Servo.write(servo_angle);
      delay(DAMPING);
    }
  }
}

void setup() {
  MG995_Servo.attach(SERVO1);  // Connect D6 of Arduino with PWM signal pin of servo motor

}

void loop() {
  servo_angle=90;
  MG995_Servo.write(90); //Turn clockwise at high speed
  delay(3000);
  damp_write(150);
  
  delay(5000);

}
