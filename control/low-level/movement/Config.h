#ifndef PINCONFIG_H
#define PINCONFIG_H


#define DEAD_ZONE 2
#define numberOfVelocities 4
#define sampleTime 0.03 //millisecond
#define TIME_FREQ 30
#define RESOLUTION 560
#define minOut -65535
#define maxOut 65535

#define Kp0 800
#define Ki0 0
#define Kd0 0

#define Kp1 800
#define Ki1 0  
#define Kd1 0

#define Kp2 800
#define Ki2 0 
#define Kd2 0

#define Kp3 800
#define Ki3 0
#define Kd3 0


// wheel pins 
#define MOTOR0_EN  PB1
#define MOTOR0_IN1 PB11
#define MOTOR0_IN2 PB10

#define MOTOR1_EN PA6
#define MOTOR1_IN1 PA7
#define MOTOR1_IN2 PB0

#define MOTOR2_EN PA3
#define MOTOR2_IN1 PA5
#define MOTOR2_IN2 PA4

#define MOTOR3_EN PA1
#define MOTOR3_IN1 PA2
#define MOTOR3_IN2 PA0
 
//Encoder Pins
#define EnPin0 PB13  // motor0 
#define EnPin1 PB12  

#define EnPin2 PB14   // motor1
#define EnPin3 PB15  

#define EnPin4 PA8   // motor2
#define EnPin5 PA9  

#define EnPin6 PB4   // motor3  
#define EnPin7 PB3

#endif
