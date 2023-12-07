#ifndef PINCONFIG_H
#define PINCONFIG_H


#define DEAD_ZONE 2
#define numberOfVelocities 4
#define sampleTime 0.03 //millisecond
#define TIME_FREQ 50
#define RESOLUTION 2050
#define minOut -65535
#define maxOut 65535
#define Kp0 1000
#define Ki0 2500
#define Kd0 1

#define Kp1 800
#define Ki1 2500  
#define Kd1 0.7



// wheel pins 
#define MOTOR0_EN  PA3
#define MOTOR0_IN1 PA4
#define MOTOR0_IN2 PA5

#define MOTOR1_EN  PA1
#define MOTOR1_IN1 PA0
#define MOTOR1_IN2 PA2

#define MOTOR2_EN  PA6
#define MOTOR2_IN1 PA7
#define MOTOR2_IN2 PB0

#define MOTOR3_EN  PB11
#define MOTOR3_IN1 PB10
#define MOTOR3_IN2 PB1


#define ENCODER0_A PB4   // motor0 
#define ENCODER0_B PB3

#define ENCODER1_A PA8   // motor1
#define ENCODER1_B PA9 


#define ENCODER2_A PB15   // motor2
#define ENCODER2_B PB14

#define ENCODER3_A PB13   // motor3
#define ENCODER3_B PB14 

#endif
