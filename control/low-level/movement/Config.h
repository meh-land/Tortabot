#ifndef PINCONFIG_H
#define PINCONFIG_H


#define DEAD_ZONE 20
#define numberOfVelocities 4
#define sampleTime 0.03 //millisecond
#define TIME_FREQ 50
#define RESOLUTION 2050
#define minOut -65535
#define maxOut 65535

#define MAX_VEL 65535
#define MIN_VEL 0
#define kp_init 1000
#define ki_init 10
#define kd_init 0



// wheel pins 

#define MOTOR_FL_EN  PB1
#define MOTOR_FL_IN1 PB10
#define MOTOR_FL_IN2 PB11

#define MOTOR_FR_EN  PA6
#define MOTOR_FR_IN1 PB0
#define MOTOR_FR_IN2 PA7

#define MOTOR_BL_EN  PA3
#define MOTOR_BL_IN1 PA4
#define MOTOR_BL_IN2 PA5

#define MOTOR_BR_EN  PA1
#define MOTOR_BR_IN1 PA0
#define MOTOR_BR_IN2 PA2



#define ENCODER_FL_A PB12   // motor0
#define ENCODER_FL_B PB13 

#define ENCODER_FR_A PB15   // motor1 
#define ENCODER_FR_B PB14

#define ENCODER_BL_A PA8   // motor2
#define ENCODER_BL_B PA9 

#define ENCODER_BR_A PB4   // motor3
#define ENCODER_BR_B PB3

#endif
