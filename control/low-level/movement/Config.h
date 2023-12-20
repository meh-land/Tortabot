#ifndef PINCONFIG_H
#define PINCONFIG_H


#define DEAD_ZONE 0.3
#define numberOfVelocities 4
#define sampleTime 0.03 //millisecond
#define TIME_FREQ 50
#define RESOLUTION 2050
#define minOut -65535
#define maxOut 65535

#define MAX_VEL 65535
#define MIN_VEL 0
#define kp_init 30000
#define ki_init 30000
#define kd_init 10



// wheel pins 

#define MOTOR_FL_EN  PA1
#define MOTOR_FL_IN1 PA2
#define MOTOR_FL_IN2 PA0

#define MOTOR_FR_EN  PA3
#define MOTOR_FR_IN1 PA5
#define MOTOR_FR_IN2 PA4

#define MOTOR_BL_EN  PA6
#define MOTOR_BL_IN1 PA7
#define MOTOR_BL_IN2 PB0

#define MOTOR_BR_EN  PB1
#define MOTOR_BR_IN1 PB11
#define MOTOR_BR_IN2 PB10



#define ENCODER_FL_A PB3   // motor0
#define ENCODER_FL_B PB4 

#define ENCODER_FR_A PA9   // motor1 
#define ENCODER_FR_B PA8

#define ENCODER_BL_A PB14   // motor2
#define ENCODER_BL_B PB15 

#define ENCODER_BR_A PB13   // motor3
#define ENCODER_BR_B PB12

#endif
