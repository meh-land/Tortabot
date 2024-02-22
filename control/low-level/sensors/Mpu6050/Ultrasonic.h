#ifndef ULTRASONIC_H   
#define ULTRASONIC_H

#include"Arduino.h"

#define echo_front PA1
#define trig_front PA0
#define echo_back PA3
#define trig_back PA2
#define echo_left PA5
#define trig_left PA4
#define echo_right PA7
#define trig_right PA6
#define Sspeed 0.034

float calc_distance_front();
float calc_distance_back();
float calc_distance_left();
float calc_distance_right();
void ultrasonic_init();


#endif
