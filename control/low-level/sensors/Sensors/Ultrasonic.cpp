#include "Ultrasonic.h"

float t=0;

void ultrasonic_init()
{
  pinMode(echo_front,INPUT);
  pinMode(trig_front,OUTPUT);
  pinMode(echo_back,INPUT);
  pinMode(trig_back,OUTPUT);
  pinMode(echo_left,INPUT);
  pinMode(trig_left,OUTPUT);
  pinMode(echo_right,INPUT);
  pinMode(trig_right,OUTPUT);
}
float calc_distance_front()
{
  t=0;
  delayMicroseconds(2000);
  digitalWrite(trig_front,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_front,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front,LOW);
  t=pulseIn(echo_front,HIGH,timeout_ms);
  float distance_front = t*Sspeed/2;
  return distance_front;  
}

float calc_distance_back()
{
  t=0;
  delayMicroseconds(2000);
  digitalWrite(trig_back,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_back,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_back,LOW);
  t=pulseIn(echo_back,HIGH,timeout_ms);
  float distance_back = t*Sspeed/2.0f;
  return distance_back;
}
float calc_distance_left()
{   
  t=0;
  delayMicroseconds(2000);
  digitalWrite(trig_left,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_left,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_left,LOW);
  t=pulseIn(echo_left,HIGH,timeout_ms);
  float distance_left = t*Sspeed/2.0f;
  return distance_left;
}
float calc_distance_right()
{  
  t=0;
  delayMicroseconds(2000);
  digitalWrite(trig_right,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_right,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right,LOW);
  t=pulseIn(echo_right,HIGH,timeout_ms);
  float distance_right = t*Sspeed/2.0f;
  return distance_right;
}
