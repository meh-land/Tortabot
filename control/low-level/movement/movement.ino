#define __STM32F1__

#include <ros.h> 
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <Timer2.h>
#include "Encoder.h"
#include "Config.h"
#include "ISR_Encoder.h"


void speedControl();

//Encoder Objects
ISR_Encoder MOTOR_Encoder[] {ISR_Encoder(RESOLUTION,ENCODER0_A, ENCODER0_B),
                             ISR_Encoder(RESOLUTION,ENCODER1_A, ENCODER1_B),};

long wheel_speeds [] = {0, 0};

Timer2 timer;
std_msgs::Float32MultiArray Espeed_msg;
std_msgs::Int32MultiArray counts_msg;

void callback_speeds(const std_msgs::Float32MultiArray &speeds_msg)
{
  wheel_speeds[0] = speeds_msg.data[0];
  wheel_speeds[1] = speeds_msg.data[1];

}


ros::NodeHandle nh;  // Initalizing the ROS node
ros::Publisher Espeed_pub("Espeeds",&Espeed_msg);
ros::Publisher counts_pub("counts",&counts_msg);

ros::Subscriber<std_msgs::Float32MultiArray> wheel_vel_sub("wheel_vel",&callback_speeds);






//Initializing some arrays
long encoder_counts[] = {0, 0};
float encoder_feedback[] = {0, 0};




void setup() 
{ 
  //Encoders Setup
  MOTOR_Encoder[0].setup();
  MOTOR_Encoder[1].setup();

  //Seting the modes of the speed pins  
  pinMode(MOTOR0_EN, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);


  //Setting the modes of the direction pins
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  analogWriteResolution(16);

  timer.every(TIME_FREQ, TimerFunction);

  //ROS node setup
  nh.initNode();
  nh.advertise(Espeed_pub);
  nh.advertise(counts_pub);
  
  nh.subscribe(wheel_vel_sub);


  //*********************** Attaching intterupts to the encoder pins ********************************//
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinA), Motor0_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinB), Motor0_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinA), Motor1_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinB), Motor1_ISR_EncoderB, CHANGE);



}



void loop() 
{
  nh.spinOnce();

  timer.update();

}


void TimerFunction() 
{

  encoder_feedback[0] = { MOTOR_Encoder[0].calcspeed()};  
  encoder_feedback[1] = { MOTOR_Encoder[1].calcspeed()};

  encoder_counts[0] = { MOTOR_Encoder[0].counts};  
  encoder_counts[1] = { MOTOR_Encoder[1].counts};

  Espeed_msg.data_length= 2;
  Espeed_msg.data= encoder_feedback;

  counts_msg.data_length= 2;
  counts_msg.data= encoder_counts;

  speedControl();
  
  Espeed_pub.publish(&Espeed_msg);
  counts_pub.publish(&counts_msg);
}


void speedControl()
{
  // Drive Motor0
  if (wheel_speeds[0] > 0)
  {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
  }
  analogWrite(MOTOR0_EN, abs(constrain(wheel_speeds[0], MIN_VEL, MAX_VEL)));


  // Drive Motor1
  if (wheel_speeds[1] > 0)
  {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
  }
    analogWrite(MOTOR1_EN, abs(constrain(wheel_speeds[1], MIN_VEL, MAX_VEL)));

  
}

void Motor0_ISR_EncoderA()
{
  MOTOR_Encoder[0].update_ISR_A();
}

void Motor0_ISR_EncoderB()
{
  MOTOR_Encoder[0].update_ISR_B();
}

void Motor1_ISR_EncoderA()
{
  MOTOR_Encoder[1].update_ISR_A();
}

void Motor1_ISR_EncoderB()
{
  MOTOR_Encoder[1].update_ISR_B();
}
