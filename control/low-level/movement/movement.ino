#define __STM32F1__

#include <ros.h> 
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

//#include <Timer2.h>
#include "Encoder.h"
#include "Config.h"
#include "ISR_Encoder.h"


void speedControl();

//Encoder Objects
ISR_Encoder MOTOR_Encoder[] {ISR_Encoder(RESOLUTION,ENCODER_FL_A, ENCODER_FL_B),
                             ISR_Encoder(RESOLUTION,ENCODER_FR_A, ENCODER_FR_B),
                             ISR_Encoder(RESOLUTION,ENCODER_BL_A, ENCODER_BL_B),
                             ISR_Encoder(RESOLUTION,ENCODER_BR_A, ENCODER_BR_B)
                             };

long wheel_speeds [] = {0, 0, 0, 0};

//Timer2 timer;
std_msgs::Float32MultiArray Espeed_msg;
std_msgs::Int32MultiArray counts_msg;

void callback_speeds(const std_msgs::Float32MultiArray &speeds_msg)
{
  wheel_speeds[0] = speeds_msg.data[0];
  wheel_speeds[1] = speeds_msg.data[1];
  wheel_speeds[2] = speeds_msg.data[2];
  wheel_speeds[3] = speeds_msg.data[3];
}

ros::NodeHandle nh;  // Initalizing the ROS node
ros::Publisher Espeed_pub("Espeeds",&Espeed_msg);
ros::Publisher counts_pub("counts",&counts_msg);

ros::Subscriber<std_msgs::Float32MultiArray> 
wheel_vel_sub("wheel_vel",&callback_speeds);






//Initializing some arrays
long encoder_counts[] = {0, 0, 0, 0};
float encoder_feedback[] = {0, 0, 0, 0};




void setup() 
{ 
  //Encoders Setup
  MOTOR_Encoder[0].setup();
  MOTOR_Encoder[1].setup();
  MOTOR_Encoder[2].setup();
  MOTOR_Encoder[3].setup();
  
  //Seting the modes of the speed pins  
  pinMode(MOTOR_FL_EN, OUTPUT);
  pinMode(MOTOR_FR_EN, OUTPUT);
  pinMode(MOTOR_BL_EN, OUTPUT);
  pinMode(MOTOR_BR_EN, OUTPUT);

  //Setting the modes of the direction pins
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_BL_IN1, OUTPUT);
  pinMode(MOTOR_BL_IN2, OUTPUT);
  pinMode(MOTOR_BR_IN1, OUTPUT);
  pinMode(MOTOR_BR_IN2, OUTPUT);
  
  analogWriteResolution(16);

//  timer.every(TIME_FREQ, TimerFunction);

  //ROS node setup
  nh.initNode();
  nh.advertise(Espeed_pub);
  nh.advertise(counts_pub);
  
  nh.subscribe(wheel_vel_sub);


  //*********************** Attaching intterupts to the encoder pins ********************************//
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinA), Motor_FL_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinB), Motor_FL_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinA), Motor_FR_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinB), Motor_FR_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[2].PinA), Motor_BL_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[2].PinB), Motor_BL_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[3].PinA), Motor_BR_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[3].PinB), Motor_BR_ISR_EncoderB, CHANGE);




}


long long lastTime=0;
void loop() 
{
  nh.spinOnce();

//  timer.update();
  if(millis()-lastTime>= 1000/TIME_FREQ)
  {
     TimerFunction();
     lastTime = millis();
  }

}


void TimerFunction() 
{

  encoder_feedback[0] = { MOTOR_Encoder[0].calcspeed()};  
  encoder_feedback[1] = { MOTOR_Encoder[1].calcspeed()};
  encoder_feedback[2] = { MOTOR_Encoder[2].calcspeed()};
  encoder_feedback[3] = { MOTOR_Encoder[3].calcspeed()};

  encoder_counts[0] = { MOTOR_Encoder[0].counts};  
  encoder_counts[1] = { MOTOR_Encoder[1].counts};
  encoder_counts[2] = { MOTOR_Encoder[2].counts};
  encoder_counts[3] = { MOTOR_Encoder[3].counts};

  Espeed_msg.data_length= 4;
  Espeed_msg.data= encoder_feedback;

  counts_msg.data_length= 4;
  counts_msg.data= encoder_counts;

  speedControl();
  
  Espeed_pub.publish(&Espeed_msg);
  counts_pub.publish(&counts_msg);
}


void speedControl()
{
  // Drive Motor FL
  if (wheel_speeds[0] > 0)
  {
    digitalWrite(MOTOR_FL_IN1, HIGH);
    digitalWrite(MOTOR_FL_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_FL_IN1, LOW);
    digitalWrite(MOTOR_FL_IN2, HIGH);
  }
  analogWrite(MOTOR_FL_EN, constrain(abs(wheel_speeds[0]), MIN_VEL, MAX_VEL));


  // Drive Motor FR
  if (wheel_speeds[1] > 0)
  {
    digitalWrite(MOTOR_FR_IN1, HIGH);
    digitalWrite(MOTOR_FR_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_FR_IN1, LOW);
    digitalWrite(MOTOR_FR_IN2, HIGH);
  }
  analogWrite(MOTOR_FR_EN, constrain(abs(wheel_speeds[1]), MIN_VEL, MAX_VEL));

    // Drive Motor BL
  if (wheel_speeds[2] > 0)
  {
    digitalWrite(MOTOR_BL_IN1, HIGH);
    digitalWrite(MOTOR_BL_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_BL_IN1, LOW);
    digitalWrite(MOTOR_BL_IN2, HIGH);
  }
  analogWrite(MOTOR_BL_EN, constrain(abs(wheel_speeds[2]), MIN_VEL, MAX_VEL));


  // Drive Motor BR
  if (wheel_speeds[3] > 0)
  {
    digitalWrite(MOTOR_BR_IN1, HIGH);
    digitalWrite(MOTOR_BR_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_BR_IN1, LOW);
    digitalWrite(MOTOR_BR_IN2, HIGH);
  }
  analogWrite(MOTOR_BR_EN, constrain(abs(wheel_speeds[3]), MIN_VEL, MAX_VEL));

}

void Motor_FL_ISR_EncoderA()
{
  MOTOR_Encoder[0].update_ISR_A();
}

void Motor_FL_ISR_EncoderB()
{
  MOTOR_Encoder[0].update_ISR_B();
}

void Motor_FR_ISR_EncoderA()
{
  MOTOR_Encoder[1].update_ISR_A();
}

void Motor_FR_ISR_EncoderB()
{
  MOTOR_Encoder[1].update_ISR_B();
}

void Motor_BL_ISR_EncoderA()
{
  MOTOR_Encoder[2].update_ISR_A();
}

void Motor_BL_ISR_EncoderB()
{
  MOTOR_Encoder[2].update_ISR_B();
}

void Motor_BR_ISR_EncoderA()
{
  MOTOR_Encoder[3].update_ISR_A();
}

void Motor_BR_ISR_EncoderB()
{
  MOTOR_Encoder[3].update_ISR_B();
}
