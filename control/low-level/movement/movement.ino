#define __STM32F1__

#include <ros.h> 
#include <std_msgs/Float32MultiArray.h>
#include <Timer2.h>
#include "Encoder.h"
#include "PID.h"
#include "Config.h"
#include "ISR_Encoder.h"

void SpeedControl();
void update_setpoint(const std_msgs::Float32MultiArray &object);
void update_parameters(const std_msgs::Float32MultiArray &object);
void set_paramters();
void Forward(int index);
void Backward(int index);


Timer2 timer;

ros::NodeHandle nh;  // Initalizing the ROS node
ros::Subscriber<std_msgs::Float32MultiArray> joystick_sub("/wheel_velocities", update_setpoint);
ros::Subscriber<std_msgs::Float32MultiArray> gains_sub("/PID_Parameters", update_parameters);


std_msgs::Float32MultiArray Espeed_msg;
std_msgs::Float32MultiArray PID_msg;

ros::Publisher Espeed_pub("Espeeds",&Espeed_msg);
ros::Publisher PID_pub("PIDs", &PID_msg);

//Creating Objects from the PID class and assigning them their parameters
PIDControl MOTOR[] {PIDControl(Kp0,Ki0,Kd0,sampleTime),
                    PIDControl(Kp1,Ki1,Kd1,sampleTime),
                    PIDControl(Kp2,Ki2,Kd2,sampleTime),
                    PIDControl(Kp3,Ki3,Kd3,sampleTime)};

//Encoder Objects
ISR_Encoder MOTOR_Encoder[] {ISR_Encoder(RESOLUTION,EnPin0,EnPin1),
                             ISR_Encoder(RESOLUTION,EnPin2,EnPin3),
                             ISR_Encoder(RESOLUTION,EnPin4,EnPin5),
                             ISR_Encoder(RESOLUTION,EnPin6,EnPin7)};



//Initializing some arrays
float encoder_feedback[] = {0, 0, 0, 0};
float PID[] = {0, 0, 0, 0};
bool dir[] = {LOW ,LOW ,LOW ,LOW ,LOW ,LOW, LOW, LOW};



void setup() 
{ 
  //Encoders Setup
  MOTOR_Encoder[0].setup();
  MOTOR_Encoder[1].setup();
  MOTOR_Encoder[2].setup();
  MOTOR_Encoder[3].setup();
  //Seting the modes of the speed pins  
  pinMode(MOTOR0_EN, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR3_EN, OUTPUT);

  //Setting the modes of the direction pins
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);  

  analogWriteResolution(16);

  timer.every(TIME_FREQ, TimerFunction);

  //ROS node setup
  nh.initNode();
  nh.advertise(Espeed_pub);
  nh.advertise(PID_pub);
  nh.subscribe(joystick_sub);
  nh.subscribe(gains_sub);

  //*********************** Attaching intterupts to the encoder pins ********************************//
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinA), Motor0_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[0].PinB), Motor0_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinA), Motor1_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[1].PinB), Motor1_ISR_EncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[2].PinA), Motor2_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[2].PinB), Motor2_ISR_EncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[3].PinA), Motor3_ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_Encoder[3].PinB), Motor3_ISR_EncoderB, CHANGE);

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
  encoder_feedback[2] = { MOTOR_Encoder[2].calcspeed()};
  encoder_feedback[3] = { MOTOR_Encoder[3].calcspeed()};

  Espeed_msg.data_length= 4;
  Espeed_msg.data= encoder_feedback;
  SpeedControl();
  
  Espeed_pub.publish(&Espeed_msg);
}

void update_parameters(const std_msgs::Float32MultiArray& object)
{
 
  for (int i = 0; i < 4; i++) 
  {
    MOTOR[i].set_parameters(object.data[3*i], object.data[3*i+1], object.data[3*i+2]);
  }

}

void update_setpoint(const std_msgs::Float32MultiArray& object)
{

  for (int i = 0; i < numberOfVelocities; i++) 
  {
    MOTOR[i].set_setpoint(object.data[i]);
  }
}

void SpeedControl() 
{ 
  for(int i = 0; i < numberOfVelocities; i++)
  {
      PID[i] = MOTOR[i].calculateOutput(encoder_feedback[i]);
      PID[i] = constrain(PID[i], minOut, maxOut);

      if(PID[i] >= 0)
      {
        Forward(i);
      }
      else
      {
        Backward(i);
      }
  }


  digitalWrite(MOTOR0_IN1, dir[0]);
  digitalWrite(MOTOR0_IN2, dir[1]);

  digitalWrite(MOTOR1_IN1, dir[2]);
  digitalWrite(MOTOR1_IN2, dir[3]);

  digitalWrite(MOTOR2_IN1, dir[4]);
  digitalWrite(MOTOR2_IN2, dir[5]);

  digitalWrite(MOTOR3_IN1, dir[6]);
  digitalWrite(MOTOR3_IN2, dir[7]);

  PID_msg.data_length = 4;
  PID_msg.data = PID;
  PID_pub.publish(&PID_msg);

  analogWrite(MOTOR0_EN, abs(PID[0]));
  analogWrite(MOTOR1_EN, abs(PID[1]));
  analogWrite(MOTOR2_EN, abs(PID[2]));
  analogWrite(MOTOR3_EN, abs(PID[3]));

}
void Forward(int index)
{
    dir[2*index] = HIGH;
    dir[2*index+1] = LOW;
}

void Backward(int index)
{
    dir[2*index] = LOW;
    dir[2*index+1] = HIGH;
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

void Motor2_ISR_EncoderA()
{
  MOTOR_Encoder[2].update_ISR_A();
}

void Motor2_ISR_EncoderB()
{
  MOTOR_Encoder[2].update_ISR_B();
}

void Motor3_ISR_EncoderA()
{
  MOTOR_Encoder[3].update_ISR_A();
}

void Motor3_ISR_EncoderB()
{
  MOTOR_Encoder[3].update_ISR_B();
}
