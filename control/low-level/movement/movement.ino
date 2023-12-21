#define __STM32F1__

#include <ros.h> 
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

//#include <Timer2.h>
#include "Encoder.h"
#include "Config.h"
#include "ISR_Encoder.h"
#include "PID.h"

ros::NodeHandle nh;  // Initalizing the ROS node

void speedControl();

//Encoder Objects
ISR_Encoder MOTOR_Encoder[] {ISR_Encoder(RESOLUTION,ENCODER_FL_A, ENCODER_FL_B),
                             ISR_Encoder(RESOLUTION,ENCODER_FR_A, ENCODER_FR_B),
                             ISR_Encoder(RESOLUTION,ENCODER_BL_A, ENCODER_BL_B),
                             ISR_Encoder(RESOLUTION,ENCODER_BR_A, ENCODER_BR_B)
                             };

PIDControl PID_Controller[] {
                              PIDControl(kp_init,ki_init,kd_init,1.0f/TIME_FREQ),
                              PIDControl(kp_init,ki_init,kd_init,1.0f/TIME_FREQ),
                              PIDControl(kp_init,ki_init,kd_init,1.0f/TIME_FREQ),
                              PIDControl(kp_init,ki_init,kd_init,1.0f/TIME_FREQ)
                              };
float wheel_speeds [] = {0, 0, 0, 0};
long wheel_pwm [] = {0,0,0,0};
//Timer2 timer;
std_msgs::Float32MultiArray Espeed_msg;
std_msgs::Int32MultiArray pwm_msg;
std_msgs::Float32MultiArray pid_consts;
std_msgs::Int32MultiArray counts_msg;

void callback_speeds(const std_msgs::Float32MultiArray &speeds_msg)
{
  wheel_speeds[0] = speeds_msg.data[0];
  wheel_speeds[1] = speeds_msg.data[1];
  wheel_speeds[2] = speeds_msg.data[2];
  wheel_speeds[3] = speeds_msg.data[3];

  PID_Controller[0].set_setpoint(wheel_speeds[0]);
  PID_Controller[1].set_setpoint(wheel_speeds[1]);
  PID_Controller[2].set_setpoint(wheel_speeds[2]);
  PID_Controller[3].set_setpoint(wheel_speeds[3]);
  

  String debug_str = "Got Speeds: " + String(wheel_speeds[0])+ " "+ String(wheel_speeds[1])+ " "+ String(wheel_speeds[2])+ " "+ String(wheel_speeds[3]);
  nh.loginfo(&debug_str[0]);
}

void callback_pid(const std_msgs::Float32MultiArray &pid_msg)
{
  float kp = pid_msg.data[0];
  float ki = pid_msg.data[1];
  float kd = pid_msg.data[2];

  PID_Controller[0].set_parameters(kp,ki,kd);
  PID_Controller[1].set_parameters(kp,ki,kd);
  PID_Controller[2].set_parameters(kp,ki,kd);
  PID_Controller[3].set_parameters(kp,ki,kd);

  String debug_str = "Got PID: " + String(kp)+ " " + String(ki) +" " + String(kd);
  nh.loginfo(&debug_str[0]);
}


ros::Publisher Espeed_pub("Espeeds",&Espeed_msg);
ros::Publisher counts_pub("counts",&counts_msg);
ros::Publisher pwm_pub("pwm",&pwm_msg);

ros::Subscriber<std_msgs::Float32MultiArray> 
wheel_vel_sub("wheel_vel",&callback_speeds);

ros::Subscriber<std_msgs::Float32MultiArray> 
pid_consts_sub("PID_CONST",&callback_pid);





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
  nh.advertise(pwm_pub);
  
  nh.subscribe(wheel_vel_sub);
  nh.subscribe(pid_consts_sub);


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

  PID_control();
  
  pwm_msg.data_length = 4;
  pwm_msg.data = wheel_pwm;
  
  speedControl();
  
  Espeed_pub.publish(&Espeed_msg);
  counts_pub.publish(&counts_msg);
  pwm_pub.publish(&pwm_msg);
}

void PID_control()
{
  wheel_pwm[0] = PID_Controller[0].calculateOutput(encoder_feedback[0]);
  wheel_pwm[1] = PID_Controller[1].calculateOutput(encoder_feedback[1]);
  wheel_pwm[2] = PID_Controller[2].calculateOutput(encoder_feedback[2]);
  wheel_pwm[3] = PID_Controller[3].calculateOutput(encoder_feedback[3]);
}

void speedControl()
{
  // Drive Motor FL
  digitalWrite(MOTOR_FL_IN1, wheel_pwm[0] > 0);
  digitalWrite(MOTOR_FL_IN2, wheel_pwm[0] < 0);
  analogWrite(MOTOR_FL_EN, constrain(abs(wheel_pwm[0]), MIN_VEL, MAX_VEL));


  // Drive Motor FR
  digitalWrite(MOTOR_FR_IN1, wheel_pwm[1] > 0);
  digitalWrite(MOTOR_FR_IN2, wheel_pwm[1] < 0);
  analogWrite(MOTOR_FR_EN, constrain(abs(wheel_pwm[1]), MIN_VEL, MAX_VEL));

  // Drive Motor BL
  digitalWrite(MOTOR_BL_IN1, wheel_pwm[2] > 0);
  digitalWrite(MOTOR_BL_IN2, wheel_pwm[2] < 0);
  analogWrite(MOTOR_BL_EN, constrain(abs(wheel_pwm[2]), MIN_VEL, MAX_VEL));


  // Drive Motor BR
  digitalWrite(MOTOR_BR_IN1, wheel_pwm[3] > 0);
  digitalWrite(MOTOR_BR_IN2, wheel_pwm[3] < 0);
  analogWrite(MOTOR_BR_EN, constrain(abs(wheel_pwm[3]), MIN_VEL, MAX_VEL));

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
