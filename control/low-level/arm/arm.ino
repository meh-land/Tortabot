#define __STM32F1__

#include <Servo.h> // include servo library to use its related functions


#include <ros.h> 
#include <std_msgs/Int16MultiArray.h>



#define BASE_PIN    PA3
#define GRIPPER_PIN PA7

#define GRIPPER_OPEN 90
#define GRIPPER_CLOSED 150
#define HIP_HIGH  90
#define HIP_LOW   0


#define DAMPING     5
#define START_TIME 2000
#define ROUTINE 1
#define CALIBRATION_TIME 2000



Servo base;  // Define an instance of of Servo with the name of "MG995_Servo"
Servo gripper;  // Define an instance of of Servo with the name of "MG995_Servo"


ros::NodeHandle nh;  // Initalizing the ROS node


int arm_joints[] = {90, 90};
int arm_state[] = {90, 90};

void damp_write(Servo S, int servo_angle, int setpoint_angle);

void callback_joints(const std_msgs::Int16MultiArray &arm_msg)
{
  arm_joints[0] = constrain(abs(arm_msg.data[0]), HIP_LOW, HIP_HIGH);
  arm_joints[1] = constrain(abs(arm_msg.data[1]), GRIPPER_OPEN, GRIPPER_CLOSED);

  

//  String debug_str = "Got Arm_joints: " + String(arm_joints[0])+ " "+ String(arm_joints[1]);
//  nh.loginfo(&debug_str[0]);
}

ros::Subscriber<std_msgs::Int16MultiArray>  arm_sub("arm_joints",&callback_joints);

void setup() 
{
    base.attach(BASE_PIN);  
    gripper.attach(GRIPPER_PIN);


    nh.initNode();
  
    nh.subscribe(arm_sub);

    delay(CALIBRATION_TIME);

}

void loop() {
  nh.spinOnce();

  damp_write(gripper, arm_state[1], arm_joints[1] );
  arm_state[1] = arm_joints[1];
  delay(ROUTINE);
  
}

void damp_write(Servo S, int servo_angle, int setpoint_angle)
{
  if (setpoint_angle > servo_angle)
  {
    while(setpoint_angle > servo_angle)
    {
      servo_angle++;
      S.write(servo_angle);
      delay(DAMPING);
    }
  }

  else if (setpoint_angle < servo_angle)
  {
    while(setpoint_angle < servo_angle)
    {
      servo_angle--;
      S.write(servo_angle);

      delay(DAMPING);
    }
  }
}
