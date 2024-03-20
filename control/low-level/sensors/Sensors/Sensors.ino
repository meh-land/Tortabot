
#define __STM32F1__

#include "Ultrasonic.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
std_msgs::Float32 Mpu_Values;
std_msgs::Float32MultiArray dist_msg;

float dist[4]={0,0,0,0};

ros::NodeHandle nh;  // Initalizing the ROS node
ros::Publisher Mpu_pub("/imu/yaw/deg",&Mpu_Values);
ros::Publisher dist_pub("Ultrasonics",&dist_msg);
float euler[3];         // [psi, theta, phi]    Euler angle container
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

void setup () 
{

 ultrasonic_init();

//Give the gyro time to start
  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  
  Wire.begin();
  Wire.setClock (400000);
  ///delay (250);
//Start serial port at 57600bps

//Set the I2C clock speed to 400kHz

//Start the I2C as master


//Wire.beginTransmission (0x68); //Start communication with the MPU-6050
// Initialize the MPU6050
    mpu.initialize();
    mpu.dmpInitialize();
    
// Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

//mpu.PrintActiveOffsets();   
    mpu.setDMPEnabled(true);

//ROS node setup
  nh.initNode();
  nh.advertise(Mpu_pub);
  nh.advertise(dist_pub);
  delay(1000);
}

  void loop() {
  nh.spinOnce();

  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Mpu_Values.data =euler[0] * 180/M_PI;
    Mpu_pub.publish(&Mpu_Values);
  }

  dist[0] = calc_distance_front();
  dist[1] = calc_distance_back();
  dist[2] = calc_distance_left();
  dist[3] = calc_distance_right();
  
  dist_msg.data_length= 4;
  dist_msg.data= dist;
  dist_pub.publish(&dist_msg);
  delay(100);

}
