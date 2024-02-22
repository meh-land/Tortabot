#define __STM32F1__

#include "Ultrasonic.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <Wire.h>

int Gyro_X, Gyro_Y, Gyro_Z;
geometry_msgs::Point Mpu_Values;
std_msgs::Float32MultiArray dist_msg;

float dist[4]={0,0,0,0};

ros::NodeHandle nh;  // Initalizing the ROS node
ros::Publisher Mpu_pub("Mpu",&Mpu_Values);
ros::Publisher dist_pub("Ultrasonics",&dist_msg);

void setup () 
{

ultrasonic_init();

//Serial.begin(57600);
Wire.setClock (400000);
Wire.begin();
delay (250);
//Start serial port at 57600bps

//Set the I2C clock speed to 400kHz

//Start the I2C as master

//Give the gyro time to start
Wire.setSDA(PB11);
Wire.setSCL(PB10);
Wire.begin();
Wire.beginTransmission (0x68); //Start communication with the MPU-6050

Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();
//Start writing at this register (PWR MGMT 1)

//Set register 0x6B to zero (wakes up the MPU-6050)

//Terminate the connection


  //ROS node setup
  nh.initNode();
  nh.advertise(Mpu_pub);
  nh.advertise(dist_pub);

  
}

void loop() {
nh.spinOnce();
/*
Wire.beginTransmission (0x68);

Wire.write(0x43);

Wire.endTransmission();

Wire.requestFrom(0x68,6);

//Start communication with the MPU-6050

//Sent byte 0x43 to indicate startregister

//Terminate the connection

//Request 6 bytes from the MPU 6050

Gyro_X = Wire.read()<<8 | Wire.read();

//Shift high byte left and add low and high byte to Gyro_X

Gyro_Y = Wire.read()<<8 | Wire.read(); //Shift high byte left and add low and high byte to Gyro_Y

Gyro_Z = Wire.read()<<8 | Wire.read(); //Shift high byte left and add low and high byte to Gyro_2

Mpu_Values.x=Gyro_X;
Mpu_Values.y=Gyro_Y;
Mpu_Values.z=Gyro_Z;
*/

Mpu_pub.publish(&Mpu_Values);
dist[0] = calc_distance_front();
dist[1] = calc_distance_back();
dist[2] = calc_distance_left();
dist[3] = calc_distance_right();

dist_msg.data_length= 4;
dist_msg.data= dist;
dist_pub.publish(&dist_msg);
delay(100);

}
