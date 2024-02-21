
#define __STM32F1__
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>

#include <Wire.h>
#define echo PA1
#define trig PA0
#define Sspeed 0.034
float distance=0;
float t =0;
float capacity=0;
int Gyro_X, Gyro_Y, Gyro_Z;
geometry_msgs::Point Mpu_Values;
float dist[4]={0,0,0,0};
std_msgs::Float32MultiArray dist_msg;

ros::NodeHandle nh;  // Initalizing the ROS node
ros::Publisher Mpu_pub("Mpu",&Mpu_Values);
ros::Publisher dist_pub("Ultrasonics",&dist_msg);

void setup () 
{


  
pinMode(echo,INPUT);
pinMode(trig,OUTPUT);
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


//Serial.print("X = ");
//Serial.print (Gyro_X);
//Serial.print(" Y = ");
//Serial.print(Gyro_Y);
//Serial.print(" Z = ");
//Serial.println(Gyro_Z);
Mpu_pub.publish(&Mpu_Values);
digitalWrite(trig,LOW);
delayMicroseconds(2);
digitalWrite(trig,HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);
t=pulseIn(echo,HIGH);
dist[0] = t*Sspeed/2;

  dist_msg.data_length= 4;
  dist_msg.data= dist;
dist_pub.publish(&dist_msg);
//Serial.print(" Distance = ");
//Serial.println(distance);

delay(100);

}
