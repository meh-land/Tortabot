#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#define echo PA0
#define trig PA1
#define Sspeed 0.034
float distance=0;
float t =0;
float capacity=0;
int Gyro_X, Gyro_Y, Gyro_Z;
geometry_msgs::Point Mpu_Values;
std_msgs::Float32 dist;
ros::NodeHandle nh;  // Initalizing the ROS node

void setup () {
  
pinMode(echo,INPUT);
pinMode(trig,OUTPUT);
Serial.begin(57600);
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
}
void loop() {

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
ros::Publisher Mpu_pub("Mpu",&Mpu_Values);


Serial.print("X = ");
Serial.print (Gyro_X);
Serial.print(" Y = ");
Serial.print(Gyro_Y);
Serial.print(" Z = ");
Serial.println(Gyro_Z);

digitalWrite(trig,LOW);
delayMicroseconds(2);
digitalWrite(trig,HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);
t=pulseIn(echo,HIGH);
distance = t*Sspeed/2;
Serial.print(" Distance = ");
Serial.println(distance);

}
