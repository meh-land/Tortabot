#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import numpy as np
import math
import Model
from std_msgs.msg import Float32MultiArray


wheel_radius = 0.04 #0.075 
lx = 0.15           #horizontal distance from the center of the robot
ly = 0.36           #vertical distance from the center of the robot

model_output= np.zeros(4)
arr = Float32MultiArray()


def rad_per_sec_to_rpm(speeds):
    return speeds * 60 / (2 * np.pi)


def vel2wheels(msg):
    x = msg.linear.x
    y = msg.linear.y
    t = msg.angular.z

    model_output = Model.kinematicModel(x, y, t, wheel_radius, lx, ly).mecanum_4_vel()
    model_output = rad_per_sec_to_rpm(model_output)


    arr.data = model_output
    # publish the model data to the robot
    angular_publisher.publish(arr)




# initializing rospy nodes and topics' publisher
rospy.init_node('robot_to_mecanum', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, vel2wheels)
angular_publisher = rospy.Publisher('/wheel_vel', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz


rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        pass