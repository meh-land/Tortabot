#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import numpy
import math
import Model
from std_msgs.msg import Float32MultiArray

import numpy as np

# a set of strings of all currently-pressed keys
WHEEL_RADIUS = 0.04    # 4cm
ROBOT_BASELINE = 0.3+0.12   # 30+12cm


# this array will be used to send the kinematic's model data to the robot STM
arr = Float32MultiArray()

# twist_cmd is storing x,y,z linear and angular velocities 
twist_cmd = Twist()

# initializing speed and kinematic model variables
xlinearVelocity  = 0.0           # X position represents Forward and Backwards velocity
ylinearVelocity  = 0.0         
maxAngularVelocity = 0.585     # radian per second
angularVelocity = 0.0


model_output= numpy.zeros(4)

xlinearVelocity  = 0           # X position represents Forward and Backwards velocity
ylinearVelocity  = 0         
maxAngularVelocity = 0.585     # radian per second
angularVelocity = 0



wheel_vels = numpy.zeros(4)
MAX_GAIN = 100000
LINEAR_BIAS = 1000
TURN_BIAS = 3000


# initializing rospy nodes and topics' publisher
rospy.init_node('website_robot_control', anonymous=True)
angular_publisher = rospy.Publisher('/wheel_vel', Float32MultiArray, queue_size=10)


def twist_callback(msg):
    """
    Callback function for handling Twist messages.
    """
    linear_x = msg.linear.x
    linear_y = msg.linear.y
    linear_z = msg.linear.z
    angular_x = msg.angular.x
    angular_y = msg.angular.y
    angular_z = msg.angular.z
    
    robot_vels = np.array([[linear_x, linear_y, angular_z]])

    # Applying kinematic Model to produce velocities on each wheel
    wheel_vels = Model.kinematicModel(R=WHEEL_RADIUS, b=ROBOT_BASELINE).mecanum4_wheels_inverse(robot_vels)

    # publish the results to see speeds
    arr.data = wheel_vels

    # publish the model data to the robot
    angular_publisher.publish(arr)
    

pub = rospy.Subscriber('/cmd_vel', Twist, twist_callback)
rospy.spin()
