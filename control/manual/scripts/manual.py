#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import numpy as np

REDIUS = 0.06
b = 0.3
WEIGHT = 65000

def get_vels(data):
    x = data.linear.x
    y = data.linear.y
    theta = data.angular.z

    rw = WEIGHT * ( (x/2) / (2 * np.pi * REDIUS) + theta )
    rl = WEIGHT * ( (x/2) / (2 * np.pi * REDIUS) - theta )



    wheel_vel_msg.data = [rl, rw]
    wheel_vel_pub.publish(wheel_vel_msg)
    



wheel_vel_msg = Float32MultiArray()


rospy.init_node("manual_node")

rospy.Subscriber(name='cmd_vel', data_class=Twist, callback=get_vels)

wheel_vel_pub = rospy.Publisher(name='wheel_vel', data_class=Float32MultiArray, queue_size=10)

rospy.spin()