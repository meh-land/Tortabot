#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math
angle_set = False
start_angles = 0, 0, 0
new_angle = 0
last_angle = 0
C =0

def limit_point(curr, prev):
    '''
    Check that IMU reading pass which breakpoint 
    True : ... 179 180 -180 -179 ...
    False: ... 2 1 0 -1 -2 ...
    '''
    total = abs(curr) + abs(prev)
    if total >= 180:
        return 1
    else :
        return 0

def break_limit(scl_imu, prev_imu, curr_imu):
    if prev_imu * curr_imu >= 0:
        ''' When both have same sign (no breakpoint occurs) '''
        diff = curr_imu - prev_imu
    
    else :
        ''' When both have different sign (breakpoint occurs) '''
        if limit_point(curr_imu, prev_imu):
            ''' When second breakpoint occurs '''
            if curr_imu < 0 and prev_imu >= 0 :
                ''' Check Direction of IMU '''
                diff = (180 - prev_imu) + (180 + curr_imu)
                
            else :
                diff = (-180 - prev_imu) + (-180 + curr_imu) 

        else:
            ''' When first breakpoint occurs '''
            if curr_imu < 0 and prev_imu >= 0 :
                ''' Check Direction of IMU '''
                diff = curr_imu - prev_imu
                
            else :
                diff = curr_imu - prev_imu
    
    return scl_imu + diff


def imu_callback(imu_msg):
    global imu_pub,start_angles,angle_set,imu_yaw_pub,new_angle,last_angle
    
    # Extracting the orientation data from the IMU message
    orientation = imu_msg.orientation
    orientation_list = [orientation.x,
                        orientation.y, orientation.z, orientation.w]

    # Converting quaternion to Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = euler_from_quaternion(orientation_list)

    if not angle_set:
        start_angles = roll,pitch,yaw
        angle_set = True
        last_angle = math.degrees(yaw)

    else:
        roll -= start_angles[0]
        pitch -= start_angles[1]
        yaw -= start_angles[2]
    
    # Printing the Euler angles
    # print("Yaw: {:.2f} degrees, Pitch: {:.2f} degrees, Roll: {:.2f} degrees".format(
    #     math.degrees(yaw), math.degrees(pitch), math.degrees(roll)
    # ))
    rpy = Vector3()
    rpy.x = math.degrees(roll)
    rpy.y = math.degrees(pitch)
    rpy.z = math.degrees(yaw)
    
    new_angle = break_limit(scl_imu=new_angle,prev_imu=last_angle, curr_imu=rpy.z)
    last_angle = rpy.z

    imu_pub.publish(rpy)
    imu_yaw_msg = Float32()
    imu_yaw_msg.data = new_angle
    imu_yaw_pub.publish(imu_yaw_msg)


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


if __name__ == '__main__':  
    try:
        rospy.init_node('phone_imu_driver', anonymous=True)
        rospy.Subscriber('android/imu', Imu, imu_callback)
        imu_pub = rospy.Publisher("imu/euler", Vector3, queue_size=10)
        imu_yaw_pub = rospy.Publisher("imu/yaw/deg", Float32, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


