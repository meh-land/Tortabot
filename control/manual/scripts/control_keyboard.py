#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import numpy
import math
import Model
from std_msgs.msg import Float32MultiArray

import numpy as np

# a set of strings of all currently-pressed keys
pressed_keys = set()

WHEEL_RADIUS = 0.03    # 3cm
ROBOT_BASELINE = 0.3   # 30cm
'''
MANUAL USAGE to control the robot.

for movement:
        W -> Forward
    A   S   D -q> Right
    |    \ 
    v     v
   Left  Backward
Q: for clockwise turn
E: for anti-clockwise turn

hold L-SHIFT to speed up (limit= MAX_VEL)
hold L-CTRL to speed down (limit= 0)
# when releasing shift/ctrl key, the gain speed will be reset.

Press ESC to turn off the program
'''

# initializing rospy nodes and topics' publisher
rospy.init_node('keyboard_robot_control', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
angular_publisher = rospy.Publisher('/wheel_vel', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

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

# You can tune the following constants to your best fit
# declaring constants of speed in all directions:
VERTICAL=3000.0          #'w','s'
HORIZONTAL=3000.0        #'d','a'
CLOCKWISE=3000       # Q
ANTICLOCKWISE=3000  # E

gain=float(1)# initializing variables
xlinearVelocity  = 0           # X position represents Forward and Backwards velocity
ylinearVelocity  = 0         
maxAngularVelocity = 0.585     # radian per second
angularVelocity = 0



wheel_vels = numpy.zeros(4)
MAX_GAIN = 100000
LINEAR_BIAS = 1000
TURN_BIAS = 3000

# function called on press of the key
def on_press(key):
    #globalizing variables
    global xlinearVelocity, ylinearVelocity, angularVelocity,wheel_vels, gain, pressed_keys
    # add the currently pressed key
    # if it's alphanumeric:
    if hasattr(key, 'char'):
        pressed_keys.add(str(key.char).lower()) # the character is in lowercase to make (i.e: 'A'/'a') the same
    else:
        # For special keys, adding their name to the list
        pressed_keys.add(str(key)) ## for example: shift will be 'Key.shift'
    
    seeKeyboard() # check the pressed keys
    # store the x,y,z of linear velocity and angular velocity
    twist_cmd.linear.x = xlinearVelocity  
    twist_cmd.linear.y = ylinearVelocity
    twist_cmd.angular.z = angularVelocity
    
    robot_vels = np.array([[xlinearVelocity, ylinearVelocity, angularVelocity]])

    # Applying kinematic Model to produce velocities on each wheel
    wheel_vels = Model.kinematicModel(R=WHEEL_RADIUS, b=ROBOT_BASELINE).omni4_wheels_inverse(robot_vels)

    # publish the results to see speeds
    pub.publish(twist_cmd)
    arr.data = wheel_vels

    # publish the model data to the robot
    angular_publisher.publish(arr)

# function called on release of key 
def on_release(key):
    global xlinearVelocity, ylinearVelocity, angularVelocity,wheel_vels,  gain, pressed_keys
    # remove the key released
    if hasattr(key, 'char'): #alphanumeric character
        try:
            pressed_keys.remove(str(key.char).lower())
        except (ValueError, KeyError) as e:
            print(key)
            print(str(key.char))
            # the released shift key is acting as a NoneObject  with code <<66032>>
            if ('Key.shift' in pressed_keys):
                # when releasing shift key, the gain speed will be reset.
                pressed_keys.remove('Key.shift')
                gain = 1
            pass
    else: # special character
        if (str(key) in pressed_keys):
            pressed_keys.remove(str(key))
    
    if (not hasattr(key, 'char')): # if it's a special character released
        print("Special Character released")
    if(key==keyboard.Key.esc):
        exit() # Exit the program when ESC pressed
    if (key==keyboard.Key.shift):
        # when releasing shift key, the gain speed will be reset.
        if ('Key.shift' in pressed_keys):
            pressed_keys.remove('Key.shift')
        gain = 1
    if (key==keyboard.Key.ctrl): # CTRL released
        gain = 1
        
    seeKeyboard() # check the pressed keys


    # store the x,y,z of linear velocity and angular velocity
    twist_cmd.linear.x = xlinearVelocity 
    twist_cmd.linear.y = ylinearVelocity
    twist_cmd.angular.z = angularVelocity
    robot_vels = np.array([[xlinearVelocity, ylinearVelocity, angularVelocity]])

    # Applying kinematic Model to produce velocities on each wheel
    wheel_vels = Model.kinematicModel(R=WHEEL_RADIUS, b=ROBOT_BASELINE).omni4_wheels_inverse(robot_vels)

    # publish the results to see speed
    pub.publish(twist_cmd)
    arr.data = wheel_vels

    # publish the model data to the robot
    angular_publisher.publish(arr)

# function checks for pressed keys
def seeKeyboard():
    global gain,xlinearVelocity,ylinearVelocity,angularVelocity,pressed_keys
    xlinear=0
    ylinear=0
    angular=0

    if 'Key.esc' in pressed_keys:
        exit()

    if 'Key.shift' in pressed_keys:
        if (gain + 10 <= float(MAX_GAIN)): # limit the speed gain = MAX_GAIN
            gain += 10 

    if 'Key.ctrl' in pressed_keys:
        if (gain-10 >= 0): # limit = zero
            gain -= 10



    if 'd' in pressed_keys:
        #ylinearVelocity=VERTICAL*gain
        ylinear+=VERTICAL*gain + LINEAR_BIAS

    if 'w' in pressed_keys:
        #xlinearVelocity=HORIZONTAL*gain
        xlinear+=HORIZONTAL*gain + LINEAR_BIAS


    if 'a' in pressed_keys:
        #ylinearVelocity=-VERTICAL*gain
        ylinear-=VERTICAL*gain + LINEAR_BIAS

    if 's' in pressed_keys:
        #xlinearVelocity=-HORIZONTAL*gain
        xlinear-=HORIZONTAL*gain + LINEAR_BIAS

    if 'q' in pressed_keys:
        #angularVelocity=ANTICLOCKWISE*gain
        angular-=ANTICLOCKWISE*gain + TURN_BIAS

    if 'e' in pressed_keys:
        #angularVelocity=CLOCKWISE*gain
        angular+=CLOCKWISE*gain + TURN_BIAS

    angularVelocity=angular
    xlinearVelocity=xlinear
    ylinearVelocity=ylinear

def control_robot():
    while not rospy.is_shutdown():
        with keyboard.Listener( on_press=on_press, on_release=on_release) as listener: 
            listener.join()
        

if __name__ == '__main__':
    control_robot()