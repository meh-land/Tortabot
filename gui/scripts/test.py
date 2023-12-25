#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage, Image







def get_image(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
        
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    
    print(image.shape)

rospy.init_node("Dearpy_Node")
rospy.Subscriber("camera/image/compressed", CompressedImage,  get_image, queue_size=1)

rospy.spin()