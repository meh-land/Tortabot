#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
import subprocess
import glob
# from camera_info_manager import CameraInfoManager
from typing import List
from cv_bridge import CvBridge
import yaml
import rospkg

cap = cv2.VideoCapture(2)

# while  cap.isOpened():
#     ret, frame = cap.read()

#     cv2.imshow("frame", frame)
#     cv2.waitKey(1)

class StereoCamera:
    def __init__(self) -> None:
        self.pub_raw = rospy.Publisher("image_raw", Image, queue_size=10)
        self.set_frame_size()


    def set_frame_size(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        rospy.loginfo(f"Started streaming from camera {2} ")

    def stream_stereo_camera(self):
        ret, frame = self.cap.read()
        self.current_image = frame
        
        if frame is None:
            print("No camera frame received")
            return

        raw_frame_msg = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')
        raw_frame_msg.header.frame_id = "camera_optical_frame"
        header = raw_frame_msg.header
        header.stamp = rospy.Time.now()
        self.pub_raw.publish(raw_frame_msg)

rospy.init_node('stereo_driver', anonymous=True)
stereo_streamer = StereoCamera()
while True:
    stereo_streamer.stream_stereo_camera()
# cv2.destroyAllWindows()