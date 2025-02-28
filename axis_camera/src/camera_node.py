#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import requests
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_URL = "http://192.168.0.20/mjpg/video.mjpg"  # Atualizar de acordo com o IP da câmera

def main():
    rospy.init_node("axis_camera")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(CAMERA_URL)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(ros_image)
        rospy.sleep(0.1)

    cap.release()

if __name__ == "__main__":
    main()