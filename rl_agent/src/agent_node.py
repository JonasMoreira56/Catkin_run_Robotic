#!/usr/bin/env python3
import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

MODEL_PATH = "/home/user/catkin_ws/src/rl_agent/src/model_weights.pth"

class RLAgent:
    def __init__(self):
        rospy.init_node("rl_agent")
        self.bridge = CvBridge()
        self.model = torch.load(MODEL_PATH, map_location="cpu")
        self.model.eval()
        
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        state = self.preprocess_image(cv_image)
        action = self.select_action(state)
        self.send_command(action)

    def preprocess_image(self, image):
        image = cv2.resize(image, (128, 128))
        image = np.transpose(image, (2, 0, 1)) / 255.0
        return torch.tensor(image, dtype=torch.float32).unsqueeze(0)

    def select_action(self, state):
        with torch.no_grad():
            output = self.model(state)
        return torch.argmax(output).item()

    def send_command(self, action):
        cmd = Twist()
        if action == 0:  # Frente
            cmd.linear.x = 0.5
        elif action == 1:  # Esquerda
            cmd.angular.z = 0.5
        elif action == 2:  # Direita
            cmd.angular.z = -0.5
        elif action == 3:  # Parar
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

if __name__ == "__main__":
    RLAgent()
    rospy.spin()
