#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import random
import time

def move_robot():

    z = random.uniform(-0.3,0.3) 
    #print(z)
     # Create a Twist message to represent the desired robot movement
    twist_msg = Twist()
    # Create a Twist message to represent the desired robot movement
    twist_msg.linear.x = 0.5  # Example: forward linear velocity
    twist_msg.angular.z = z  # Example: rotational velocity

    rate = rospy.Rate(1)  # Loop rate (10 Hz)

    for i in range(10):
        # Publish the Twist message to the '/cmd_vel' topic
        pub.publish(twist_msg)
        rate.sleep()  

def stop_robot():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0

    pub.publish(twist)
        

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    pub = rospy.Publisher('/sim_actor/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        move_robot()
