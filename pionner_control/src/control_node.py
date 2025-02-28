#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot(linear=0.0, angular=0.0):
    rospy.init_node("pioneer_control")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    move_robot()
