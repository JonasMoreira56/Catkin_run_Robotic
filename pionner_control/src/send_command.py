#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def send_command(linear, angular):
    rospy.init_node("send_command")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    for _ in range(10):  # Publica o comando 10 vezes
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        linear_speed = float(input("Digite a velocidade linear: "))
        angular_speed = float(input("Digite a velocidade angular: "))
        send_command(linear_speed, angular_speed)
    except rospy.ROSInterruptException:
        pass