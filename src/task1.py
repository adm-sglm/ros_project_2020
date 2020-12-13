#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("adm_task_one")
    rate = rospy.Rate(2)

    msg = Twist()
    msg.linear.x = 0.1       
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

main()