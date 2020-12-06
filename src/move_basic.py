#! /usr/bin/env python
import rospy

rospy.init_node("adm")
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    print("tick")
    rate.sleep()