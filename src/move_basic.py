#! /usr/bin/env python
import rospy
from classes.app import App

def main():
    rospy.init_node("adm")
    rate = rospy.Rate(2)

    app = App(rospy)

    msg = """
    Available Modes
    ---------------------------

    3 : Create way points
    9 : Simple move with /cmd_vel

    CTRL-C to quit
    """

    while not rospy.is_shutdown():
        print(msg)        
        mode = input("Please select a mode")
        print("You selected")
        if mode == 3:
            app.mode_waypoint_create()
        print(mode)
        rate.sleep()



main()