#! /usr/bin/env python
import rospy
import threading
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
        mode = raw_input("Please select a mode")
        print(mode)
        if mode == "":
            print("you selected nothing")
        elif int(mode) == 3:
            app_thread = threading.Thread(target=app.mode_waypoint_create)
            app_thread.start()
            # app.mode_waypoint_create()
        # print(mode)
        # rate.sleep()

main()