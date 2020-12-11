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
        active_thread = None
        if mode == "":
            print("you selected nothing")
            if active_thread:
                active_thread.join()
        elif int(mode) == 3:
            active_thread = threading.Thread(target=app.mode_waypoint_create)
            active_thread.start()
        elif int(mode) == 2:
            active_thread = threading.Thread(target=app.start_patrolling)
            active_thread.start()
        # print(mode)
        # rate.sleep()

main()