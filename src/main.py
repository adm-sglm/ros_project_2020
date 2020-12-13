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

    1 : Follow way points
    2 : Create way points

    CTRL-C to quit
    """

    while not rospy.is_shutdown():
        print(msg)
        mode = raw_input("Please select a mode\n")
        print(mode)
        stop_highlight = False
        background_thread = None
        active_thread = None
        background_thread = threading.Thread(target=app.publish_markers, args=(lambda : stop_highlight, ))
        background_thread.start()
        stop_threads = False
        if mode == "":
            stop_threads = True
            if active_thread:
                active_thread.join()
        elif int(mode) == 2:
            active_thread = threading.Thread(target=app.mode_waypoint_create, args=(lambda : stop_threads, ))
            active_thread.start()
        elif int(mode) == 1:
            active_thread = threading.Thread(target=app.start_patrolling, args=(lambda : stop_threads, ))
            active_thread.start()                

main()