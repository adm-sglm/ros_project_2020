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

    1 : Keyboard teleop
    2 : Follow way points
    3 : Create way points
    9 : Simple move with /cmd_vel

    CTRL-C to quit
    """    
    
    while not rospy.is_shutdown():
        print(msg)        
        mode = raw_input("Please select a mode\n")
        print(mode)
        stop_highlight = False
        background_thread = None
        active_thread = None
        stop_threads = False        
        if mode == "":            
            stop_threads = True
            if active_thread:
                active_thread.join()
        elif int(mode) == 3:
            # app.mode_waypoint_create()
            active_thread = threading.Thread(target=app.mode_waypoint_create, args=(lambda : stop_threads, ))
            active_thread.start()
        elif int(mode) == 2:
            active_thread = threading.Thread(target=app.start_patrolling, args=(lambda : stop_threads, ))
            active_thread.start()
        elif int(mode) == 1:
            active_thread = threading.Thread(target=app.basic_move)
            active_thread.start()        
        elif int(mode) == 0:
            background_thread = threading.Thread(target=app.publish_markers, args=(lambda : stop_highlight, ))
            background_thread.start()
            # stop_highlight = True

main()