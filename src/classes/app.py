from geometry_msgs.msg import PointStamped

class App:
    rospy = None
    points = []
    def __init__(self, interface):
        self.rospy = interface    

    def cb_point_clicked(self, data):
        # data.point carries the point clicked    
        print(data.point)
        self.points.append(data.point)
        # mode_waypoint_create(waypoint_count+1)

    def mode_waypoint_create(self, waypoint_count = 0):
        self.rospy.Subscriber("clicked_point", PointStamped, self.cb_point_clicked)
        print("Select a point using Publish Point button on Rviz or press enter if you are done")
        # cmd = input("waiting input")
        # if cmd == "":
        #     print("you defined waypoints")
        self.rospy.spin()

    def start_patrolling(self):
        print(self.points)