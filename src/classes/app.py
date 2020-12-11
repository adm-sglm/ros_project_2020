import actionlib

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class App:
    rospy = None
    points = []
    client = None
    def __init__(self, interface):
        self.rospy = interface
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

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
    
    def basic_move(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pub = self.rospy.Publisher("cmd_vel", Twist, queue_size=10)
        print("publish")
        pub.publish(msg)

    def start_patrolling(self):        
        print(self.points)