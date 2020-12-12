import actionlib

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class App:
    rospy = None
    points = []
    client = None
    rate = None
    def __init__(self, interface):
        self.rospy = interface
        self.rate = self.rospy.Rate(2)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def cb_point_clicked(self, data):
        # data.point carries the point clicked
        point_msg = """
        Point added
        X: {x}
        Y: {y}
        Z: {z}
        """
        print(point_msg.format(x=data.point.x,y=data.point.y,z=data.point.z))        
        self.points.append(data.point)
        # mode_waypoint_create(waypoint_count+1)

    def mode_waypoint_create(self, stop):
        sub = self.rospy.Subscriber("clicked_point", PointStamped, self.cb_point_clicked)
        print("Select a point using Publish Point button on Rviz or press enter if you are done\n")
        while not stop():
            self.rate.sleep()
        sub.unregister()
        msg = '{count} waypoint(s) saved\n'
        print(msg.format(count=len(self.points)))
    
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
        self.rospy.spin()

    def start_patrolling(self):        
        print(self.points)