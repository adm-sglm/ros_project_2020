import actionlib

from geometry_msgs.msg import Twist, PointStamped, Pose, Point, Quaternion, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, MarkerArray , InteractiveMarkerControl
from std_msgs.msg import Header, ColorRGBA

class App:
    rospy = None
    points = []
    markers = MarkerArray()
    client = None
    rate = None
    robot_pose = None
    frame_id = None
    amcl_sub = None
    marker_pub = None
    def __init__(self, interface):
        self.rospy = interface
        self.rate = self.rospy.Rate(2)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.amcl_sub = self.rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl_pose)
        self.marker_pub = self.rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)        

    def publish_markers(self, stop):        
        while not stop():
            self.marker_pub.publish(self.markers)
            self.rate.sleep()

    def cb_amcl_pose(self, data):
        self.robot_pose = data
        self.amcl_sub.unregister()
    
    def cb_point_clicked(self, data):
        point_msg = """
        Point added
        X: {x}
        Y: {y}
        Z: {z}

        Press enter if you are done
        """
        print(point_msg.format(x=data.point.x,y=data.point.y,z=data.point.z))        
        self.points.append(data.point)

    def mode_waypoint_create(self, stop):
        sub = self.rospy.Subscriber("clicked_point", PointStamped, self.cb_point_clicked)
        print("Select a point using Publish Point button on Rviz to save the waypoint\n")
        while not stop():
            self.rate.sleep()
        sub.unregister()
        msg = '{count} waypoint(s) saved\n'
        self.create_markers()
        print(msg.format(count=len(self.points)))
    
    def create_markers(self):        
        for i, wp in enumerate(self.points):
            marker = Marker(
                id=i,
                type=Marker.TEXT_VIEW_FACING,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(wp.x, wp.y, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.0, 0.0, 1.5),
                header=Header(frame_id='map'),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text=str(i+1)
                )
            self.markers.markers.append(marker)  
    
    def basic_move(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pub = self.rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pub.publish(msg)
        self.rospy.spin()

    def start_patrolling(self, stop):        
        for wp in self.points:
            goal = MoveBaseGoal()
            pose = Pose()
            pose.position = wp
            pose.orientation = self.robot_pose.pose.pose.orientation
            goal.target_pose.header.frame_id = self.robot_pose.header.frame_id
            goal.target_pose.pose = pose
            self.client.send_goal(goal)
            self.client.wait_for_result()
        print("All points visited\n")