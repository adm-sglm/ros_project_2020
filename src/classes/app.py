import actionlib

from geometry_msgs.msg import Twist, PointStamped, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class App:
    rospy = None
    points = []
    client = None
    rate = None
    robot_pose = None
    frame_id = None
    amcl_sub = None
    def __init__(self, interface):
        self.rospy = interface
        self.rate = self.rospy.Rate(2)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.amcl_sub = self.rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl_pose)

    def cb_amcl_pose(self, data):
        self.robot_pose = data
        self.amcl_sub.unregister()
    
    def cb_point_clicked(self, data):
        # data.point carries the point clicked
        point_msg = """
        Point added
        X: {x}
        Y: {y}
        Z: {z}

        Press enter if you are done
        """
        print(point_msg.format(x=data.point.x,y=data.point.y,z=data.point.z))        
        self.points.append(data.point)
        # mode_waypoint_create(waypoint_count+1)

    def mode_waypoint_create(self, stop):
        sub = self.rospy.Subscriber("clicked_point", PointStamped, self.cb_point_clicked)
        print("Select a point using Publish Point button on Rviz to save the waypoint\n")
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

    def start_patrolling(self, stop):        
        for wp in self.points:
            print(wp)
            goal = MoveBaseGoal()
            pose = Pose()
            pose.position = wp
            pose.orientation = self.robot_pose.pose.pose.orientation
            # goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose = pose
            self.client.send_goal(goal)
            self.client.wait_for_result()