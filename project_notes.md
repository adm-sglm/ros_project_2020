## Task 1

1.- The /cmd_vel topic is the topic used to move the robot. Do a rostopic info /cmd_vel in order to get information about this topic, and identify the message it uses. You have to modify the code to use that message.

2.- In order to fill the Twist message, you need to create an instance of the message. In Python, this is done like this: var = Twist()

3.- In order to know the structure of the Twist messages, you need to use the rosmsg show command, with the type of the message used by the topic /cmd_vel.

4.- In this case, the robot uses a differential drive plugin to move. That is, the robot can only move linearly in the x** axis, or rotationaly in the angular **z axis. This means that the only values that you need to fill in the Twist message are the linear x** and the angular **z.

5.- The magnitudes of the Twist message are in m/s, so it is recommended to use values between 0 and 1. For example, 0'5 m/s.

## Difference between rospy.spin() and rospy.sleep()

>rospy.spin() will effectively go into an infinite loop until it receives a shutdown signal (e.g. ctrl-c). During that loop it will process any events that occur, such as data received on a topic or a timer triggering, when they occur but otherwise it will sleep. You should use spin() when your node doesn't do anything outside of its callbacks.

> rospy.sleep() will do the same as rospy.spin() but only for the length of time you specify. It is used when your node has to do some kind of regular processing in addition to responding to topic events, timer events, etc. For example, a node controlling a motor will need to do some processing at regular intervals to set the motor's desired speed.

> (Technically, what both are doing is sleeping your main thread and allowing other, internal threads to do their work.)

Maybe use a different teleop like this one which only moves as long as you keep holding the button

roslaunch turtlebot_teleop keyboard_teleop.launch


## Task 2

You've seen that you need a Map in order to Navigate **autonomously** with your robot



### Mapping the environment

Create a launch file which runs bringup with remote and sets the **slam_gmapping** parameters

```
<launch>
  <!-- Turtlebot3 -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch" />

  <!-- Gmapping -->
  <node pkg="gmapping" type="slam_gmapping" name="turtlebot3_slam_gmapping" output="screen">
    <param name="base_frame" value="base_footprint"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_update_interval" value="2.0"/>
    <param name="maxUrange" value="4.0"/>
    <param name="minimumScore" value="100"/>
    <param name="linearUpdate" value="0.2"/>
    <param name="angularUpdate" value="0.2"/>
    <param name="temporalUpdate" value="0.5"/>
    <param name="delta" value="0.05"/>
    <param name="lskip" value="0"/>
    <param name="particles" value="120"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="xmin" value="-10.0"/>
    <param name="ymin" value="-10.0"/>
    <param name="xmax" value="10.0"/>
    <param name="ymax" value="10.0"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>

    </node>
</launch>
```

In one terminal I launch `start_launch.launch` file to bring up the ROS core.

In another terminal I launch rviz with the modified code of my own.

In a third terminal I launch `turtlebot3_teleop turtlebot3_teleop_key.launch` for moving the robot around to save the map.

After observing all the map I save it using `rosrun map_server map_saver -f name_of_map`


## Explaining the initial pose

https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation

## Navigation

![](report/assets/overview_tf.png)

According to the shown diagram, we must provide some functional blocks in order to work and communicate with the Navigation stack. Following are brief explanations of all the blocks which need to be provided as input to the ROS Navigation stack:

  Odometry source: Odometry data of a robot gives the robot position with respect to its starting position. Main odometry sources are wheel encoders, IMU, and 2D/3D cameras (visual odometry). The odom value should publish to the Navigation stack, which has a message type of nav_msgs/ Odometry. The odom message can hold the position and the velocity of the robot.
  Sensor source: Sensors are used for two tasks in navigation: one for localizing the robot in the map (using for example the laser) and the other one to detect obstacles in the path of the robot (using the laser, sonars or point clouds).
  sensor transforms/tf: the data captured by the different robot sensors must be referenced to a common frame of reference (usually the base_link) in order to be able to compare data coming from different sensors. The robot should publish the relationship between the main robot coordinate frame and the different sensors' frames using ROS transforms.
  base_controller: The main function of the base controller is to convert the output of the Navigation stack, which is a Twist (geometry_msgs/Twist) message, into corresponding motor velocities for the robot.

**For Kobuki example**

> /cmd_vel: Receives the output of the Navigation Stack and transforms the commands into motor velocities.

> /kobuki/laser/scan: Provides the Laser readings to the Stack.

> /odom: Provides the Odometry readings to the Stack.

> /tf: Provides the Transformations to the Stack.

**How move_base functions**

The main function of the move_base node is to move a robot from its current position to a goal position with the help of other Navigation nodes. This node links the global planner and the local planner for the path planning, connecting to the rotate recovery package if the robot is stuck in some obstacle, and connecting global costmap and local costmap for getting the map of obstacles of the environment.

Following is the list of all the packages which are linked by the move_base node:

    global-planner
    local-planner
    rotate-recovery
    clear-costmap-recovery
    costmap-2D

Following are the other packages which are interfaced to the move_base node:

    map-server
    AMCL
    gmapping

## Localization

You should see on the Rviz window the map of the cafeteria, a representation of the Kobuki on that map, and a lot of green arrows. Those green arrows represent location guesses of the robot in the map. That is, the green arrows are guesses that the localization algorithm is doing in order to figure out where in the map the robot is located. The arrows will concentrate on the most likely location when you move the robot. Let's try this.

## Move base

`roslaunch turtlebot_navigation_gazebo move_base_demo.launch`

Working launch file from Ros Navigation in 5 days course.

## SLAM mapping

https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node

## Extras

Spawning a model look for description on course
_"ROS NAVIGATION IN 5 DAYS"_

**Copying the model (a box) to our workspace**

```bash
cp /home/simulations/public_sim_ws/src/all/turtlebot/turtlebot_navigation_gazebo/urdf/object.urdf /home/user/catkin_ws/src
```
**Spawning the model into map**
```bash
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/object.urdf -urdf -x 0 -y 0 -z 1 -model my_object
```

***Removing the model from map**

```bash
rosservice call /gazebo/delete_model "model_name: 'my_object'"
```

After each action of the video I can explain what is happening here by sharing topics and their data.
# Moving between waypoints

```bash
header:
  seq: 0
  stamp:
    secs: 22886
    nsecs: 834000000
  frame_id: ''
status:
  goal_id:
    stamp:
      secs: 22866
      nsecs: 499000000
    id: "/move_base-1-22866.499000000"
  status: 2
  text: "This goal was canceled because another goal was recieved by the simple action server"
result:

---
header:
  seq: 1
  stamp:
    secs: 22909
    nsecs: 167000000
  frame_id: ''
status:
  goal_id:
    stamp:
      secs: 22886
      nsecs: 586000000
    id: "/move_base-2-22886.586000000"
  status: 3
  text: "Goal reached."
result:

---
rostopic echo /move_base/result
```

`source /opt/ros/melodic/setup.bash`
