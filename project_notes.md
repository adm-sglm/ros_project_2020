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


## Task 2

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

