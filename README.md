<center>
    <h1>Robotics Project</h1>
    <strong><h2>Technical Documentation</h2></strong>
    <br />
    <h3>Adem SAGLAM - Syed Muhammad Hashaam SAEED</h3>
    <br />
    <h2>Under Supervision of</h2>
    <h3><strong>Ralph SEULIN, Raphael DUVERNE and Daniel BRAUN</strong></h3>

![](report/assets/vibot.png)

### VIBOT M2 - 2020
</center>

---

## Introduction

Our robotics project is built on top of **ROS** short for Robot Operating System. An open source framework which provides many libraries and tools to simplify complex robotic behaviour. Ros even though the name contains operating system, it is a meta operating system made to create a thin layer between hardware and software while achieving;

- Platform independence (Easily integratable with other frameworks)
- Ros-Agnostic Libraries (Libraries less bound to ROS)
- Language independence (Roscpp, Rospy...)
- Testing
- Scaling

### Concepts of ROS

ROS seperates its concepts into three levels Filesystem level, Computation Graph level and Community level. From these three levels several resources from **Filesystem level** and **Computation Graph level** are very important for our project. First let us summarize the resources;

#### **File System Level**

This level contains files on storage device.

Packages, metapackages, package manifests, repositories, message types and service types.

#### **Computation Graph Level**

On this level the resources are computational.

Master, nodes, topics, messages, services, parameter server and bags.

#### **Community Level**

This level contains community additions to the project most important one is the documentation.

## Technical Analysis

Within the availability of concepts; we used several of them extensively. Here is the resources we used with explanations;

#### **Master**
is the bridge of the computational level it register [nodes](#node) and [services](#service), exchanges [messages](#message) between different nodes by tracking [subscribers](#subscribe) and [publishers](#publish).

#### **Node**
is a process unit generally built to occupe with one concern like navigation. Nodes register to master and using the [topics](#topic) and services they communicate between other nodes. Nodes have a unique name and they form a graph when combined.

<center>
  <figure>
    <img src="./report/assets/rosgraph.svg">
    <figcaption>A graph showing the nodes and their subscriptions.</figcaption>
  </figure>
</center>

#### **Topic**
can be considered like a pipeline strictly connecting two endpoints, here endpoints are nodes. All the topic's are bound to a **message type** from [File System Level](#file-system-level).

<center>
  <figure>
    <img src="./report/assets/topic.png">
    <figcaption>A graph showing how topics connect nodes</figcaption>
  </figure>
</center>

#### **Message**

Communication between nodes are done by transmitting messages through topics or services. Messages can be primitive or structural and their type should be defined in [File System Level](#file-system-level), in order for to be bound by topics and services.

#### **Service**





Sometimes a graphic worths a thousand words:

A node registered to Master

The node subscribes a topic with a message type

Another node registers

Publishes to that topic

Emphasize many to many (
  a node gets a message through a topic that it subscribed
  a side effect occurs and maybe it publishes that in another topic but not in a request response fashion
)

Show service

Request and response

in Implementation

Launch file / Initializing master

basically a master is initialized by launching `roscore` but here we dont directly run it but let it run by a launch file provided by turtlebot3 package which also sets up some parameters

show i used service if not how would it look

Why i created my teleops command and demonstrated basic cmd_vel commands (with the delay on platform the supported teleops on turtlebot3 package wasn't suitable and making us miss spots)

Pathfinding

Show it actively working by placing an obstacle and emphasize the dynamic map and static map file OccupancyGrid.


## Launching the project

Our project has an entry script file which a user can give commands to achieve the tasks and a customized launch file that sets up the required nodes and configuration parameters in place for the script to work properly. Since some functionality of the application required us to select points from the map we also use Rviz.

**First we launch the base**

Using a terminal;

`roslaunch ros_project_2020 move_base.launch`

**Second we launch the application**

From another terminal

`rosrun ros_project_2020 main.py`

**Launching the Rviz for observing**

In order to observe the robot

`rosrun rviz rviz -d ~/catkin_ws/src/ros_project_2020/turtlebot3_slam.rviz`


http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation

## References

- http://wiki.ros.org/ROS/Introduction
- http://wiki.ros.org/ROS/Concepts