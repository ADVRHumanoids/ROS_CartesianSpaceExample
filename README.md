# ROS_CartesianSpaceExample

# Scope

This example allows you to perform a Cartesian Pose Task (see: https://github.com/ADVRHumanoids/Pose-Task).

It generates a thrird order trajectory for the position and orientation (using the Euler Angles).

# Start

Clone and build the pose project: there are several options to do it, but probably the easiest is using the catkin build tool from ROS.

```
mkdir -p ~/src/catkin_cspose_ws/src
cd ~/src/catkin_cspose_ws/src
catkin_init_workspace
git clone https://github.com/ADVRHumanoids/ROS_CartesianSpaceExample.git
cd ..
catkin_make
```

# Run

The selected robot was Centauro, whose basic config file can be set in this way:

```
set_xbot_config /opt/xbot/build/install/share/xbot/configs/CogimonConfig/cogimon_basic.yaml
```


As first thing you should run [roscore](http://wiki.ros.org/roscore):

```
roscore
```

Later on you can start the XBotCore in "dummy" mode in another terminal (more information [here](https://github.com/ADVRHumanoids/XBotControl/wiki/Quick-XBotCore-Start)) in this way:

```
XBotCore -D
```

In another terminal you can use the [RViZ](http://wiki.ros.org/rviz) tool to visualize the kinematic simulation of the robot by running:

```
rviz
```
and adding a _"RobotModel"_ with the following Robot Description: _xbotcore/robot_description_
Remind to select "world" for "Fixed Frame" in "Global Options".

This should be the visualization:


Start XBot Communication Plugin:

```
rosservice call /xbotcore/XBotCommunicationPlugin_switch 1
```

In another terminal launch:
``` 
. ~/src/catkin_cspose_ws/devel/setup.bash
```
Terminal 1: 
            
            rosrun pose poseCSp_talker   
