#!/usr/bin/env bash

# Setup the ROS workspace.
source ~/demo_ws/devel/setup.bash

# Move the the demo package.
roscd demo133

# Start the roscore
roscore &
sleep 1

# Load the URDF into the description parameter.
rosparam set /robot_description -t urdf/fourR-gazebo.urdf

# Launch the Gazebo server with an empty world.
#roslaunch gazebo_ros empty_world.launch &
rosparam set /use_sim_time true
rosrun gazebo_ros gzserver __name:=gazebo &
sleep 1

# Instantiate the robot.
rosrun gazebo_ros spawn_model -urdf -model fourR -param robot_description -J theta2 1.57

# Run the client
# rosrun gazebo_ros gzclient __name:=gazebo_gui &
gzclient

# This implicitly waits until the client window is closed.

# # Kill everything.
kill -9 %2
kill -9 %1

killall -9 roscore
killall -9 rosmaster
killall -9 rosout
killall -9 gzserver
killall -9 gzclient
