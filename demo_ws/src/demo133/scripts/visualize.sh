#!/usr/bin/env bash

# Setup the ROS workspace.
source ~/demo_ws/devel/setup.bash

# Move the the demo package.
roscd demo133

# Start the roscore
roscore &
sleep 1

# Load the URDF into the description parameter.
rosparam set /robot_description -t urdf/fourR.urdf

# Start a GUI with sliders to command the joints.
rosrun joint_state_publisher_gui joint_state_publisher_gui &

# Start the ROS kinematic chain processing.
rosrun robot_state_publisher robot_state_publisher _ignore_timestamp:=true &

# Start the visualizer
rosrun rviz rviz -d rviz/viewfourR.rviz

# The above implicitly waits until the visualizer is closed...

# Kill everything.
kill -9 %3
kill -9 %2
kill -9 %1

killall -9 roscore
killall -9 rosmaster
killall -9 rosout
