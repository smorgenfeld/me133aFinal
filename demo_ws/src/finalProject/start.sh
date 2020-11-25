#!/usr/bin/env bash

# Setup the ROS workspace.
source ~/demo_ws/devel/setup.bash

# Move the the demo package.
roscd finalProject

# Start the roscore
roscore &
sleep 2

# Load the URDF into the description parameter.
rosparam set /robot_description -t urdf/sevenR.urdf

# Start a GUI with sliders to command the joints.
#rosrun joint_state_publisher_gui joint_state_publisher_gui &

# Start the ROS kinematic chain processing.
rosrun robot_state_publisher robot_state_publisher _ignore_timestamp:=true &

# Start python script
rosrun finalProject scripts/controller.py &

#rosbag record --duration 10s -O p5c.bag /joint_states &

# Start the visualizer
rosrun rviz rviz -d rviz/view133.rviz


# The above implicitly waits until the visualizer is closed...

# Gazebo
#rosparam set /use_sim_time true
#rosrun gazebo_ros gzserver __name:=gazebo &
#sleep 1

# Instantiate the robot.
#rosrun gazebo_ros spawn_model -urdf -model sevenR -param robot_description -J theta2 1.57

# Run the client
# rosrun gazebo_ros gzclient __name:=gazebo_gui &
#gzclient

# Kill everything.
kill -9 %3
kill -9 %2
kill -9 %1

killall -9 roscore
killall -9 rosmaster
killall -9 rosout