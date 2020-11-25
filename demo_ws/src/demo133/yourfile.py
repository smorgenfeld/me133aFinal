#!/usr/bin/env python3
##   motion.py
##   Create a motion by continually sending joint values
##   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy
from sensor_msgs.msg   import JointState
import numpy, math
#
#  Main Code
#

# Adapted from https://stackoverflow.com/questions/2879441/how-to-interpolate-rotations
def slerp(p0, p1, Tcur, Tfin):
	# Cheat by scaling time such that movement is smooth (so that velocity is continuous)
    t = Tcur / Tfin;
    t = t * t * (3 - 2 * t)
    omega = numpy.arccos(numpy.dot(p0/numpy.linalg.norm(p0), p1/numpy.linalg.norm(p1)))
    so = math.sin(omega)
    return [math.sin((1-t)*omega) / so * p0[i] + math.sin(t*omega)/so * p1[i] for i in range(len(p0))]

if __name__ == "__main__":
	np.set_printoptions(suppress = True, precision = 6)
    # Prepare the node.
    rospy.init_node('motion')
    # Create a publisher to send the joint values (joint_states).
    pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    # Wait until connected.  You don't have to wait, but the first
    # messages might go out before the connection and hence be lost.
    rospy.sleep(0.25)
    # Create a joint state message.  Each joint is explicitly named.
    msg = JointState()
    msg.name.append('theta1')
    # Keep appending joint names
    msg.name.append('theta2')
    msg.name.append('theta3')
    msg.name.append('theta4')
    msg.name.append('theta5')
    msg.name.append('theta6')
    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
        (dt, rate))
    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    starttime = rospy.Time.now()
    forwards = True
    moveLength = 2
    while not rospy.is_shutdown():
        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()
        # Set the positions as a function of time.
        if forwards:
        	msg.position = slerp(vec1,vec2,t,moveLength)
       	else:
       		msg.position = slerp(vec2,vec1,t,moveLength)
       	if t > 2:
       		forwards = not forwards
       		starttime = rospy.Time.now()
        # Send the command (with time of current loop).
        msg.header.stamp = servotime
        pub.publish(msg)
        # Wait for the next turn.
        servo.sleep()
