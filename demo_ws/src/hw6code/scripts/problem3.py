#!/usr/bin/env python3
#
#   problem3.py
#
#   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy, math, numpy as np

from sensor_msgs.msg import JointState
from hw6code.kinematics import Kinematics


#
#  Main Code
#
def ikin(x, rot, t0, error, robot, round, N):
    for i in range(150):
        q = np.zeros((N,1))
        p = np.zeros((3,1))
        R = np.identity(3)
        J = np.zeros((6,N))
        robot.Jac(t0, J)
        JFixed = np.concatenate([J[3:6][3:6],J[0:3][0:3]])
        robot.fkin(t0, p, R)
        er1 = x-p;
        er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
        er2 = er2.reshape((3,1))

        er = np.concatenate([er1,er2]);
        t0 = t0 + np.linalg.inv(J) * er;
        if np.linalg.norm(er) < error:
            break;
    theta = t0;
    
    if round:
        for i in range(6):
            theta[i]=theta[i]%(2*3.14159);
    return theta


def Rx(theta):
    return np.matrix([[ 1, 0            , 0            ],
                     [ 0, np.cos(theta),-np.sin(theta)],
                     [ 0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
    return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                     [ 0            , 1, 0            ],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
    return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta), np.cos(theta) , 0 ],
                     [ 0            , 0             , 1 ]])

if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('motion')

    # Create a publisher to send the joint values (joint_states).
    # Note having a slightly larger queue prevents dropped messages!
    pub = rospy.Publisher("/joint_states", JointState, queue_size=100)

    # Wait until connected.  You don't have to wait, but the first
    # messages might go out before the connection and hence be lost.
    rospy.sleep(1)

    # Create a joint state message.  Each joint is explicitly named.
    msg = JointState()
    msg.name.append('theta1')    # Keep appending joint names
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
    t   = 0.0
    tf  = 3;
    lam = 0.1/dt
    urdf = rospy.get_param('/robot_description');
    kin = Kinematics(urdf, 'world', 'tip');
    N   = kin.dofs()

    t0 = np.matrix([2.9,2.6,1.2,1.6,2.3,1.2]).reshape((6,1));
    x = np.matrix([0,0.6,0.5]).reshape((3,1));
    R1 = np.matrix([[-1,0,0],[0,1,0],[0,0,-1]]);
    R2 = np.matrix([[-1,0,0],[0,0,1],[0,1,0]]);
    P1 = np.matrix([[0.3],[0.5],[0.05]]);
    P2 = np.matrix([[-0.3],[0.5],[0.05]]);
    q = ikin(x, R2, t0, 0.0001, kin, True, N)
    forward = True;
    while not rospy.is_shutdown():

        # Move to a new time step, assuming a constant step!
        t = t + dt
        # Smooth time input
        tt = (math.sin(3.141/3*t - 3.141/2)+1)*3/2
        a = -3.141 / 2 * tt/tf;
        b = 0.3 - 0.6 * tt/tf
        if (not forward):
            a = -3.141 / 2 * (tf - tt)/tf;
            b = 0.3 - 0.6 * (tf - tt)/tf
        R = R1 * Rx(a)
        x = np.matrix([[b],[0.5],[-55/9*b**2+3/5]])
        # Set the positions as a function of time.
        q = ikin(x, R, q, 0.0001, kin, False, N)
        msg.position = q;
        # Send the command (with the current time) and sleep.
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        servo.sleep()

        # Break if we have completed the full time.
        if (t > tf):
            t = 0
            if (forward):
                forward = not forward
            else:
                break
