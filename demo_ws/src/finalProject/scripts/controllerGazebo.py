#!/usr/bin/env python3
#
#   controller.py
#
#   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy, math, numpy as np

from sensor_msgs.msg import JointState
from kinematics import Kinematics


#
#  Main Code
#

def ikinVel(x, xp, rot, t0, robot, wrist, rnd, N, dt, c, partC):

    q = np.zeros((N,1))
    qdot = np.zeros((N,1))
    p = np.zeros((3,1))
    pEl = np.zeros((3,1))
    R = np.identity(3)
    rEl = np.identity(3)
    J = np.zeros((6,N))
    J4 = np.zeros((6,N))

    robot.Jac(t0, J)
    wrist.Jac(t0, J4)
    wrist.fkin(t0, pEl, rEl)

    robot.fkin(t0, p, R)
    er1 = x-p;
    er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
    er2 = er2.reshape((3,1))

    r = 0.83
    pLimit = np.matrix([[0],[0],[0.6]])
    wristVel = -c * (pEl-pLimit) / abs(np.linalg.norm(pEl-pLimit) - r);

    er3 = np.zeros((3,1))

    er = np.concatenate([er1,er2, er3]);

    Jtot = np.concatenate([J,J4[0:3,:]]);

    goalVel = np.concatenate([xp,[[0],[0],[0]],wristVel])

    qdot = np.linalg.pinv(Jtot) * (goalVel + 1/(10 * dt) * er)
    t0 = t0 + dt * qdot

    if partC:
        # Implement elbow secondary task
        tgoal = t0.copy();
        goalAng = 0.05
        if 0 < t0[3] < goalAng:
            tgoal[3] = goalAng
        if math.pi * 2 > t0[3] > math.pi * 2-goalAng:
            tgoal[3] = math.pi * 2-goalAng
        qdot += (1 - np.linalg.pinv(Jtot) * np.matrix(Jtot)) * (1/(10 * dt)) * (tgoal - t0)

    #print(qdot)
    t0 = t0 + dt * qdot;


    theta = t0;
    
    if rnd:
        for i in range(int(N)):
            theta[i]=theta[i]%(2*math.pi);
    return theta

def bound(arr, ind, lbound, hbound, changeAmount):
    changed = 0;
    if arr[ind] <= lbound:
        arr[ind] += changeAmount * min(2,abs(arr[ind] - lbound));
        changed = 1;
    elif arr[ind] >= hbound:
        arr[ind] -= changeAmount * min(2,abs(arr[ind] - hbound));
        changed = 1;
    return changed;

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

def doPhysics(topPole, botPole, q, qdot, adot, N, dt):
    ang = np.zeros((2,1));
    m = 1;
    g = .981;
    pDot = np.array([0,0,-g])

    # Get forward kinematics for top and bottom ends of the pole
    topRot = np.identity(3)
    topPos = np.zeros((3,1))
    topPole.fkin(q, topPos, topRot);

    botRot = np.identity(3)
    botPos = np.zeros((3,1))
    botPole.fkin(q, botPos, botRot);


    # Center of mass position (not assuming point mass pendulum)
    CoM = np.add(botPos,topPos) / 2;
    # Local CoM (to tip)
    CoMl = CoM - botPos
    l = np.linalg.norm(CoMl)

    # Get center of mass in spherical coords
    latitude = np.arccos(CoMl[2] / np.linalg.norm(CoMl));
    longitude = np.arctan(CoMl[1] / CoMl[0]) + np.pi / 2;

    goalLongitude = longitude;
    
    # Effect of gravity
    #print(np.sin(latitude))
    adotdot = (g) / l * np.sin(latitude);
    #print(adotdot)
    adot += adotdot * dt;


    goalLatitude = latitude + adot;


    if goalLatitude < 0:
        goalLatitude = - goalLatitude;

    elif goalLatitude > 2*np.pi:
        goalLatitude = 2*np.pi- goalLatitude;


    #goalLongitude = goalLongitude % 2*np.pi

    ang = np.array([0, goalLatitude]);
    #print(goalLongitude)

    # Get goal angle for next timestep

    return (ang, adot)



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
    msg.name.append('theta7')
    msg.name.append('theta8')
    msg.name.append('theta9')

    repeating = True;

    # Prepare a servo loop at 100Hz.
    rate  = 10;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    t   = 0.0
    tf  = 2 * math.pi;
    urdf = rospy.get_param('/robot_description');
    tipKin = Kinematics(urdf, 'world', 'tip');
    wristKin = Kinematics(urdf, 'world', 'wrist');
    topKin = Kinematics(urdf, 'world', 'end');

    N   = topKin.dofs()

    q = np.matrix([0,0,0,0,0,0,0]).reshape((N,1));
    q0 = q.copy();
    adot = 0;
    x = np.matrix([0,0.6,0.5]).reshape((3,1));
    R1 = np.matrix([[-1,0,0],[0,1,0],[0,0,-1]]);
    R2 = np.matrix([[-1,0,0],[0,0,1],[0,1,0]]);
    P1 = np.matrix([[0.3],[0.5],[0.05]]);
    P2 = np.matrix([[-0.3],[0.5],[0.05]]);


    
    while not rospy.is_shutdown():

        # Move to a new time step, assuming a constant step!
        t = t + dt

        # Calculate current qdot using previous q (q0)
        qdot = np.subtract(q, q0) * dt;
        # Save current matrix to be used to calculate qdot next timestep
        q0 = q.copy();

        # Do pole physics
        #q89, adot = doPhysics(topKin, tipKin, q, qdot, adot, N, dt)
        #q[7] = q89[0];
        #q[8] = q89[1];

        # Set the positions as a function of time.

        msg.position = q;
        # Send the command (with the current time) and sleep.
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        servo.sleep()

        # Break if we have completed the full time.
        if (t > tf):
            if repeating:
                t = 0
            else:
                break;
