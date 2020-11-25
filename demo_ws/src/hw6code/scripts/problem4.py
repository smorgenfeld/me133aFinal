#!/usr/bin/env python3
#
#   problem4.py
#
#   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy, math, numpy as np

from sensor_msgs.msg import JointState
from hw6code.kinematics import Kinematics


#
#  Main Code
#
def ikin(x, rot, t0, error, robot, round, N, partB, partC, partD):
    breakPoint = 0
    iterations = 100;
    for i in range(iterations):
        q = np.zeros((N,1))
        p = np.zeros((3,1))
        R = np.identity(3)
        J = np.zeros((6,N))
        robot.Jac(t0, J)
        robot.fkin(t0, p, R)
        er1 = x-p;
        er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
        er2 = er2.reshape((3,1))

        er = np.concatenate([er1,er2]);
        t0 = t0 + np.linalg.pinv(J) * er;

        bounded = 0

        if partB:
            t0[2] = -t0[6]
        elif partC:
            changeAmount = 0.001 * (iterations - i) / iterations;
            bounded += bound(t0,0,-math.pi,math.pi/2,changeAmount);
            bounded += bound(t0,1,-math.pi,math.pi/2,changeAmount);
            bounded += bound(t0,2,0,math.pi,changeAmount);
            bounded += bound(t0,3,-math.pi,0,changeAmount);
            bounded += bound(t0,4,-math.pi/2,math.pi/2,changeAmount);
            bounded += bound(t0,5,-math.pi/2,math.pi/2,changeAmount);
            bounded += bound(t0,6,-math.pi/2,math.pi/2,changeAmount);

        breakPoint = i
        if np.linalg.norm(er) < error:
            break;
    theta = t0;
    print(breakPoint)
    if bounded > 0:
        print(bounded)
    
    if round:
        for i in range(int(N)):
            theta[i]=theta[i]%(2*math.pi);
    return theta

def ikinVel(x, xp, rot, t0, robot, elbow, rnd, N, dt):

    q = np.zeros((N,1))
    qdot = np.zeros((N,1))
    p = np.zeros((3,1))
    pEl = np.zeros((3,1))
    R = np.identity(3)
    rEl = np.identity(3)
    J = np.zeros((6,N))
    J4 = np.zeros((6,N))
    c = 1

    robot.Jac(t0, J)
    elbow.Jac(t0, J4)
    elbow.fkin(t0, pEl, rEl)
    robot.fkin(t0, p, R)
    er1 = x-p;
    er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
    er2 = er2.reshape((3,1))

    pOb = np.matrix([[0],[0.5],[0.5]]);
    elVel = c * (pEl - pOb) / np.linalg.norm(pEl - pOb) ** 2;
    #print(elVel)
    er3 = np.zeros((3,1))

    er = np.concatenate([er1,er2, er3]);

    Jtot = np.concatenate([J,J4[0:3,:]]);

    goalVel = np.concatenate([xp,[[0],[0],[0]],elVel])

    #print(goalVel)
    #print(1/(10 * dt) * er)
    qdot = np.linalg.pinv(Jtot) * (goalVel + 1/(10 * dt) * er)
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

    # Question specific variables
    partB = False;
    partC = False;
    partD = True;
    repeating = False;

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
    kin2 = Kinematics(urdf, 'world', 'elbow');
    N   = kin.dofs()

    t0 = np.matrix([2.9,2.6,1.2,1.6,2.3,1.2,1]).reshape((N,1)); 
    x = np.matrix([0,0.6,0.5]).reshape((3,1));
    R1 = np.matrix([[-1,0,0],[0,1,0],[0,0,-1]]);
    R2 = np.matrix([[-1,0,0],[0,0,1],[0,1,0]]);
    P1 = np.matrix([[0.3],[0.5],[0.05]]);
    P2 = np.matrix([[-0.3],[0.5],[0.05]]);
    q = ikin(x, R2, t0, 0.0001, kin, True, N, partB, partC, partD)
    forward = True;
    
    while not rospy.is_shutdown():

        # Move to a new time step, assuming a constant step!
        t = t + dt
        ct = t;
        if not forward:
            ct = 3 - t;

        s = ((math.sin(math.pi/3*ct - math.pi/2)+1)*3/2) / tf
        sp = (math.pi / 2 * math.sin(math.pi * ct / 3)) / tf

        b = 0.3 - 0.6 * s

        R = R1 * Rx(-math.pi / 2 * s)
        x = np.matrix([[b],[0.5],[-55/9*b**2+3/5]])
        xp = np.matrix([[-0.2 * sp],[0],[-11/45*(2 * s - 3) * sp],])
        
        # Set the positions as a function of time.
        if partD:
            q = ikinVel(x, xp, R, q, kin, kin2, False, N, dt)
        else:
            q = ikin(x, R, q, 0.0001, kin, False, N, partB, partC, partD)
        msg.position = q;
        # Send the command (with the current time) and sleep.
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        servo.sleep()

        # Break if we have completed the full time.
        if (t > tf):
            t = 0
            if repeating:
                forward = not forward;
            else:
                if (forward):
                    forward = not forward
                else:
                    break
