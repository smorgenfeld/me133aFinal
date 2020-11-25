#!/usr/bin/env python3
#
#   controller.py
#
#   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy, math, time, numpy as np

from sensor_msgs.msg import JointState
from kinematics import Kinematics


#
#  Main Code
#

def ikinVel(x, xp, rot, t0, robot, rnd, N, dt):

    q = np.zeros((N,1))
    qdot = np.zeros((N,1))
    p = np.zeros((3,1))
    R = np.identity(3)
    J = np.zeros((6,N))

    robot.Jac(t0, J)

    robot.fkin(t0, p, R)
    er1 = np.matrix(x).reshape(3,1)-p;
    er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
    er2 = er2.reshape((3,1))

    er = np.concatenate([er1,er2]);
    goalVel = np.concatenate([np.matrix(xp).reshape(3,1),[[0],[0],[0]]])

    qdot = np.linalg.pinv(J) * (goalVel + 1/(10 * dt) * er)
   
    t0 = t0 + dt * qdot

    theta = t0;
    
    if rnd:
        for i in range(int(N)):
            theta[i]=theta[i]%(2*np.pi);
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


def doPhysics(topPole, botPole, q, pos, N, dt):
    ang = np.zeros((2,1));
    m = 1;
    k = 500;
    b = 2;
    lgoal = 1;
    g = 9.81;
    G = np.array([0,0,-g*m]).reshape((3,1))

    # Get forward kinematics for top and bottom ends of the pole
    topRot = np.identity(3)
    topPos = np.zeros((3,1))
    topPole.fkin(q, topPos, topRot);

    botRot = np.identity(3)
    botPos = np.zeros((3,1))
    botPole.fkin(q, botPos, botRot);


    # Find real length of pole (for use with spring)
    lReal = np.linalg.norm(np.matrix(pos[0:3]).reshape((3,1))-botPos)

    # Tension (from 'pole' as very stiff spring)
    tVec = (np.matrix(pos[0:3]).reshape((3,1))-botPos) / lReal;
    T = -k * (lReal - lgoal)* tVec #- (pos[3] * tVec[0,0] + pos[4] * tVec[1,0]+pos[5] * tVec[2,0]) * b;
    #print(np.linalg.norm(np.matrix(pos[0:3]).reshape((3,1)) - topPos))
    #print(abs(lReal - lgoal))

    # Calculate acceleration (on simulated spring mass)
    A = (T + G) / m;
    #print(A)

    # Iterate to next point
    pos[3] += A[0,0] * dt;
    pos[4] += A[1,0] * dt;
    pos[5] += A[2,0] * dt;
    pos[0] += pos[3] * dt;
    pos[1] += pos[4] * dt;
    pos[2] += pos[5] * dt;

    # Fake point for robot to move to
    fpos = botPos + lgoal * (np.matrix(pos[0:3]).reshape((3,1)) - botPos) / np.linalg.norm(lgoal * (np.matrix(pos[0:3]).reshape((3,1)) - botPos));


    ang = ikinPole(fpos, q, 0.01, topPole, True, N)

    # Get goal angle for next timestep

    return (ang, pos)

def ikinPole(x, t0, error, robot, rnd, N):
    original = t0.copy();

    for i in range(1050):
        q = np.zeros((N,1))
        p = np.zeros((3,1))
        R = np.identity(3)
        J = np.zeros((6,N))
        robot.Jac(t0, J)
        robot.fkin(t0, p, R)
        er = np.matrix(x).reshape((3,1))-p;

        t0 = t0 + np.linalg.pinv(J[0:3]) * er;
        # cheese the ikin :D
        t0[0:7] = original[0:7]

        # Exit if error is small
        if np.linalg.norm(er) < error:
            break;

    theta = t0;
    
    # Clamp angle within 2pi (having large angles gives me anxiety)
    if rnd:
        for i in range(N):
            theta[i]=theta[i]%(2 * np.pi);

    return theta[7:]

def ikinBot(x, rot, t0, error, robot, rnd, N):
    for i in range(150):
        q = np.zeros((N,1))
        p = np.zeros((3,1))
        R = np.identity(3)
        J = np.zeros((6,N))
        robot.Jac(t0, J)
        JFixed = np.concatenate([J[3:6][3:6],J[0:3][0:3]])
        robot.fkin(t0, p, R)
        er1 = np.matrix(x).reshape(3,1)-p;
        er2 = 0.5 * (np.cross(R[:,0].reshape((1,3)), rot[:,0].reshape((1,3))) + np.cross(R[:,1].reshape((1,3)) , rot[:,1].reshape((1,3))) + np.cross(R[:,2].reshape((1,3)), rot[:,2].reshape((1,3))));
        er2 = er2.reshape((3,1))

        er = np.concatenate([er1,er2]);
        print(J)
        t0 = t0 + np.linalg.pinv(J) * er;
        if np.linalg.norm(er) < error:
            break;
    theta = t0;

    # Clamp angle within 2pi (having large angles gives me anxiety)
    if rnd:
        for i in range(N):
            theta[i]=theta[i]%(2 * np.pi);
    return theta


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
    swingDemo = True;

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    t   = 0.0
    tf  = 2 * math.pi;
    urdf = rospy.get_param('/robot_description');
    tipKin = Kinematics(urdf, 'world', 'tip');
    topKin = Kinematics(urdf, 'world', 'end');

    N   = topKin.dofs()
    R = np.matrix([[1,0,0],[0,1,0],[0,0,1]]);

    q = np.matrix([0.1,0.1,0.1,0.1,0.1,0.1,0.1,np.pi,0.1]).reshape((N,1));
    #q1 = ikinBot([0,0.25,0.6], R1, q[0:7], 0.01, tipKin, True, N-2)
    #q[0:7] = q1
    

    # Set cartesian coords for simulated mass
    pos = [0,0,1.6,0,0,0]
    if swingDemo:
        pos = [0,0,-0.6,0,0,0]

    #topRot = np.identity(3)
    #topPos = np.zeros((3,1))
    #topKin.fkin(q, topPos, topRot);
    #pos[0] = topPos[0,0]
    #pos[1] = topPos[1,0]
    #pos[2] = topPos[2,0]
    
    while not rospy.is_shutdown():

        # Move to a new time step, assuming a constant step
        t = t + dt

        # Do pole physics
        q89, pos = doPhysics(topKin, tipKin, q, pos, N, dt)
        q[7] = q89[0,0];
        q[8] = q89[1,0];

        # Try? to catch the pole (DOES NOT WORK!!!)
        a = 2;
        b = 10;
        x = [pos[0] + a * pos[3],pos[1] + a * pos[4],0.6]
        xp = [b * pos[3],b * pos[4],0]

        # Demo the pendulum physics (THIS DOES WORK!!!)
        if swingDemo:
            x = [np.sin(4*t)/4,np.cos(4*t)/4,0.6]
            xp = [np.cos(4*t),-np.sin(4*t),0]

        # Using velocity ikin to move rest of robot
        q1 = ikinVel(x, xp, R, q[0:7], tipKin, False, N-2, dt)
        q[0:7] = q1
        
        # Send the command (with the current time) and sleep.
        msg.position = q;
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        servo.sleep()

        # Break if we have completed the full time.
        if (t > tf):
            if repeating:
                t = 0
            else:
                break;
