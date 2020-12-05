#!/usr/bin/env python3
#
#   controller.py
#
#   Publish:   /joint_states      sensor_msgs/JointState
#
import rospy, math, time, random, numpy as np

from sensor_msgs.msg import JointState
from kinematics import Kinematics


#
#  Main Code
#

def ikinVel(x, xp, rot, t0, robot, rnd, N, dt):
    # Calculate angles for each joint, aiming for a goal position and velocity.
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



def doPhysics(topPole, botPole, q, pos, N, dt,p):
    ang = np.zeros((2,1));
    m = 1;
    k = 1000;
    b = math.pow(k,0.5);
    lgoal = 1;
    g = 9.81;
    G = np.array([p[0],p[1],p[2]-g*m]).reshape((3,1))

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
    T = (-k * (lReal - lgoal) - (pos[3] * tVec[0,0] + pos[4] * tVec[1,0]+pos[5] * tVec[2,0]) * b) * tVec;
    #print(lReal - lgoal)

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

    # Get goal angle for next timestep
    ang = ikinPole(fpos, q, 0.01, topPole, True, N)

    return (ang, pos)


def ikinPole(x, t0, error, robot, rnd, N):
    # Pole position ikin
    original = t0.copy();

    for i in range(10):
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

def clamp(n, s, l): return max(s, min(n, l));

def getStoppingDist(v, a, x): return v*x-a/2*x**2;

def getQuintic(x0,v0,a0,x1,v1,a1,dt):
    # Scale time to x1 to constrain max acceleration (doesn't work)
    #t = 2.4 * (abs(x1-x0)/maxAccel)**0.5;
    t = dt * 25

    # Calculate coefficients of quintic function
    a = (-t*(a0*t-a1*t+6*(v0+v1))+12*(x1-x0))/(2*t**5)
    b = (t*(3*a0*t-3*a1*t+16*v0+14*v1)+30*(x0-x1))/(2*t**4)
    c = (t*(-3*a0*t+a1*t-12*v0-8*v1)+20*(x1-x0))/(2*t**3)
    d = a0/2
    e = v0
    f = x0

    # Calculate position/velocity/acceleration at next time step
    # These will be fed back into this function next time step to continue 
    # the quintic motion
    x01 = a*dt**5  +  b*dt**4+    c*dt**3+   d*dt**2+ e*dt+ f;
    v01 = 5*a*dt**4+  4*b*dt**3+  3*c*dt**2+ 2*d*dt+  e;
    a01 = 20*a*dt**3+ 12*b*dt**2+ 6*c*dt+    2*d;

    return [x01, v01, a01];

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

    # Time constants
    t   = 0.0
    tf  = 2 * math.pi;

    # Set repeating to false to run sim for length tf
    repeating = True;
    # Set swingDemo to True for pendulum physics demos
    swingDemo = False;

    # Prepare a servo loop at 1000Hz.
    rate  = 1000;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))

    # Assign kinematics constants
    urdf = rospy.get_param('/robot_description');
    tipKin = Kinematics(urdf, 'world', 'tip');
    topKin = Kinematics(urdf, 'world', 'end');
    N = topKin.dofs()
    R = np.matrix([[1,0,0],[0,1,0],[0,0,1]]);

    # Set initial joint angle guess
    q = np.matrix([0.1,0.1,0.1,0.1,0.1,0.1,0.1,np.pi,0.1]).reshape((N,1));

    # Set cartesian coords for simulated mass
    pos = [0,0.1,1.6,0,0,0]
    if swingDemo:
        pos = [0,0,-0.6,0,0,0]

    # Pole perturbation constants
    pTimer = 0;
    perturbFreq = 5000;    # Occurs every perturbFreq time steps
    perturbLength = 50;    # Number of timesteps that perturbMag force is applied
    perturbAng = 0;        # Placeholder for random force angle
    perturbMag = 25;       # Magnitude of force

    # Tip accel constraint constants
    x = [0,0];
    xp = [0,0];
    goalX = [0,0];
    curV = [0,0];
    curA = [0,0];
    armRange = 0.75;       # Robot's reach (any further and it will hit a singularity)
    tipRot = np.identity(3)
    tipPos = np.zeros((3,1))

    # Constant to scale overshoot bias
    a = 0.1
    # Constant to scale centering bias
    b = 3

    # Sleep a bit so rviz starts before robot starts moving/failing
    time.sleep(3)

    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    while not rospy.is_shutdown():

        # Move to a new time step, assuming a constant step
        t = t + dt

        # Perturb the top of the pole in a random direction
        pTimer += 1
        p = [0,0,0]
        if pTimer >= perturbFreq:
            p = [perturbMag * np.sin(perturbAng),perturbMag * np.cos(perturbAng),0]
            if (pTimer > perturbFreq + perturbLength):
                pTimer = 0 # Reset perturb timer
            elif (pTimer == perturbFreq):
                perturbAng = random.random() * 2 * np.pi;
                print("Perturbed in" + str(p))

        # Do pole physics
        q89, pos = doPhysics(topKin, tipKin, q, pos, N, dt, p)
        q[7] = q89[0,0];
        q[8] = q89[1,0];

        # Try? to catch the pole (WORKS!!!)

        # Get the current position of the robot tip
        tipPos = np.zeros((3,1))
        tipKin.fkin(q, tipPos, tipRot)
        tipPos = [tipPos[0,0],tipPos[1,0],tipPos[2,0]]

        # Figure out where we want the tip to be, in order to balance the pendulum and bring it back towards the center
        goalX = [pos[0] + a * pos[3] + b*(pos[0] / armRange)**3,pos[1] + a * pos[4]+ b*(pos[1] / armRange)**3]
        # Constrain goal position within circular workspace of radius armRange
        goalX = [min(1,armRange / np.linalg.norm(goalX)) * goalX[0],min(1,armRange / np.linalg.norm(goalX)) * goalX[1]]

        # Get the next position/velocity/acceleration using a quintic function
        Xpos = getQuintic(tipPos[0],curV[0],curA[0],goalX[0],0,0,dt)
        Ypos = getQuintic(tipPos[1],curV[1],curA[1],goalX[1],0,0,dt)

        # Save new position, velocity and acceleration (scaling position to stay within circular workspace)
        goalPos = [min(1,armRange / np.linalg.norm([Xpos[0],Ypos[0]])) * Xpos[0],min(1,armRange / np.linalg.norm([Xpos[0],Ypos[0]])) * Ypos[0]]
        Xpos[0] = goalPos[0]
        Ypos[0] = goalPos[1]
        curV = [Xpos[1], Ypos[1]]
        curA = [Xpos[2], Ypos[2]]

        # Assign new position and velocity goals for ikin function
        x = [goalPos[0],goalPos[1],1.0]
        xp = [curV[0],curV[1],0];

        # Demo the pendulum physics
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
        # Doesn't do anything if repeating is set to false.
        if (t > tf):
            if repeating:
                t = 0
            else:
                break;
