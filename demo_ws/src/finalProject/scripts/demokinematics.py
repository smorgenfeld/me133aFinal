#!/usr/bin/env python3
#
#   demokinematics.py
#
#   Demostrate using the kinematics class to compute the fkin and
#   Jacobian, given a URDF.
#
#   Parameters: /robot_description      URDF
#
import rospy

import numpy as np
from hw6code.kinematics import Kinematics


#
#  Basic Rotation Matrices
#
def Rx(theta):
    return np.array([[ 1, 0            , 0            ],
                     [ 0, np.cos(theta),-np.sin(theta)],
                     [ 0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
    return np.array([[ np.cos(theta), 0, np.sin(theta)],
                     [ 0            , 1, 0            ],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta), np.cos(theta) , 0 ],
                     [ 0            , 0             , 1 ]])


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('testing')
    rospy.loginfo("Starting the test...")

    # Grab the URDF from the parameter server.
    urdf = rospy.get_param('/robot_description')
    #print(urdf)

    # Set up the kinematics, from world to tip.  Report the DOFs.
    kin = Kinematics(urdf, 'world', 'tip')
    N   = kin.dofs()
    rospy.loginfo("Loaded URDF for %d joints" % N)

    # Allocate the numpy variables for joint position, tip position,
    # tip orientation, and Jacobian.
    q = np.zeros((N,1))
    p = np.zeros((3,1))
    R = np.identity(3)
    J = np.zeros((6,N))

    # Set the numpy printing options (as not to be overly confusing).
    np.set_printoptions(suppress = True, precision = 6)


    # Set a joint position.
    q[0] = 0
    q[2] = -np.pi/2
    print(q)

    # Compute the fkin.
    kin.fkin(q, p, R)
    print(p)
    print(R)

    # Compute another rotation.
    print(Rz(np.pi/2) @ Rx(np.pi))

    # Compute the Jacobian.  NOTE KDL places the translational part in
    # the top 3 rows, and the rotational part in the bottom three
    # rows!  THIS IS REVERSED FROM MATLAB!!!  So be sure to place the
    # translational velocity/errors on top too.
    kin.Jac(q, J)
    print(J)

    # Check whether the Jacobian is singular.
    (U,S,VT) = np.linalg.svd(J)
    rospy.loginfo("Smallest singular value is %f", S[-1])
    print(U)
    print(S)
    print(VT)

    # Compare the SVD with the original.
    J1 = U @ np.diag(S) @ VT
    e = np.max(J - J1)
    print(e)

    # Compute an inverse velocity.
    ep = np.array([[1.0], [-1.0], [1.0]])
    eR = np.zeros((3,1))
    vd = np.array([[0.0], [2.0], [np.cos(1)]])
    wd = np.zeros((3,1))
    lam = 10.0
    vr = np.vstack((vd,wd)) + lam * np.vstack((ep,eR))

    qdot = np.linalg.pinv(J) @ vr
