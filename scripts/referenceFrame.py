import numpy as np
from math import cos, sin

# Define distances
L_oa = 0.1
L_ab = 0.1
L_bp = 500. # Laser goes to infinity

# Define angles of rotation
phi_1 = 0  # rads
phi_2 = 0  # rads

################################################################################
def defineVectors (L1, L2, L3):
    """
    Gets the position vector of each reference frame expressed in the previous one
    """
    # First axis
    r1 = np.array([[0.],
                   [0.],
                   [L1]])
    # Second axis
    r2 = np.array([[0.],
                   [0.],
                   [L2]])
    # Laser beam
    r3 = np.array([[L3],
                   [0],
                   [0]])
    return r1, r2, r3
############################
def calcRotMat (phi1, phi2):
    """
    Calculates the two rotation matrices
    """
    # First axis of rotation
    R1 = np.array([[0, cos(phi1), -sin(phi1)],
                   [0, sin(phi1),  cos(phi1)],
                   [1, 0,          0        ]])
    # Second axis of rotation
    R2 = np.array([[cos(phi2), -sin(phi2), 0],
                   [sin(phi2),  cos(phi2), 0],
                   [0,          0,         1]])
    return R1, R2
####################
def calcHomogMat ():    
    """
    Calculates the homogeneous matrices for each axis and for the composition
    """
    # First axis
    H_oa = np.zeros((4, 4))
    H_oa[0:-1, 0:-1] = R_oa
    H_oa[0:-1, -1]   = r_oa_o.T
    H_oa[  -1, -1]   = 1
    # Second axis
    H_ab = np.zeros((4, 4))
    H_ab[0:-1, 0:-1] = R_ab
    H_ab[0:-1, -1]   = r_ab_a.T
    H_ab[  -1, -1]   = 1
    # Composition of the two rotations
    return H_oa.dot(H_ab)
##################
def calcPosVect():
    """
    Calculates the position vector for the 'end' of the laser beam
    """
    # Calculate the homogenenous matrix
    H = calcHomogMat() 
    # Define homogeneous equation
    r_homog = np.ones((4, 1))
    r_homog[0:-1] = r_bp_b
    return H.dot(r_homog)[0:-1] # Remove last (homogeneous) component

################################################################################
"""
### RUN ###

Functions:
    defineVectors <-- L1, L2, L3
    calcRotMat    <-- phi1, phi2
    calcHomogMat  <-- {}
    calcPosVect   <-- {}

Params:
    N     --> number of steps (angle positions)
    start --> initial position
    goal  --> goal position
     
"""
# Position vectors for each axis    
r_oa_o, r_ab_a, r_bp_b = defineVectors(L_oa, L_ab, L_bp)
# Define rotation matrices
R_oa, R_ab = calcRotMat(phi_1, phi_2)
# Get position vector of the 'end' of the the laser beam
r_p = calcPosVect()


