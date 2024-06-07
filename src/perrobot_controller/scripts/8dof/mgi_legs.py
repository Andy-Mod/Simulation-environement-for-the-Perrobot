import numpy as np
from numpy import arcsin, cos, pi, sin, arccos, arctan2, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_utils import RobotUtils

# Define link lengths
L1 = 19.5 * 0.001  # Length of Link 1 in meters
L2 = 160 * 0.001  # Length of Link 2 in meters
L3 = 160 * 0.001  # Length of Link 3 in meters

HALF_LEG_LENGTH = 0.16
TARGET_HEIGHT = 0.27

arg = 0.5 * (TARGET_HEIGHT / HALF_LEG_LENGTH)

alpha = arccos(arg)
gamma = arcsin(arg)
beta = gamma - alpha - pi/2

def mgd(q1, q2, q3):
    # Transformation from base to Link 1
    T01 = np.array([
        [1, 0, 0, 0],
        [0, cos(q1), -sin(q1), 0],
        [0, sin(q1), cos(q1), 0],
        [0, 0, 0, 1]
    ])

    # Transformation from Link 1 to Link 2
    T12 = np.array([
        [cos(q2), 0, sin(q2), L1],
        [0, 1, 0, 0],
        [-sin(q2), 0, cos(q2), 0],
        [0, 0, 0, 1]
    ])

    # Transformation from Link 2 to Link 3
    T23 = np.array([
        [cos(q3), 0, sin(q3), L2],
        [0, 1, 0, 0],
        [-sin(q3), 0, cos(q3), 0],
        [0, 0, 0, 1]
    ])

    # Transformation from Link 3 to end-effector
    T3E = np.array([
        [1, 0, 0, L3],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Compute the final transformation matrix from base to end-effector
    T0E = np.dot(T01, np.dot(T12, np.dot(T23, T3E)))
    
    TE0 = RobotUtils.homog_transform_inverse(T0E)

    # Extract the position of the end-effector
    x = T0E[0, 3]
    y = T0E[1, 3]
    z = T0E[2, 3]

    print(TE0)
    
    return [x, y, z]

def mgi(x, y, z):
    # Calculate q1 based on y and z positions
    q1 = arctan2(z, y)
    
    # Calculate q2 and q3 based on x position and remaining lengths
    # Solve for the positions
    r = sqrt(y**2 + z**2)
    D = ((x - L1)**2 + r**2 - L2**2 - L3**2) / (2 * L2 * L3)
    
    if D < -1 or D > 1:
        raise ValueError("Position out of reach for the robot")

    q3 = arccos(D)
    q2 = arctan2(r, x - L1) - arctan2(L3 * sin(q3), L2 + L3 * cos(q3))
    
    return [q1, q2, q3]

def plot_robot(q1, q2, q3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Base
    x_base = [0]
    y_base = [0]
    z_base = [0]
    
    # Link 1
    x1 = L1
    y1 = 0
    z1 = 0
    
    # Link 2
    x2 = x1 + L2 * cos(q2)
    y2 = L2 * sin(q2)
    z2 = 0
    
    # Link 3
    x3 = x2 + L3 * cos(q3)
    y3 = y2 + L3 * sin(q3)
    z3 = 0
    
    # Plot the links
    ax.plot([0, x1], [0, y1], [0, z1], 'r', label='Link 1')
    ax.plot([x1, x2], [y1, y2], [z1, z2], 'g', label='Link 2')
    ax.plot([x2, x3], [y2, y3], [z2, z3], 'b', label='Link 3')
    
    # Plot the end-effector
    ax.scatter(x3, y3, z3, c='k', marker='o', label='End-effector')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.show()

# Test the mgd function with sample joint angles
q1 = 0
q2 = alpha
q3 = beta
position = mgd(q1, q2, q3)
print(f"End-effector position: {position}")

# Plot the robot with the sample joint angles
plot_robot(q1, q2, q3)

# # Test the mgi function with a desired end-effector position
# desired_position = [0.3, 0.1, 0.2]
# joint_angles = mgi(*desired_position)
# print(f"Joint angles: {joint_angles}")

# # Verify the result with forward kinematics
# verified_position = mgd(*joint_angles)
# print(f"Verified end-effector position: {verified_position}")

# # Plot the robot with the calculated joint angles
# plot_robot(*joint_angles)
