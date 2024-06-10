import numpy as np
from numpy import arcsin, cos, pi, sin, arccos, arctan2, sqrt, nan
import math

# Robot configurations 
L1 = 19.5 * 0.001 
L2 = 160 * 0.001  
L3 = 160 * 0.001  

HALF_LEG_LENGTH = 0.16
TARGET_HEIGHT = 0.25

# Compute test angles
arg = 0.5 * (TARGET_HEIGHT / HALF_LEG_LENGTH)
alpha = arccos(arg)
gamma = arcsin(arg)
beta = gamma - alpha - pi/2

def solve_quadratic(a, b, c):
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        return None
    else:
        sqrt_discriminant = math.sqrt(discriminant)
        x1 = (-b + sqrt_discriminant) / (2 * a)
        x2 = (-b - sqrt_discriminant) / (2 * a)
        return (x1, x2)

def Numerical_MGD(theta1, theta2, theta3):
    T01 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta1), -np.sin(theta1), 0],
        [0, np.sin(theta1), np.cos(theta1), 0],
        [0, 0, 0, 1]
    ])
    T12 = np.array([
        [np.cos(theta2), 0, np.sin(theta2), -L1],
        [0, 1, 0, 0],
        [-np.sin(theta2), 0, np.cos(theta2), 0],
        [0, 0, 0, 1]
    ])
    T23 = np.array([
        [np.cos(theta3), 0, np.sin(theta3), 0],
        [0, 1, 0, 0],
        [-np.sin(theta3), 0, np.cos(theta3), -L2],
        [0, 0, 0, 1]
    ])
    end_effector = np.dot(np.dot(np.dot(T01, T12), T23), np.array([0, 0, -L3, 1]))
    return end_effector[:3]

def Analogical_MGD(theta1, theta2, theta3):
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)
    s2 = np.sin(theta2)
    c3 = np.cos(theta3)
    s3 = np.sin(theta3)
    d1 = (-L3 * s3)
    d2 = (-L3 * c3 + L2)
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    
    x = d7
    y = -s1 * d8
    z = c1 * d8
    
    return x, y, z

def mgi(x, y, z):
    abort = False
    out = []
    q1 = arctan2(-y, z)

    a = L2 * L2
    b = 2 * L2 * (x + L1) 
    c = (x + L1)**2 - L3**2
    arg = ( -sqrt(y**2 + z**2) / L3)
    
    X = solve_quadratic(a, b, c)
    
    if X is not None:
        x1, x2 = X
        if x1 < -1 or x1 > 1:
            out = [nan, nan, nan]
            abort = True
        if x2 < -1 or x2 > 1:
            out = [nan, nan, nan]
            abort = True
        if not abort:
            q2_1 = arcsin(x1)
            q3_1 = arccos(arg) - q2_1
            q2_2 = arcsin(x2)
            q3_2 = arccos(arg) - q2_2
            out = [
                [q1, q2_1, q3_1],
                [q1, q2_2, q3_2]
            ]
    else:
        out = [nan, nan, nan]
    
    return out 

# Initial joint angles
theta1, theta2, theta3 = 0.0, alpha, beta 
print(f"Initial values of angles: theta1={theta1}, theta2={theta2}, theta3={theta3}")

# Compute end-effector position using numerical forward kinematics
numerical_position = Numerical_MGD(theta1, theta2, theta3)
print(f"Numerical Forward Kinematics - End-Effector Position: {numerical_position}")

# Compute end-effector position using analogical forward kinematics
analogical_position = Analogical_MGD(theta1, theta2, theta3)
print(f"Analogical Forward Kinematics - End-Effector Position: {analogical_position}")

# Compute inverse kinematics to find joint angles
ik_solutions = mgi(numerical_position[0], numerical_position[1], numerical_position[2])
print(f"Inverse Kinematics - Solutions for joint angles: {ik_solutions}")
