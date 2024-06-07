
import numpy as np
from numpy import arcsin, cos, pi, sin, arccos, arctan2, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot configurations 
L1 = 19.5 * 0.001 
L2 = 160 * 0.001  
L3 = 160 * 0.001  

HALF_LEG_LENGTH = 0.16
TARGET_HEIGHT = 0.25

# test angles coputation
arg = 0.5 * (TARGET_HEIGHT / HALF_LEG_LENGTH)

alpha = arccos(arg)
gamma = arcsin(arg)
beta = gamma - alpha - pi/2

# limits 
x_limit = 0.1
z_limit = 0.3


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

def Analogical_MGD(q1, q2, q3):
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    c3 = np.cos(q3)
    s3 = np.sin(q3)
    d1 = (-L3 * s3)
    d2 = (-L3 * c3 - L2)
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    
    x = d7
    y = -s1 * d8
    z = c1 * d8
    
    return x, y, z
    
def mgi(x, y, z):
    
    q1 = np.arctan2(y, x)
    
    d7 = x
    d8 = z * np.sin(q1) - y * np.cos(q1)
    
    q3 = -np.arctan2(d8, d7)

    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c3 = np.cos(q3)
    s3 = np.sin(q3)

    d1 = (-L3 * s3)
    d2 = (-L3 * c3 - L2)
    d7 = x
    d8 = z * c1 - y * s1

    q2 = np.arctan2(c1 * d1 + s1 * d2 - L1, -s1 * d1 + c1 * d2)

    return q1, q2, q3

theta1, theta2, theta3 = 0.0, alpha, beta 
print(f"first values of angles : {theta1}, {theta2}, {theta3}")

x, y, z = Numerical_MGD(theta1, theta2, theta3)
print(f"End-Effector Position: {x}, {y}, {z}")

x, y, z = Analogical_MGD(theta1, theta2, theta3)
print(f"End-Effector Position mgd: {x}, {y}, {z}")

theta1, theta2, theta3 = mgi(x, y, z)

print(f"ik values of angles : {theta1}, {theta2}, {theta3}")
