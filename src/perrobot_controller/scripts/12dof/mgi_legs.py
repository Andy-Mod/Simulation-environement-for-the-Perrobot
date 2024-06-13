import numpy as np
import sympy as sp
from numpy import arcsin, arctan2, cos, pi, sin, arccos, sqrt

# Robot configurations
L1 = 19.5 * 0.001 
L2 = 160 * 0.001  
L3 = 160 * 0.001  
qdeg = [pi/2, 0, 0]

HALF_LEG_LENGTH = 0.16
TARGET_HEIGHT = 0.25

def calcul_angles(h, l):
    # Optimized angles computation
    arg = 0.5 * (h / l)
    alpha = arccos(arg)
    gamma = arcsin(arg)
    beta = gamma - alpha - pi / 2
    return alpha, beta

# Limits
x_limit = 0.1
z_limit = 0.3

def Numerical_MGD(q):
    theta1, theta2, theta3 = q
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
    end_effector = T01 @ T12 @ T23 @ np.array([0, 0, -L3, 1])
    return end_effector[:3]

def Analogical_MGD(q):
    q1, q2, q3 = q
    c1, s1 = cos(q1), sin(q1)
    c2, s2 = cos(q2), sin(q2)
    c3, s3 = cos(q3), sin(q3)
    d1 = -L3 * s3
    d2 = -L3 * c3 - L2
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    return np.array([d7, -s1 * d8, c1 * d8])

def precompute_jacobian():
    q1, q2, q3 = sp.symbols('q1 q2 q3')
    c1, s1 = sp.cos(q1), sp.sin(q1)
    c2, s2 = sp.cos(q2), sp.sin(q2)
    c3, s3 = sp.cos(q3), sp.sin(q3)
    d1 = -L3 * s3
    d2 = -L3 * c3 - L2
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    x, y, z = d7, -s1 * d8, c1 * d8
    J = sp.Matrix([[sp.diff(x, q) for q in (q1, q2, q3)],
                   [sp.diff(y, q) for q in (q1, q2, q3)],
                   [sp.diff(z, q) for q in (q1, q2, q3)]])
    return sp.lambdify((q1, q2, q3), J, 'numpy')

jacobian_func = precompute_jacobian()

def jacobian(q):
    return np.array(jacobian_func(*q), dtype=float)

def mgi(Xbut, qinit):
    x, y, z = Xbut
    
    # x += L1
    l = sqrt(x*x + z*z)
    s = sqrt(-x*x + L2*L2)
    q2 = arcsin(s/L2)
    q3 = pi - arccos((-l*l + 2*L2*L2)/(2*L2))
    
    q1 = arctan2(y, -z)
    
    return np.array([q1, q2, q3])