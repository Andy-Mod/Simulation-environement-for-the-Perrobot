import numpy as np
import sympy as sp
from numpy import arcsin, arctan, arctan2, cos, pi, sin, arccos, sqrt

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

def calcul_A(y, z, q1):
    return -y * sin(q1) + z * cos(q1)



def calcul_q3(X, Y, Z1, Z2):
    try:
        c3 = (Z1**2 + Z2**2 - X**2 - Y**2) / (2 * X * Y)
        sqrt_term = np.sqrt(1 - c3**2)
        return np.arctan2(-sqrt_term, c3), np.arctan2(sqrt_term, c3)
    except ValueError as e:
        print(f"Error in calcul_q3: {e}")
        return None, None

def calcul_q2(X, Y, Z1, Z2, q3):
    try:
        b1 = X + Y * np.cos(q3)
        b2 = Y * np.sin(q3)
        s2 = (b1 * Z2 - b2 * Z1) / (b1**2 + b2**2)
        c2 = (b1 * Z1 + b2 * Z2) / (b1**2 + b2**2)
        return np.arctan2(s2, c2)
    except ZeroDivisionError as e:
        print(f"Error in calcul_q2: {e}")
        return None

def mgi(Xbut):
    try:
        x, y, z = Xbut
        sol = []

        X, Y, Z = y, z, 0
        X1, Y1, Z1, Z2 = L2, L3, 0.0, -(x + L1)

        if X == 0 and Y != 0:
            s1 = Z / Y
            try:
                q1_1 = np.arctan2(s1, -np.sqrt(1 - s1**2)) 
                q1_2 = np.arctan2(s1, np.sqrt(1 - s1**2))
            except ValueError as e:
                print(f"Error in arctan2 or sqrt: {e}")
                return []

            Z1_1 = -calcul_A(y, z, q1_1)
            Z1_2 = -calcul_A(y, z, q1_2)

            q3_1_1, q3_1_2 = calcul_q3(X1, Y1, Z1_1, Z2)
            q3_2_1, q3_2_2 = calcul_q3(X1, Y1, Z1_2, Z2)

            q2_1_1 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_1)
            q2_1_2 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_2)
            q2_2_1 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_1)
            q2_2_2 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_2)

            sol.extend([
                [q1_1, q2_1_1, q3_1_1], 
                [q1_1, q2_1_2, q3_1_2],
                [q1_2, q2_2_1, q3_2_1],
                [q1_2, q2_2_2, q3_2_2]
            ])

        elif X != 0 and Y == 0:
            c1 = Z / X
            try:
                q1_1 = np.arctan2(c1, -np.sqrt(1 - c1**2)) 
                q1_2 = np.arctan2(c1, np.sqrt(1 - c1**2)) 
            except ValueError as e:
                print(f"Error in arctan2 or sqrt: {e}")
                return []

            Z1_1 = -calcul_A(y, z, q1_1)
            Z1_2 = -calcul_A(y, z, q1_2)

            q3_1_1, q3_1_2 = calcul_q3(X1, Y1, Z1_1, Z2)
            q3_2_1, q3_2_2 = calcul_q3(X1, Y1, Z1_2, Z2)

            q2_1_1 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_1)
            q2_1_2 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_2)
            q2_2_1 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_1)
            q2_2_2 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_2)

            sol.extend([
                [q1_1, q2_1_1, q3_1_1], 
                [q1_1, q2_1_2, q3_1_2],
                [q1_2, q2_2_1, q3_2_1],
                [q1_2, q2_2_2, q3_2_2]
            ])

        elif X != 0 and Y != 0 and Z == 0:
            q1_1 = np.arctan2(-X, Y) 
            q1_2 = (q1_1 + np.pi) % (2*pi)

            Z1_1 = -calcul_A(y, z, q1_1)
            Z1_2 = -calcul_A(y, z, q1_2)

            q3_1_1, q3_1_2 = calcul_q3(X1, Y1, Z1_1, Z2)
            q3_2_1, q3_2_2 = calcul_q3(X1, Y1, Z1_2, Z2)

            q2_1_1 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_1)
            q2_1_2 = calcul_q2(X1, Y1, Z1_1, Z2, q3_1_2)
            q2_2_1 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_1)
            q2_2_2 = calcul_q2(X1, Y1, Z1_2, Z2, q3_2_2)
            
            sol.extend([
                [q1_1, q2_1_1, q3_1_1], 
                [q1_1, q2_1_2, q3_1_2],
                [q1_2, q2_2_1, q3_2_1],
                [q1_2, q2_2_2, q3_2_2]
            ])

        # print(sol)
        return np.array(sol)
    except Exception as e:
        print(f"An error occurred in mgi: {e}")
        return []

# tests 

# q2, q3 = calcul_angles(TARGET_HEIGHT, HALF_LEG_LENGTH)
# q = np.array([0, q2, q3])
# Xbut = Analogical_MGD(q)
# print(q, Xbut, mgi(Xbut))