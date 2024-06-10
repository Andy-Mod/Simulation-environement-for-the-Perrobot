
import numpy as np
import sympy as sp
from numpy import arcsin, cos, pi, sin, arccos, arctan2, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot configurations 
L1 = 19.5 * 0.001 
L2 = 160 * 0.001  
L3 = 160 * 0.001  
qdeg = [pi/2, 0, 0]

HALF_LEG_LENGTH = 0.16
TARGET_HEIGHT = 0.25

def calcul_angles(h, l):
    # test angles coputation
    arg = 0.5 * (h / l)

    alpha = arccos(arg)
    gamma = arcsin(arg)
    beta = gamma - alpha - pi/2
    
    return alpha, beta

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

def Analogical_MGD(q):
    q1, q2, q3 = q
    c1 = cos(q1)
    s1 = sin(q1)
    c2 = cos(q2)
    s2 = sin(q2)
    c3 = cos(q3)
    s3 = sin(q3)
    d1 = (-L3 * s3)
    d2 = (-L3 * c3 - L2)
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    
    x = d7
    y = -s1 * d8
    z = c1 * d8
    
    return np.array([x, y, z])


def jacobian(q):
    q1_val, q2_val, q3_val = q
    # Define symbolic variables
    q1, q2, q3 = sp.symbols('q1 q2 q3')

    # Define expressions for c1, s1, c2, s2, c3, s3, d1, d2, d7, d8, x, y, z
    c1 = sp.cos(q1)
    s1 = sp.sin(q1)
    c2 = sp.cos(q2)
    s2 = sp.sin(q2)
    c3 = sp.cos(q3)
    s3 = sp.sin(q3)
    d1 = (-L3 * s3)
    d2 = (-L3 * c3 - L2)
    d7 = c2 * d1 + s2 * d2 - L1
    d8 = -s2 * d1 + c2 * d2
    x = d7
    y = -s1 * d8
    z = c1 * d8

    # Calculate the partial derivatives
    dx_dq1 = sp.diff(x, q1)
    dx_dq2 = sp.diff(x, q2)
    dx_dq3 = sp.diff(x, q3)
    dy_dq1 = sp.diff(y, q1)
    dy_dq2 = sp.diff(y, q2)
    dy_dq3 = sp.diff(y, q3)
    dz_dq1 = sp.diff(z, q1)
    dz_dq2 = sp.diff(z, q2)
    dz_dq3 = sp.diff(z, q3)

    # Define the Jacobian matrix
    J = sp.Matrix([[dx_dq1, dx_dq2, dx_dq3],
                   [dy_dq1, dy_dq2, dy_dq3],
                   [dz_dq1, dz_dq2, dz_dq3]])

    # Evaluate the Jacobian matrix numerically
    J_numeric = J.subs({q1: q1_val, q2: q2_val, q3: q3_val}).evalf()
    
    return np.array(J_numeric, dtype=float)

def mgi(Xbut, qinit):
    
    def direction(Xd, qdeg):
        Ja = jacobian(qdeg)
        dir = np.dot(Ja.transpose(), Xd-Analogical_MGD(qdeg))
        return dir
        
    def pas():
        p = 0.5
        return p

    Nmax = 200

    list_q = [qinit]
    list_erreur = [np.linalg.norm(Xbut-Analogical_MGD(qinit))*np.linalg.norm(Xbut-Analogical_MGD(qinit))]
    qtemp = qinit
    Xd = Xbut
    erreur_ok = 0.000001
    i = 0
    
    while i < Nmax and list_erreur[-1] > erreur_ok:
        qtemp = qtemp + np.dot(pas(), direction(Xd, qtemp))
        list_q.append(qtemp)
        list_erreur.append(np.linalg.norm(Xd-Analogical_MGD(qtemp))*np.linalg.norm(Xd-Analogical_MGD(qtemp)))
        i+=1
        
    
    return list_q[-1]
