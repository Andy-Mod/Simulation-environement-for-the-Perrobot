#!/bin/python3

import numpy as np
from numpy import arccos, arcsin, pi, cos, sin

def angles_for_height(h, L, cut=100, Rpose='x'):
    path = np.linspace(0, h, cut)
    Values = [[pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi]]
    
    Values.extend(hight_to_angles(height, L, Rpose) for height in path)
    
    return Values

def hight_to_angles(h, L, Rpose='x', s=False):
    arg = 0.5 * (h / L)
    
    alpha = arccos(arg)
    gamma = arcsin(arg)
    beta = gamma - alpha - pi / 2
    
    a, b = round(alpha, 2), round(beta, 2)
    
    if Rpose == 'x':
        values = [a, b, a, b, -a, -b, -a, -b]
    elif Rpose == 'nx':
        values = [-a, -b, -a, -b, a, b, a, b]
    elif Rpose == 'cc':
        values = [-a, -b, -a, -b, -a, -b, -a, -b]
    elif Rpose == 'ncc':
        values = [a, b, a, b, a, b, a, b]
    else:
        values = None
        
    return (a, b) if s else values

def angles_to_height(alpha, L):
    return round(2 * L * cos(alpha), 2)

def generate_sequences(a1, a2, n):
    a1 = np.array(a1)
    a2 = np.array(a2)
    
    return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]

def from_x_to_nx(h, L, cut=100):
    x = np.array(hight_to_angles(h, L, 'x'))
    nx = np.array(hight_to_angles(h, L, 'nx'))
    
    return generate_sequences(x, nx, cut)

def rotx(alpha):
    c, s = cos(alpha), sin(alpha)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def roty(beta):
    c, s = cos(beta), sin(beta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(gamma):
    c, s = cos(gamma), sin(gamma)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rotxyz(alpha, beta, gamma):
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))

def homog_transxyz(dx, dy, dz):
    return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])

def homog_transform(dx, dy, dz, alpha, beta, gamma):
    rot4x4 = np.eye(4)
    rot4x4[:3, :3] = rotxyz(alpha, beta, gamma)
    return np.dot(homog_transxyz(dx, dy, dz), rot4x4)

def homog_transform_inverse(matrix):
    inv = np.eye(4)
    inv[:3, :3] = matrix[:3, :3].T
    inv[:3, 3] = -np.dot(inv[:3, :3], matrix[:3, 3])
    return inv
