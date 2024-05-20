#!/bin/python3

from numpy import arccos, arcsin, pi, cos, sin
import numpy as np


def angles_for_hight(h, L, cut=10, Rpose='x'):
    
    path = [i*(h/cut) for i in range(cut)]
    Values = [[pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi]]
    
    for i in range(cut):
       Values.append(hight_to_angles(path[i], L, Rpose))
    
    return Values


def hight_to_angles(h, L, Rpose='x', s=False):
    arg = 0.5*(h/L)
    
    alpha = arccos(arg)
    gamma = arcsin(arg)
    beta = gamma - alpha - pi/2
    
    a, b = round(alpha, 2), round(beta, 2)
    
    if Rpose =='x':
        values = [
            a,
            b, 
            a,
            b,
            -a,
            -b,
            -a,
            -b
        ] 
    
    elif Rpose =='nx':
        values = [
            -a,
            -b, 
            -a,
            -b,
            a,
            b,
            a,
            b
        ]
    
    elif Rpose =='cc':
        values = [
            -a,
            -b, 
            -a,
            -b,
            -a,
            -b,
            -a,
            -b
        ]

    elif Rpose =='ncc':
        values = [
            a,
            b, 
            a,
            b,
            a,
            b,
            a,
            b
        ]
    
    else:
        values = None
    if s == False:
        return values
    else :
        return a, b   
        
def angles_to_hight(alpha, L):
    
    return round(2*L*cos(alpha), 2)


def from_x_to_nx(h, L):
    epsillon = 0.1
    x = hight_to_angles(h, L, 'x')
    nx = hight_to_angles(h, L, 'nx')
   
    
    
    

def rotx(alpha):
    """
    Create a 3x3 rotation matrix about the x axis
    """
    rx = np.array([[1, 0,          0,         ],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha) ]])

    return rx

def roty(beta):
    """
    Create a 3x3 rotation matrix about the y axis
    """
    ry = np.array([[cos(beta),   0, sin(beta)],
                   [0,           1, 0        ],
                   [-sin(beta),  0, cos(beta)]])

    return ry

def rotz(gamma):
    """
    Create a 3x3 rotation matrix about the z axis
    """
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma),  0],
                   [0,          0,           1]])

    return rz

def rotxyz(alpha, beta, gamma):
    """
    Create a 3x3 rotation matrix about the x,y,z axes
    """
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))

def homog_transxyz(dx, dy, dz):
    """
    Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)
    """
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans

def homog_transform(dx,dy,dz,alpha,beta,gamma):
    """
    Create a homogeneous 4x4 transformation matrix
    """
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rotxyz(alpha,beta,gamma)
    return np.dot(homog_transxyz(dx,dy,dz),rot4x4)

def homog_transform_inverse(matrix):
    """
    Return the inverse of a homogeneous transformation matrix.

                 ------------------------- 
                 |           |           |  
    inverse   =  |    R^T    |  -R^T * d | 
                 |___________|___________| 
                 | 0   0   0 |     1     | 
                 -------------------------  

    """
    inverse = matrix
    inverse[:3,:3] = inverse[:3,:3].T # R^T
    inverse[:3,3] = -np.dot(inverse[:3,:3],inverse[:3,3]) # -R^T * d
    return inverse