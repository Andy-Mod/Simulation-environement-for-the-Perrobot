import numpy as np
from matosc import plot_3d_points
from mgi_legs import mgi
from moves12dof import Moves_12dof

def rotate_around_z(v, alpha):
    """
    Rotates a 3D vector around the z-axis by an angle alpha.
    
    :param v: The initial 3D vector.
    :param alpha: The rotation angle in radians.
    :return: The rotated vector.
    """
    R_z = np.array([
        [np.cos(alpha), -np.sin(alpha), 0],
        [np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 1]
    ])
    return np.dot(R_z, v)

def generate_intermediate_points(v1, v2, num_points=10):
    """
    Generates a set of intermediate points between two vectors.
    
    :param v1: The initial vector.
    :param v2: The target vector after rotation.
    :param num_points: The number of intermediate points to generate.
    :return: A list of intermediate points.
    """
    points = [v1 + t * (v2 - v1) for t in np.linspace(0, 1, num_points)]
    return points

# Example usage
initial_vector = np.array([-0.0195, 0, -0.25])
angle_in_radians = np.pi / 4  # 45 degrees

# Rotate the initial vector around the z-axis
rotated_vector = rotate_around_z(initial_vector, angle_in_radians)

# Generate intermediate points
intermediate_points = generate_intermediate_points(initial_vector, rotated_vector, num_points=10)

print("Initial Vector:", initial_vector)
print("Rotated Vector:", rotated_vector)
print("Intermediate Points:")

ik = [mgi(xyz)[2] for xyz in intermediate_points]

plot_3d_points([Moves_12dof.MGD(q) for q in ik], 'b')
plot_3d_points(intermediate_points, 'g')
