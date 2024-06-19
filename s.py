import numpy as np
import matplotlib.pyplot as plt

def interpolate_arc_xz_plane(P0, P1, num_points):
    # Calculate midpoint in xz-plane
    Pmid1 = np.array([(P0[0] + P1[0]) / 2, 0, (P0[2] + P1[2]) / 2])
    
    # Normal vector for xz-plane
    N1 = np.array([0, 1, 0])
    
    # Choose orthogonal vectors U1 and V1
    U1 = np.array([1, 0, 0])  # Example, can be adjusted
    V1 = np.array([0, 0, 1])  # Example, can be adjusted
    
    # Calculate radius R1
    R1 = 0.5 * np.linalg.norm(P1 - P0) / np.sin(np.pi / 2)
    
    # Parametrize the arc in xz-plane
    theta = np.linspace(0, np.pi, num_points)
    arc_points_xz = []
    for t in theta:
        arc_point_xz = Pmid1 + R1 * (np.cos(t) * U1 + np.sin(t) * V1)
        arc_points_xz.append(arc_point_xz)
    
    return arc_points_xz

def interpolate_arc_yz_plane(P2, P3, num_points):
    # Calculate midpoint in yz-plane
    Pmid2 = np.array([0, (P2[1] + P3[1]) / 2, (P2[2] + P3[2]) / 2])
    
    # Normal vector for yz-plane
    N2 = np.array([1, 0, 0])
    
    # Choose orthogonal vectors U2 and V2
    U2 = np.array([0, 1, 0])  # Example, can be adjusted
    V2 = np.array([0, 0, 1])  # Example, can be adjusted
    
    # Calculate radius R2
    R2 = 0.5 * np.linalg.norm(P3 - P2) / np.sin(np.pi / 2)
    
    # Parametrize the arc in yz-plane
    phi = np.linspace(0, np.pi, num_points)
    arc_points_yz = []
    for p in phi:
        arc_point_yz = Pmid2 + R2 * (np.cos(p) * U2 + np.sin(p) * V2)
        arc_points_yz.append(arc_point_yz)
    
    return arc_points_yz

# Example points
P0 = np.array([1, 0, 1])
P1 = np.array([4, 0, 3])
P2 = np.array([0, 2, 1])
P3 = np.array([0, 4, 3])

num_points = 50
arc_points_xz = interpolate_arc_xz_plane(P0, P1, num_points)
arc_points_yz = interpolate_arc_yz_plane(P2, P3, num_points)

# Plotting
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')

# Plotting points
ax.scatter(*zip(P0, P1), color='blue', label='Points in xz-plane')
ax.scatter(*zip(P2, P3), color='green', label='Points in yz-plane')

# Plotting arcs
arc_points_xz = np.array(arc_points_xz)
arc_points_yz = np.array(arc_points_yz)

ax.plot(arc_points_xz[:,0], arc_points_xz[:,1], arc_points_xz[:,2], color='blue', label='Arc in xz-plane')
ax.plot(arc_points_yz[:,0], arc_points_yz[:,1], arc_points_yz[:,2], color='green', label='Arc in yz-plane')

# Customize plot
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Interpolated Arcs in xz and yz Planes')
ax.legend()

plt.show()
