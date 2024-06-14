import numpy as np
from numpy import pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Limits
x_limit = 0.1
z_limit = 0.27

def get_gait_phase_shifts(gait_name):
    """
    Returns the phase shifts for a given gait.

    Parameters:
        - gait_name: Name of the gait ('walk', 'trot', 'pace', 'gallop')

    Returns:
        - phase_shifts: Dictionary with foot names as keys and phase shifts in radians as values
    """
    # Phase shifts for common gaits
    # gaitname : [FL, FR, HL, HR]
    gaits = {
        'walk': [
             0,
             np.pi / 2,
             np.pi,
             3 * np.pi / 2
        ],
        'trot': [
             0,
             np.pi,
             np.pi,
             0
        ],
        'pace': [
             0,
             np.pi,
             np.pi,
             0
        ],
        'gallop': [
             0,
             np.pi / 2,
             np.pi,
             3 * np.pi / 2
        ], 
        's': [
            0,
            0,
            0,
            0
        ]
    }

    if gait_name not in gaits:
        raise ValueError(f"Unknown gait: {gait_name}. Supported gaits are: 'walk', 'trot', 'pace', 'gallop'")

    return gaits[gait_name]

def plot_3d_points(points, colors):
    """
    Plots a set of 3D points.
    
    Parameters:
    points (list of tuple): List of (x, y, z) coordinates of points.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Unpack the points
    x_vals = [point[0] for point in points]
    y_vals = [point[1] for point in points]
    z_vals = [point[2] for point in points]

    # Scatter plot
    ax.scatter(x_vals, y_vals, z_vals, c=colors, marker='o',)

    # Set labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

def transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=200):
    t = np.linspace(-t_span, t_span, num_points)
    x = (t)**2
    z = amplitude*((width * (x-1))**2)  
    return t, z

def sinusoide_ich(t_span, amplitude=1, num_points=500):
    t = np.linspace(-t_span, t_span, num_points+1)
    f =  (t**2)
    x = (-f + np.max(f))
    x = (amplitude/x[t==0]) * x 
    
    return t, x

def generate_parabolic_trajectory(start, freq=0.1, amplitude=0.05, num_points=100, on_x=True):
    """
    Generates a parabolic trajectory from start to end with a sinusoidal height variation.
    
    Parameters:
        - start: Starting point (x0, y0, z0)
        - end: Ending point (x1, y1, z1)
        - amplitude: Amplitude of the sinusoidal height variation
        - frequency: Frequency of the sinusoidal height variation
        - num_points: Number of points in the trajectory (default is 100)
        - phase_shift: Phase shift for the height variation (default is 0)
        
    Returns:
        - trajectory: Array of points (x, y, z) in the trajectory
    """
    x, y, z = start
    xf, yf, zf = start
    xout, yout, zout = 0.0, 0.0, 0.0
    
    _, s = sinusoide_ich(freq, amplitude, num_points)
    # _, s = transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=num_points+1)
    
    if on_x:
        xf += freq 
    else:
        yf += freq
         
    xout, yout, zout = np.linspace(x, xf, num_points+1), np.linspace(y, yf, num_points+1), np.linspace(z, zf, num_points+1)
    out = np.column_stack((xout, yout, zout+s))
    # plot_3d_points(out, 'g')
    
    return out 

def concatenate_trajectories(points, amplitude, frequency, num_points=30):
    """
    Concatenates multiple parabolic trajectories defined by a list of points.
    
    Parameters:
        - points: List of points defining the trajectory
        - amplitude: Amplitude of the sinusoidal height variation
        - frequency: Frequency of the sinusoidal height variation
        - num_points: Number of points in each segment of the trajectory (default is 30)
        
    Returns:
        - full_x: x-coordinates of the full trajectory
        - full_y: y-coordinates of the full trajectory
        - full_z: z-coordinates of the full trajectory
    """
    trajectories = [generate_parabolic_trajectory(points[i], points[i + 1], amplitude, frequency, num_points) 
                    for i in range(len(points) - 1)]
    
    full_trajectory = np.vstack(trajectories)
    return full_trajectory.T

