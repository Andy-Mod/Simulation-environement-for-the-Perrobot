import numpy as np
from numpy import pi
from scipy.integrate import solve_ivp

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
        ]
    }

    if gait_name not in gaits:
        raise ValueError(f"Unknown gait: {gait_name}. Supported gaits are: 'walk', 'trot', 'pace', 'gallop'")

    return gaits[gait_name]

def cpg(t_eval, mu=1e-7, amplitude=2e-2):
    def vdp(t, y, mu):
        x, v = y
        dxdt = v
        dvdt = mu * (1 - x**2) * v - x
        return [dxdt, dvdt]

    # Define initial conditions
    y0 = [1.0, 0.0]

    # Define time span

    # Solve the differential equation
    sol = solve_ivp(vdp, (t_eval[0], t_eval[-1]), y0, args=(mu,), t_eval=t_eval)
    t, x, x_point = np.array(sol.t), np.array(sol.y[0]), np.array(sol.y[1])
    
    x_point[x_point>1] = 1.0
    x_point[x_point<-1] = -1.0
    x_point[x_point<0] = -x_point[x_point<0]
    
    
    return x_point

def generate_parabolic_trajectory(start, end, amplitude, frequency, num_points=100, phase_shift=0):
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
    p = 6.3
    x0, y0, z0 = start
    x1, y1, z1 = end
    t = np.linspace(0, p/2, num_points)
    x = x0 + t * (x1 - x0)
    y = y0 + t * (y1 - y0)
    height_variations = cpg(t, amplitude=amplitude)
    z = z0 + t * (z1 - z0) + height_variations
    
    return np.column_stack((x, y, z))

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


    
    
# Example usage:
# num_oscillators = 4
# amplitude = 0.5
# freq = 2  # between 1 and 3
# phase_shifts = [0, pi/2, pi, 3*pi/2]

# num_points = 40000
# t = np.linspace(0, 2, num_points)

# cpg_outputs = [cpg(t, freq, phase_shifts[i], amplitude=amplitude) for i in range(num_oscillators)]

# plt.figure(figsize=(10, 6))
# for i in range(num_oscillators):
#     plt.plot(t, cpg_outputs[i], label=f'Oscillator {i+1}')
# plt.xlabel('Time')
# plt.ylabel('CPG Output')
# plt.title('Central Pattern Generator (CPG) with Phase-Shifting Parameters')
# plt.legend()
# plt.grid()
# plt.show()

