import numpy as np
from numpy import pi
import matplotlib.pyplot as plt

def cpg(t, freq, phase_shift, amplitude=1):
    """
    Implementation of a Central Pattern Generator (CPG) with phase-shifting parameters.
    
    Parameters:
        - t: Time
        - freq: Oscillation frequency
        - phase_shift: Phase shift for each oscillator
        
    Returns:
        - y: CPG output
    """
    omega = (2 * pi * freq)
    y = amplitude * np.sin(omega * (t + phase_shift))
    
    y[y < 0] = -y[y < 0]
    y[np.abs(y) < 1e-4] = 0.0
    index = np.where(y == 0)[0][0]
    
    y[:index] = None
    return y

def generate_parabolic_trajectory(start, end, amplitude, frequency, num_points=100):
   
    x0, y0, z0 = start
    x1, y1, z1 = end
    
    t = np.linspace(0, 1, num_points)
    
    x = x0 + t * (x1 - x0)
    y = y0 + t * (y1 - y0)
    
    height_variations = cpg(t, frequency, 0)
    
    z = z0 + t * (z1 - z0) + height_variations
    
    return [[x[i], y[i], z[i]] for i in range(len(x))]

def concatenate_trajectories(points, amplitude, frequency, num_points=30):
    
    full_x = []
    full_y = []
    full_z = []
    
    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]
        
        x, y, z = generate_parabolic_trajectory(start, end, amplitude, frequency, num_points)
        
        if i == 0:
            full_x.extend(x)
            full_y.extend(y)
            full_z.extend(z)
        else:
            full_x.extend(x[1:])
            full_y.extend(y[1:])
            full_z.extend(z[1:])
    
    return np.array(full_x), np.array(full_y), np.array(full_z)

# num_oscillators = 4
# amplitude = 0.5
# freq = 2 # between 1 and 3
# phase_shifts = [0, pi/2, pi, 3*pi/2]  

# num_points = 40000
# t = np.linspace(0, 2, num_points)

# cpg_outputs = []
# for i in range(num_oscillators):
#     cpg_output = cpg(t, freq, phase_shifts[i], amplitude=amplitude)
#     cpg_outputs.append(cpg_output)

# plt.figure(figsize=(10, 6))
# for i in range(num_oscillators):
#     plt.plot(t, cpg_outputs[i], label=f'Oscillator {i+1}')
# plt.xlabel('Time')
# plt.ylabel('CPG Output')
# plt.title('Central Pattern Generator (CPG) with Phase-Shifting Parameters')
# plt.legend()
# plt.grid()
# plt.show()
