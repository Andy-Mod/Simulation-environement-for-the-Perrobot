import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Limits
x_limit = 0.1
z_limit = 0.27

def get_gait_phase_shifts(gait_name):
    
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

def generate_gait(period, phases_shift=[0, 1, 1, 0], amplitude=1, num_points_by_cycle=500):
    pass

def plot_3d_points(points, colors):
    
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

    # plt.show()
    

def square(width, amplitude, nb=5):
    x = np.linspace(0, width, nb)  # Adjust the number of points as needed
    half_width = width / 2.0
    y = np.zeros_like(x)
    y[(x >= half_width - width / (2 * nb)) & (x <= half_width + width / (2 * nb))] = amplitude
    return x, y
    

def transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=200):
    t = np.linspace(-t_span, t_span, num_points)
    x = (t)**2
    z = amplitude*((width * (x-1))**2)  
    return t, z

def sinusoide_ich(t_span, amplitude=1, num_points=500):
    t = np.linspace(-t_span, t_span, num_points+1)
    f =  (t**2)
    x = (-f + np.max(f))
    x = (amplitude/np.max(x)) * x 

    return t, x

def f(t_span, shifts, amplitude=0.01):
    t = np.linspace(-t_span, t_span, num_points+1) 
    f =  (t**2)
    x = (-f + np.max(f))
    x = (amplitude/np.max(x)) * x 
    
    t_combined = t
    x_combined = x
    for shift in shifts:
        
    
        # Combine t and x values
        t_combined = np.concatenate((t_combined, t - shift))
        x_combined = np.concatenate((x_combined, x))

        # Sort combined data by t values (optional)
        sorted_indices = np.argsort(t_combined)
        t_combined = t_combined[sorted_indices]
        x_combined = x_combined[sorted_indices]

    # Plot combined data
    plt.plot(t_combined, x_combined, label='test : combination')
    plt.legend()
    plt.show()
    
    return x


def generate_trajectory_from_shape(start, end, shape, numberofpoints=10):
    x, y, z = start
    xf, yf, zf = end
    xout, yout, zout = 0.0, 0.0, 0.0
    
    xout, yout, zout = np.linspace(x, xf, numberofpoints+1), np.linspace(y, yf, numberofpoints+1), np.linspace(z, zf, numberofpoints+1)
    out = np.column_stack((xout, yout, zout+shape))
    
    return out 

def generate_qtraj(qtraj, tf, numberofpoints=100):
    q0, q1, q2, q3, qf = qtraj[0], qtraj[1], qtraj[2], qtraj[3], qtraj[4]
    tau = tf / 5
    t1, t2, t3 = 2* tau, 3 * tau, 4 * tau
    t = np.linspace(0, tf, numberofpoints)
    
    A = np.array([
        [t1**3, t1**4, t1**5, t1**6],
        [t2**3, t2**4, t2**5, t2**6],
        [t3**3, t3**4, t3**5, t3**6],
        [tf**3, tf**4, tf**5, tf**6]
    ])
    
    B = np.array([
        q1 - q0,
        q2 - q0,
        q3 - q0,
        qf - q0
    ])
    
    x, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
    a3, a4, a5, a6 = x
    
    # Calculating the trajectory q_t
    q_t = q0 + a3 * (t**3)[:, np.newaxis] + a4 * (t**4)[:, np.newaxis] + a5 * (t**5)[:, np.newaxis] + a6 * (t**6)[:, np.newaxis]
    key_times = [0, t1, t2, t3, tf]
    key_points = qtraj
    
    plt.figure(figsize=(10, 6))
    for i in range(q_t.shape[1]):
        plt.plot(t, q_t[:, i], label=f'q_{i}')
    
    for i in range(key_points.shape[1]):
        plt.scatter(key_times, key_points[:, i], label=f'q_{i} key points', marker='o', s=100, zorder=10)
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.title('Trajectory over time')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    return q_t

def generate_parabolic_trajectory(start, freq=0.1, amplitude=0.05, num_points=100, on_x=True):
    
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
    
    trajectories = [generate_parabolic_trajectory(points[i], points[i + 1], amplitude, frequency, num_points) 
                    for i in range(len(points) - 1)]
    
    full_trajectory = np.vstack(trajectories)
    return full_trajectory.T


def cpgs(period, amplitude, phase_shifts, numberofpoints=10):
    t = np.linspace(0, period, numberofpoints)
    omega = 2 * pi / period
    xs = []
    
    for shift in phase_shifts:
        x = amplitude * np.abs(np.sin(omega * (t - shift)))
        xs.append(x)
    
    for i, shift in enumerate(shifts):
        plt.plot(t, xs[i], label=f'{shift}')
    
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.title('Trajectory over time')
    plt.legend()
    plt.grid(True)
    plt.show()

amplitude, freq, num_points = 0.01, 0.16, 10 
shifts = [
             0,
             np.pi / 2,
             np.pi,
             3 * np.pi / 2
        ]