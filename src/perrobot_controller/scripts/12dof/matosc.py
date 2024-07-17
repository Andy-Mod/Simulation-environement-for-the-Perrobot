import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from oscitests import *

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

def sinusoide_shape(x, amp, length, phase_shift, stance_coef=1/2):
    
    omega = 2 * pi / length
    y = amp * np.sin(omega * x - length * phase_shift)
    y[y < 0] *= stance_coef
    
    return y 
    
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

def plot_3d_points(points, colors):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_vals = [point[0] for point in points]
    y_vals = [point[1] for point in points]
    z_vals = [point[2] for point in points]
    
    ax.scatter(x_vals, y_vals, z_vals, c=colors, marker='o',)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

def generate_trajectory_from_shape(start, end, shape, numberofpoints=10):
    x, y, z = start
    xf, yf, zf = end
    xout, yout, zout = 0.0, 0.0, 0.0
    
    xout, yout, zout = np.linspace(x, xf, numberofpoints+1), np.linspace(y, yf, numberofpoints+1), np.linspace(z, zf, numberofpoints+1)
    out = np.column_stack((xout, yout, zout+shape))
    
    return out 

def generate_gait_shape(gait_name, start, amp, length, stance_coef, num_point, it=2):
    phase_shifts = get_gait_phase_shifts(gait_name)
    x, y, z = start
    out = []
    x_values = np.linspace(0, 2*length, num_point+1)
    shapes = [sinusoide_shape(x_values, amp, 2*length, phase_shift, stance_coef) for phase_shift in phase_shifts]
    
    for i, shape in enumerate(shapes):
        plt.plot(x_values, shape, label=f"{i+1}")
    
    for i, zout in enumerate(shapes):
        
        z_swing = zout[zout > 0]
        z_stance = zout[zout <= 0]
        
        x_swing = np.linspace(0, length, len(z_swing))
        x_stance = np.linspace(length, 0, len(z_stance))
        
        xout = np.concatenate((x_swing, x_stance)) + x if length < 0 else np.concatenate((x_stance, x_swing)) + x
        yout = np.zeros(len(xout))
        points = np.column_stack((xout, yout, z + zout))
        out.append(points)
        
        #plot_3d_points(points, 'g')
    out = np.array(out)
    plt.show()
    
    return out[:, 1:]

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

def interpolation_qtraj(qs, tf, numberofpoints):
    """
        Systeme d'équation :
        q0, qf
        q_point0 = q_pointf = q_point_point0 = 0
        ti, qi pour i = [|1, n|] n fixed points on the foot trajectory
        
        q(t) = sum_k = 0 ^ n + 4 a_k * t^k
        a_0 = q0, a_1 = a_2 = 0
        q(ti) = qi
        
    """
    num = len(qs)
    t = np.linspace(0, tf, numberofpoints)
    tau = tf / num
    key_points = np.array(qs)
    ts = [i * tau for i in range(2, num)]
    ts.append(tf)
    
    q0 = qs[0]
    B = [q - q0 for q in qs[1:]]
    
    A = []
    
    for t_i in ts:
        A.append([t_i**i for i in range(3, num + 3)])
        
    A, B = np.array(A), np.array(B)
    
    x, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
    
    q_t = np.tile(q0, (numberofpoints, 1))
    
    for i, a in enumerate(x):
        j = i + 3
        q_t += a * (t**j)[:, np.newaxis]
    
    # plt.figure(figsize=(10, 6))
    # for i in range(q_t.shape[1]):
    #     plt.plot(t, q_t[:, i], label=f'q_{i}')
    
    # for i in range(key_points.shape[1]):
    #     plt.scatter(ts, key_points[1:, i], label=f'q_{i} key points', marker='o', s=100, zorder=10)
    # plt.xlabel('Time')
    # plt.ylabel('Position')
    # plt.title('Trajectory over time')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    
    return q_t

# amp = 0.01  
# length = 0.075
# stance_coef = 1
# start = [-0.0195, 0., -0.25]
# gait_name = 'walk'
# num_point = 20

# generate_gait_shape(gait_name, start, amp, length, stance_coef, num_point)