import numpy as np
from numpy import pi, arccos, arcsin
from mgi_legs import Analogical_MGD, mgi
from matosc import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Moves_12dof:
    # Robot configurations 
    HAA_2_UPPER_LEG = 19.5 * 0.001 
    HALF_LEG_LENGTH = 160 * 0.001  
    TARGET_HEIGHT = 240 * 0.001
    
    topic = '/perrobot_12dof_controller/joint_states'
    
    @staticmethod
    def move_foot(qinit, period=0.1, amplitude=0.005, on_x=True, num_points=10, front=True):
         
        Xinit = Analogical_MGD(qinit)
        Xbut = Xinit.copy()
        
        first = []
        dt = 0.1
        
        _, swingshape = sinusoide_ich(period, amplitude, num_points)
        # plt.plot(swingshape)
        # plt.show()
        lineshape = np.zeros(num_points+1)
        
        if on_x:
            Xbut[0] -= period
            first = swingshape
        else:
            Xbut[1] -= period
            first = swingshape
            
        s = 2 if front else 3
        
        swingtrajectory = generate_trajectory_from_shape(Xinit, Xbut, first, num_points)
        out = np.array([mgi(xbut)[s] for xbut in swingtrajectory])
        
        # out = interpolation_qtraj(out, dt, sub)

        # plot_3d_points([Analogical_MGD(q) for q in out], 'r')
        return out
    
    @staticmethod
    def move_foot_linear_stance(qinit, freq=0.1, amplitude=0.005, on_x=True, num_points=10, sub=10, front=True):
        Xinit = Analogical_MGD(qinit)
        Xbut = Xinit.copy()
        
        first, second = [], []
        dt = 0.1
        
        _, swingshape = sinusoide_ich(freq, amplitude, num_points) # transform_and_shift_parabola(width=0.3, amplitude=0.01, t_span=1, num_points=num_points+1)  transform_and_shift_parabola(width=0.3, amplitude=-0.001, t_span=1, num_points=num_points+1)
       
        lineshape = np.zeros(num_points+1)
        
        if on_x:
            Xbut[0] += freq
            first, second = swingshape, lineshape
        else:
            Xbut[1] += freq
            first, second = lineshape, lineshape
            
        s = 2 if front else 3
        
        swingtrajectory = generate_trajectory_from_shape(Xinit, Xbut, first, num_points)
        qtraj = np.array([mgi(xbut)[s] for xbut in swingtrajectory])
        out = interpolation_qtraj(qtraj, dt, numberofpoints=sub)
        
        Xinit = Xbut
        Xbut = swingtrajectory[0]
        
        
        stancetrajectory = generate_trajectory_from_shape(Xinit, Xbut, second, num_points)
        qtraj = np.array([mgi(xbut)[s] for xbut in stancetrajectory])
        out2 = interpolation_qtraj(qtraj, dt, numberofpoints=sub)
        
        plot_3d_points(np.array([Analogical_MGD(q) for q in out]), 'r')
        plot_3d_points(np.array([Analogical_MGD(q) for q in out2]), 'g')
        
        # out = np.concatenate((out, out2))
        
        plot_3d_points(np.array([Analogical_MGD(q) for q in out]), 'b')
        
        plt.show()
        return out

    @staticmethod
    def move_set_of_legs(leg_positions: dict, freq=0.1, amplitude=0.005, on_x=True, num_points=10, sub=10):
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        
        movements = []
        leg_names = leg_positions.keys()
        positions = [leg_positions[k] for k in leg_names]
        
        for i, leg_name in enumerate(leg_names):  
            qinit = [0.0, q2, q3] if positions[i] else [0.0, -q2, -q3]
            movements.append(Moves_12dof.move_foot_linear_stance(qinit, freq, amplitude, on_x, num_points, sub, positions[i]))
        
        output_array = np.array(movements[0])
        
        for i in range(1, len(leg_names)):
            output_array = np.column_stack((output_array, movements[i]))
        
        
        return leg_names, output_array
    
    @staticmethod
    def MGD(q):
        return Analogical_MGD(q)
    
    @staticmethod
    def gait(gait_name, amp, length, stance_coef, num_point):
        legs = ['FL', 'FR', 'HL', 'HR']
        dt = 0.1
        
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        Xinit = Analogical_MGD(qinit)
        trajectory = generate_gait_shape(gait_name, Xinit, amp, length, stance_coef, num_point)

        qs = []
        
        for i, foot in enumerate(trajectory):
            s = 2 if 'F' in legs[i] else 3
        
            qs.append(np.array([mgi(xbut)[s] for xbut in foot]))
        
        return legs, qs
    
    @staticmethod
    def generate_sequences(a1, a2, n):
        a1, a2 = np.array(a1), np.array(a2)
        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]
    
    @staticmethod
    def hight_to_angles(h=TARGET_HEIGHT, L=HALF_LEG_LENGTH):
        arg = 0.5 * (h / L)
        q2 = arccos(arg)
        gamma = arcsin(arg)
        q3 = gamma - q2 - pi/2
        
        return q2, q3
   
    @staticmethod
    def pose(q2, q3, Rpose='stand_x'):
        poses = {
            'stand_x': [q2, q3, q2, q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_nx': [-q2, -q3, -q2, -q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'stand_ncc': [-q2, -q3, -q2, -q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_cc': [q2, q3, q2, q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'on_flor': [pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi, 0.0, 0.0, 0.0, 0.0],
        }
        if Rpose not in poses:
            return None
        if Rpose == 'floor_recovery':
            values = np.array(poses['stand_x']) + np.array([pi/2, -pi, pi/2, -pi, -pi/2, pi, -pi/2, pi, 0.0, 0.0, 0.0, 0.0])
            return list(values)
        return poses[Rpose]
    
    @staticmethod
    def get_to_pose(current_pose, target_pose, h=None, L=None, cut=30):
        q2, q3 = Moves_12dof.hight_to_angles(h, L)
        current_pose = Moves_12dof.pose(q2, q3, current_pose)
        target_pose = Moves_12dof.pose(q2, q3, target_pose)
        return Moves_12dof.generate_sequences(current_pose, target_pose, cut)
    
    @staticmethod
    def angles_for_height(h, L=None, cut=10, Rpose='stand_x'):
        L = L if L is not None else Moves_12dof.HALF_LEG_LENGTH
        path = np.linspace(0, h, cut)
        return [Moves_12dof.pose(*Moves_12dof.hight_to_angles(height, L), Rpose) for height in path]

    @staticmethod
    def robot_gait(gait_name, qinits, amplitude=0.05, freq=1, num_points=10):
        pass
        # TODO : flemme
    
    @staticmethod
    def visualize_gait_trajectories(gait_name, qinits, amplitude=0.05, freq=1, num_points=10):
        foot_trajectories = Moves_12dof.robot_gait(gait_name, qinits, amplitude, freq, num_points)
        colors = ['r', 'g', 'b', 'orange']
        for i, qtraj in enumerate(foot_trajectories):
            Xs = [Analogical_MGD(q) for q in qtraj]
            plot_3d_points(Xs, colors[i]) 

    @staticmethod
    def test():
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        qinits = [qinit, qinit, -qinit, -qinit] 
        gait_name = 's'
        Moves_12dof.visualize_gait_trajectories(gait_name, qinits, amplitude=0.05, freq=0.1, num_points=50)


