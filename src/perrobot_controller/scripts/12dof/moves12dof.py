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
    TARGET_HEIGHT = 270 * 0.001
    topic = '/perrobot_12dof_controller/joint_states'
    
    @staticmethod
    def move_foot(qinit, freq=0.1, amplitude=0.005, num_points=10, phase_shift=0, on_x=True):
        
        Xinit = Analogical_MGD(qinit)
        traj = generate_parabolic_trajectory(Xinit, freq, amplitude, num_points, on_x)
        
        qtraj = [qinit]
        qtemp = qinit
        for point in traj:
            print(qtemp, point, Analogical_MGD(qtemp))
            qtemp = mgi(point)
            qtraj.append(qtemp)

        return np.array(qtraj)  
    
    @staticmethod
    def generate_sequences(a1, a2, n):
        a1 = np.array(a1)
        a2 = np.array(a2)
        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]
    
    @staticmethod
    def hight_to_angles(h=None, L=None):
        if h is None:
            h = Moves_12dof.TARGET_HEIGHT
        if L is None:
            L = Moves_12dof.HALF_LEG_LENGTH
        arg = 0.5 * (h / L)

        q2 = arccos(arg)
        gamma = arcsin(arg)
        q3 = gamma - q2 - pi/2
        
        return q2, q3

    @staticmethod
    def pose(q2, q3, Rpose='stand_x'):
        rpose = {
            'stand_x': [q2, q3, q2, q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_nx': [-q2, -q3, -q2, -q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'stand_ncc': [-q2, -q3, -q2, -q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_cc': [q2, q3, q2, q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'on_flor': [pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi, 0.0, 0.0, 0.0, 0.0],
        }
        
        if Rpose not in rpose:
            return None
        
        if Rpose == 'floor_recovery':
            values = np.array(rpose['stand_x']) + np.array([pi/2, -pi, pi/2, -pi, -pi/2, pi, -pi/2, pi, 0.0, 0.0, 0.0, 0.0])
            return list(values)
        
        return rpose[Rpose]
    
    @staticmethod
    def get_to_pose(current_pose, target_pose, h=None, L=None, cut=30):
        if h is None:
            h = Moves_12dof.TARGET_HEIGHT
        if L is None:
            L = Moves_12dof.HALF_LEG_LENGTH
            
        q2, q3 = Moves_12dof.hight_to_angles(h, L)
        current_pose = Moves_12dof.pose(q2, q3, current_pose)
        target_pose = Moves_12dof.pose(q2, q3, target_pose)
        
        return Moves_12dof.generate_sequences(current_pose, target_pose, cut)
    
    @staticmethod
    def angles_for_height(h, L=None, cut=10, Rpose='stand_x'):
        if L is None:
            L = Moves_12dof.HALF_LEG_LENGTH
        path = np.linspace(0, h, cut)
        Values = [Moves_12dof.pose(*Moves_12dof.hight_to_angles(height, L), Rpose) for height in path]
        return Values

    @staticmethod
    def robot_gait(gait_name, qinits, amplitude=0.05, freq=1, num_points=10):
        # gaitname : [FL, FR, HL, HR, s]
        phase_shifts = get_gait_phase_shifts(gait_name)
        foots_trajectories = []
        for i in range(len(phase_shifts)):
            foots_trajectories.append(Moves_12dof.move_foot(qinits[i], freq, amplitude, num_points, phase_shifts[i], True))
        
        return foots_trajectories
    
    @staticmethod
    def visualize_gait_trajectories(gait_name, qinits,amplitude=0.05, freq=1, num_points=10):
        """
        Visualizes the gait trajectories in 3D.
        
        Parameters:
            - gait_name: Name of the gait ('walk', 'trot', 'pace', 'gallop')
            - qinits: Initial joint configurations
            - xyz: Target positions for the feet
            - amplitude: Amplitude of the sinusoidal height variation
            - freq: Frequency of the sinusoidal height variation
            - num_points: Number of points in the trajectory
        """
        foot_trajectories = Moves_12dof.robot_gait(gait_name, qinits, amplitude, freq, num_points)
        colors = ['r', 'g', 'b', 'orange']
        
        for i, qtraj in enumerate(foot_trajectories):
            Xs = []
            for q in qtraj:
                Xs.append(Analogical_MGD(q))
            plot_3d_points(Xs, colors[i]) 


    @staticmethod
    def test():
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])

        qinits = [qinit, qinit, -qinit, -qinit] 

        gait_name = 's'
        Moves_12dof.visualize_gait_trajectories(gait_name, qinits, amplitude=0.05, freq=0.1, num_points=50)
