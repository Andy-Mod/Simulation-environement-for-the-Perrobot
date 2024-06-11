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
    def move_foot(qinit, xyz=None, amplitude=0.005, freq=1, num_points=10, phase_shift=0):
        if xyz is None:
            xyz = [-Moves_12dof.HAA_2_UPPER_LEG, 0.0, -Moves_12dof.TARGET_HEIGHT]
        
        Xinit = Analogical_MGD(qinit)
        traj = generate_parabolic_trajectory(Xinit, xyz, amplitude, freq, num_points, phase_shift)
        
        qtraj = [qinit]
        qtemp = qinit
        for point in traj:
            print(qtemp, point)
            qtemp = mgi(point, qtemp)
            qtraj.append(qtemp)

        return np.array(qtraj[1:])  # Remove the initial qinit to keep the trajectory smooth
    
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
    def robot_gait(gait_name, qinits, xyz=None, amplitude=0.05, freq=1, num_points=10):
        # gaitname : [FL, FR, HL, HR]
        phase_shifts = get_gait_phase_shifts(gait_name)
        foots_trajectories = []
        for i in range(len(phase_shifts)):
            print(i)
            foots_trajectories.append(Moves_12dof.move_foot(qinits[i], xyz[i], amplitude, freq, num_points, phase_shifts[i]))
        
        return foots_trajectories
    
    @staticmethod
    def visualize_gait_trajectories(gait_name, qinits, xyz, amplitude=0.05, freq=1, num_points=10):
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
        foot_trajectories = Moves_12dof.robot_gait(gait_name, qinits, xyz, amplitude, freq, num_points)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        foot_names = ['Front Left', 'Front Right', 'Hind Left', 'Hind Right']
        colors = ['r', 'g', 'b', 'y']
        
        for i, traj in enumerate(foot_trajectories):
            traj = np.array(traj)
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=colors[i], label=foot_names[i])
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'{gait_name.capitalize()} Gait Trajectories')
        ax.legend()
        plt.show()

    @staticmethod
    def test():
        gait_name = 'walk'
   
        xyz = np.array([-Moves_12dof.HAA_2_UPPER_LEG, 0.0, -Moves_12dof.TARGET_HEIGHT])
        shift = np.array([0.1, 0, 0])
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])

        qinits = [qinit, qinit, qinit, qinit]
        xyzs = [xyz - shift, xyz - shift, xyz - shift, xyz - shift]  
        print(xyzs)
        gait_name = 'trot'
        Moves_12dof.visualize_gait_trajectories(gait_name, qinits, xyzs, amplitude=0.05, freq=1, num_points=50)
