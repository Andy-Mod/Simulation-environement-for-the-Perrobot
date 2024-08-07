import numpy as np
from numpy import pi, arccos, arcsin
from mgi_legs import Analogical_MGD, mgi
from matosc import *
import matplotlib.pyplot as plt
from robot_utils import RobotUtils

class Moves_12dof:
    # Robot configurations
    HAA_2_UPPER_LEG = 19.5 * 0.001
    HALF_LEG_LENGTH = 160 * 0.001
    TARGET_HEIGHT = 270 * 0.001

    topic = '/perrobot_12dof_controller/joint_states'

    @staticmethod
    def move_foot(qinit, period=0.1, amplitude=0.005, on_x=True, num_points=10, front=True):
        Xinit = Analogical_MGD(qinit)
        Xbut = Xinit.copy()

        dt = 0.1
        t, swingshape = sinusoide_ich(period, amplitude, num_points)
        lineshape = np.zeros(num_points + 1)

        if on_x:
            Xbut[0] -= period
            first = swingshape
        else:
            Xbut[1] -= period
            first = swingshape

        s = 2 if front else 3
        swingtrajectory = generate_trajectory_from_shape(Xinit, Xbut, first, num_points)
        out = np.array([mgi(xbut)[s] for xbut in swingtrajectory])

        # plot_3d_points([Analogical_MGD(q) for q in out], 'g')

        return out

    @staticmethod
    def move_foot_linear_stance(qinit, freq=0.1, amplitude=0.005, on_x=True, num_points=10, sub=10, front=True):
        Xinit = Analogical_MGD(qinit)
        Xbut = Xinit.copy()

        dt = 0.1
        _, swingshape = sinusoide_ich(freq, amplitude, num_points)
        lineshape = np.zeros(num_points + 1)

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

        stancetrajectory = generate_trajectory_from_shape(Xinit, Xbut, second, num_points)
        qtraj = np.array([mgi(xbut)[s] for xbut in stancetrajectory])
        out2 = interpolation_qtraj(qtraj, dt, numberofpoints=sub)

        plot_3d_points([Analogical_MGD(q) for q in out], 'r')
        plot_3d_points([Analogical_MGD(q) for q in out2], 'g')

        plt.show()
        return out

    @staticmethod
    def move_set_of_legs(leg_positions, freq=0.1, amplitude=0.005, on_x=True, num_points=10, sub=10):
        q2, q3 = Moves_12dof.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH)

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
    def MGI(xyz):
        return mgi(xyz)

    @staticmethod
    def gait2(gait_name, amp, length, stance_coef, rate):
        q2, q3 = Moves_12dof.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        Xinit = Analogical_MGD(qinit)
        trajectory_build_and_publish_forward(gait_name, Xinit, amp, length, stance_coef, rate)

    @staticmethod
    def gait(gait_name, amp, length, stance_coef, num_point):
        dt = 0.1
        q2, q3 = Moves_12dof.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        Xinit = Analogical_MGD(qinit)
        trajectory = generate_gait_shape(gait_name, Xinit, amp, length, stance_coef, num_point)

        qs = []
        for i, foot in enumerate(trajectory):
            s = 2 if 'F' in Moves_12dof.legs[i] else 3
            qs.append(np.array([mgi(xbut)[s] for xbut in foot]))

        out = qs[0]
        for i in range(1, len(qs)):
            out = np.column_stack((out, qs[i]))

        return Moves_12dof.legs, out

    @staticmethod
    def generate_sequences(a1, a2, n):
        a1, a2 = np.array(a1), np.array(a2)
        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]

    @staticmethod
    def hight_to_angles(h=TARGET_HEIGHT, L=HALF_LEG_LENGTH):
        arg = 0.5 * (h / L)
        q2 = arccos(arg)
        gamma = arcsin(arg)
        q3 = gamma - q2 - pi / 2
        return q2, q3

    @staticmethod
    def pose(q2, q3, Rpose='stand_x'):
        poses = {
            'stand_x': [q2, q3, q2, q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_nx': [-q2, -q3, -q2, -q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'stand_ncc': [-q2, -q3, -q2, -q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_cc': [q2, q3, q2, q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'on_floor': [pi / 2, pi, pi / 2, pi, -pi / 2, -pi, -pi / 2, -pi, 0.0, 0.0, 0.0, 0.0],
        }
        poses['fall_recovery'] = list(np.array(poses['stand_x']) + np.array([pi / 2, -pi, pi / 2, -pi, -pi / 2, pi, -pi / 2, pi, 0.0, 0.0, 0.0, 0.0]))
        
        if Rpose not in poses:
            return None
       
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
        pass  # TODO: Implement robot_gait

    @staticmethod
    def visualize_gait_trajectories(gait_name, qinits, amplitude=0.05, freq=1, num_points=10):
        foot_trajectories = Moves_12dof.robot_gait(gait_name, qinits, amplitude, freq, num_points)
        colors = ['r', 'g', 'b', 'orange']
        for i, qtraj in enumerate(foot_trajectories):
            Xs = [Analogical_MGD(q) for q in qtraj]
            plot_3d_points(Xs, colors[i])

    @staticmethod
    def basic_MLCU(number_of_steps=10):
        current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        final = RobotUtils.walk_from_x(current_pose, number_of_steps, fast=True)

        final = np.array(final)
        FL = final[:, 0:2]
        FR = final[:, 2:4]
        HR = final[:, 4:6]
        HL = final[:, 6:]

        q1 = np.zeros((len(final), 4))

        FL = np.column_stack((np.zeros(len(FL)), FL))
        FR = np.column_stack((np.zeros(len(FL)), FR))
        HR = np.column_stack((np.zeros(len(FL)), HR))
        HL = np.column_stack((np.zeros(len(FL)), HL))

        plot_3d_points([Moves_12dof.MGD(q) for q in FL], 'g')

        return np.column_stack((final, q1))

    @staticmethod
    def steer(xyz, angle):
        rotated_vector = rotate_around_z(xyz, angle)
        intermediate_points = generate_intermediate_points(xyz, rotated_vector, num_points=15)
        ik = [mgi(xyz)[2] for xyz in intermediate_points]

        plot_3d_points([Moves_12dof.MGD(q) for q in ik], 'g')
        return ik
    
    @staticmethod
    def one_step():
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        num = 10
        
        FL = Moves_12dof.move_foot(qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        FR = Moves_12dof.move_foot(qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        HL = Moves_12dof.move_foot(-qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        HR = Moves_12dof.move_foot(-qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        
        values = np.column_stack((FR, HL))
        values2 = np.column_stack((FL, HR))
        
        sets = ['FR', 'HL']
        sets2 = ['FL', 'HR']
        
        return sets, sets2, values, values2
