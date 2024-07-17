import matplotlib.pyplot as plt
import csv
import numpy as np
import math
from moves12dof import Moves_12dof

# Reading the saved CSV file and plotting the data
def read_csv_and_plot(filename, values_desired):
    """
    Read joint positions from CSV and plot the data.
    """
    joint_positions_data = {}
    
    # Read the CSV file
    with open(filename, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        headers = next(csv_reader)  # First row is the header

        # Initialize dictionary with joint names
        for header in headers:
            joint_positions_data[header] = []

        # Read the data rows
        for row in csv_reader:
            for header, value in zip(headers, row):
                if value:  # Check if the value is not empty
                    joint_positions_data[header].append(float(value))
    
    # Determine the number of rows and columns for the subplots
    num_joints = len(joint_positions_data)
    num_cols = 2
    num_rows = math.ceil(num_joints / num_cols)

    # Plotting
    fig, axes = plt.subplots(num_rows, num_cols, figsize=(10, num_rows * 3))
    fig.suptitle('Joint Positions Over Time')

    axes = axes.flatten()  # Flatten the axes array for easy iteration

    for ax, (joint_name, positions) in zip(axes, joint_positions_data.items()):
        x = np.linspace(0, 1, len(positions))
        des = values_desired[joint_name] * np.ones(len(x))
        ax.plot(x, positions, label=f"live value = {positions[-1]}")
        ax.plot(x, des, label=f"target value = {values_desired[joint_name]} \n error = {(values_desired[joint_name] - positions[-1])}", linestyle='--')
        ax.set_ylabel(joint_name)
        ax.legend()
        ax.grid(True)

    # Remove unused subplots
    for i in range(len(joint_positions_data), len(axes)):
        fig.delaxes(axes[i])

    plt.xlabel('Time')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    
    values = Moves_12dof.angles_for_height(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, cut=5, Rpose='stand_x')
    
    # List of joint names
    joint_names = [
        "FL_HFE",
        "FR_HFE",
        "HL_HFE",
        "HR_HFE",
        "FL_KFE",
        "FR_KFE",
        "HL_KFE",
        "HR_KFE",
        "FL_HAA",
        "FR_HAA",
        "HL_HAA",
        "HR_HAA"
    ]

    # Create a dictionary with joint names as keys and the last set of values from `values`
    joint_positions_data = {name: values[-1][i] for i, name in enumerate(joint_names)}
    print(joint_positions_data)
    
    save_file = '/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/joint_positions.csv'
    
    read_csv_and_plot(save_file, joint_positions_data)
