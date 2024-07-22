import matplotlib.pyplot as plt
import csv
import numpy as np
import math
from moves12dof import Moves_12dof

# Reading the saved CSV file and plotting the data
def read_csv_and_plot(filename):
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
                    
    zs = np.array(joint_positions_data['yaw'])
    
    plt.plot(np.linspace(0, 1, len(zs)), zs,  label=max(zs))
    plt.plot(np.linspace(0, 1, len(zs)), np.zeros(len(zs)),  label='0')

    plt.ylabel('yaw')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    
    values = Moves_12dof.angles_for_height(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, cut=5, Rpose='stand_x')
   
    # joint_positions_data = {name: values[-1][i] for i, name in enumerate(joint_names)}
    # print(joint_positions_data)
    
    save_file = '/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/yaw.csv'
    
    read_csv_and_plot(save_file)
