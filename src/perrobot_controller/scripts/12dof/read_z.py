import matplotlib.pyplot as plt
import csv
import numpy as np
import math
from moves12dof import Moves_12dof

# Reading the saved CSV file and plotting the data
def read_csv_and_plot(filename, theoric_values):
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
                    
    zs = np.array(joint_positions_data['position_z'])
    
    err = (theoric_values*np.ones(len(zs)) - zs - 0.001 * 5)**2
    
    plt.plot(np.linspace(0, 1, len(err)),err,  label=max(err))
    plt.ylabel('Quadratic error')
    plt.xlabel('Time (s)')
    
    plt.show()

if __name__ == '__main__':
    
    values = Moves_12dof.angles_for_height(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, cut=5, Rpose='stand_x')
   
    # joint_positions_data = {name: values[-1][i] for i, name in enumerate(joint_names)}
    # print(joint_positions_data)
    
    save_file = '/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/z.csv'
    
    read_csv_and_plot(save_file, 0.24)
