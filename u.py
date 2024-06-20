import numpy as np
import matplotlib.pyplot as plt

# Define the number of oscillators
N = 4

# Define the natural frequencies of the oscillators
omega = np.ones(N) * 2* np.pi/1

# Define the coupling strength
K = 1

# Define the initial phases of the oscillators
theta =  np.array([0, 0.5 * np.pi, np.pi, 3/2 * np.pi])

# Define the time step and the number of time steps
dt = 0.01
T = 100000

# Define the array to store the phases at each time step
theta_t = np.zeros((T, N))
theta_t[0] = theta

# Run the simulation
for t in range(1, T):
    for i in range(N):
        theta_t[t, i] = theta_t[t-1, i] + dt*(omega[i] + K/N * np.sum(np.sin(theta_t[t-1] - theta_t[t-1, i])))

# Plot the simulation
plt.plot(theta_t)
plt.xlabel('Time')
plt.ylabel('Phase')
plt.show()