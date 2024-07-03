import numpy as np
from numpy import pi, sin
import matplotlib.pyplot as plt

def custom_sine_function(x, amp, length, phase_shift, stance_coef=1/2):
    omega = 2 * pi / length
    
    y = amp * np.sin(omega * x - phase_shift)
    
    y[y < 0 ] *= stance_coef
    
    return y 

# Example usage

amp = 0.01  
length = 0.075
phase_shift = 0
stance_coef = 1/2
x_values = np.linspace(0, length, 100) 

y_values = custom_sine_function(x_values, amp, length, phase_shift, stance_coef)
z_values = custom_sine_function(x_values, amp, length, pi/2, stance_coef)
k_values = custom_sine_function(x_values, amp, length, pi, stance_coef)
u_values = custom_sine_function(x_values, amp, length, 3 * pi/2, stance_coef)

y = np.concatenate((y_values, y_values))
k = np.concatenate((k_values, k_values))
u = np.concatenate((u_values, u_values))
# Plotting the function
plt.plot(np.linspace(0, 1, len(y)), y)
plt.plot(np.linspace(0, 1, len(k)), k)
plt.plot(np.linspace(0, 1, len(u)), u)

plt.xlabel('x')
plt.legend()
plt.grid(True)
plt.show()
