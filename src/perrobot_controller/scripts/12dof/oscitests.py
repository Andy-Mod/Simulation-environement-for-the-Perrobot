import numpy as np
import matplotlib.pyplot as plt

def custom_sine_function(x, amp, omega, phase_shift):
    # Calculate the period of the sine wave
    period = 2 * np.pi / omega
    
    # Calculate the sine wave only within the period
    if 0 <= x <= period/2:
        return amp * np.sin(omega * (x - phase_shift))
    else:
        return 0

# Example usage
x_values = np.linspace(-1, 1, 1000)  # Generate a range of x values
amp = 1e-2  # Reduced amplitude
omega = 2 * np.pi / 2  # Adjusted omega for the given period
phase_shift = 0

y_values = [custom_sine_function(x, amp, omega, phase_shift) for x in x_values]

# Plotting the function
plt.plot(x_values, y_values)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.title('Custom Sine Function with Reduced Amplitude')
plt.grid(True)
plt.show()
