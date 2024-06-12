import matplotlib.pyplot as plt
import numpy as np

def custom_parabola(width=1, amplitude=1, t_span=1, num_points=200):
    """
    Generates a parabolic function with user-controlled width and amplitude.

    Parameters:
    - width (float): Factor to adjust the width of the parabola.
    - amplitude (float): Factor to adjust the amplitude of the parabola.
    - t_span (float): Span of the parameter t for generating x values.
    - num_points (int): Number of points to generate along the t span.

    Returns:
    - x (array): Array of x values.
    - y (array): Array of y values.
    """
    t = np.linspace(-t_span, t_span, num_points)
    x = t**2
    y = amplitude * ((width * (x - 1)) ** 2 + np.max(x))
    return x, y

# Example usage
width = 0.3
amplitude = 0.001
t_span = 1
num_points = 200

x, y = custom_parabola(width=width, amplitude=amplitude, t_span=t_span, num_points=num_points)

# Plot the custom parabola
plt.figure(figsize=(8, 6))
plt.plot(x, y, label="Custom Parabola", color='b')
plt.xlabel("x")
plt.ylabel("y")
plt.title("Custom Parabola with Width={}, Amplitude={}".format(width, amplitude))
plt.legend()
plt.grid(True)
plt.show()
