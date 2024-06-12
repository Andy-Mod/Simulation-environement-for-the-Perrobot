import matplotlib.pyplot as plt
import numpy as np

def transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=200):
    t = np.linspace(-t_span, t_span, num_points)
    x = (t)**2
    plt.plot(t, x, linestyle='--', color='g')
    return t, amplitude*((width * (x-1))**2 + np.max(x))



x, k_transformed = transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=200)
 
# Plot the original and transformed parabolas for comparison

plt.figure(figsize=(10, 6))
plt.plot(x, k_transformed, linestyle='--', label='0.5')

plt.plot(x, k_transformed, linestyle='--', label='10')
plt.xlabel("x")
plt.ylabel("y")
plt.title("Original and Transformed Parabolas")
plt.legend()
plt.grid(True)
plt.show()
