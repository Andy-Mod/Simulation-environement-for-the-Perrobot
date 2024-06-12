import matplotlib.pyplot as plt
import numpy as np

def dirac_approx(x, epsilon=1e-2):
    s = np.exp(-x**2 / (2 * epsilon**2)) / (epsilon * np.sqrt(2 * np.pi))
    return x, s / np.max(s)

def dirac_comb(x, period, epsilon=1e-2):
    s_comb = np.zeros_like(x)
    
    for k in range(-int(np.max(x) // period), int(np.max(x) // period) + 1):
        _, s = dirac_approx(x - k * period, epsilon)
        s_comb += np.roll(s, int(k * period * len(x) / (2 * np.max(x))))
    
    s_comb = np.where(s_comb > 0, 1, 0)
    return x, s_comb

def sinusoide_ich(t, width=0.3, amplitude=0.001):
    f = width * (t**2)
    x =  amplitude * (-f/np.max(f) + 1)
    return t, x

def convolution(f, g):
    pass

# Parameters
t_span = 0.5
period = 1/12
epsilon = 5e-4
num_values = 1000

# Define the time points where convolution will be evaluated
t = np.linspace(-t_span, t_span, 1000)
_, f = sinusoide_ich(t, 1, 1)
_, g = dirac_comb(t, period, epsilon)

# Compute the convolution (f * g)(t)
conv_result = convolution(f, g)

# Plotting the results
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(t, f, label='f(t)')
plt.plot(t, g, label='g(t)')
plt.title('Functions f(t) and g(t)')
plt.xlabel('t')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t, conv_result, label='(f * g)(t)')
plt.title('Convolution of f(t) and g(t)')
plt.xlabel('t')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()