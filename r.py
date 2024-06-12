import matplotlib.pyplot as plt
import numpy as np

def dirac_comb(x, period):
    s_comb = np.zeros_like(x)
    for k in range(-int(np.max(x) // period), int(np.max(x) // period) + 1):
        idx = np.argmin(np.abs(x - k * period))
        s_comb[idx] = 1
    return x, s_comb

def sinusoide_ich(t, width=0.3, amplitude=0.001):
    f = width * (t**2)
    x = amplitude * (-f/np.max(f) + 1)
    return t, x

def convolution_with_dirac_comb(f, t, period):
    len_f = len(f)
    conv_result = np.zeros(len_f)
    
    for k in range(-int(np.max(t) // period), int(np.max(t) // period) + 1):
        shifted_f = np.roll(f, int(k * period * len_f / (2 * np.max(t))))
        conv_result += shifted_f
    
    return conv_result

# Parameters
t_span = 3
period = 0.3
num_values = 1000

# Define the time points where convolution will be evaluated
t = np.linspace(-t_span, t_span, num_values)
_, f = sinusoide_ich(t, 1, 1)
_, g = dirac_comb(t, period)

# Compute the convolution (f * g)(t)
conv_result = convolution_with_dirac_comb(f, t, period)

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
