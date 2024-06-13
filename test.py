import matplotlib.pyplot as plt
import numpy as np

def dirac_comb(x, period):
    s_comb = np.zeros_like(x)
    for k in range(-int(np.max(x) // period), int(np.max(x) // period) + 1):
        idx = np.argmin(np.abs(x - k * period))
        s_comb[idx] = 1
    return x, s_comb

def sinusoide_ich(t_span, amplitude=1, num_points=500):
    t = np.linspace(-t_span, t_span, num_points+1)
    
    f =  ((t)**2)
    x = (-f + np.max(f))
    x = (amplitude/x[t==0]) * x 
    
    return t, x

    

def transform_and_shift_parabola(width=0.3, amplitude=0.001, t_span=1, num_points=200):
    t = np.linspace(-t_span, t_span, num_points)
    x = (t)**2
    plt.plot(t, x, linestyle='--', color='g')
    z = amplitude*((width * (x-1))**2)  
    return t, z

def blend(f, z):
    s = f*z
    return s/np.max(z)

# Parameters
t_span = 1
period = 1/12
epsilon = 5e-4
num_values = 1000

# Define the time points where convolution will be evaluated
t = np.linspace(-t_span, t_span, 300)
tf, f = sinusoide_ich(t_span, 0.001)
_, f2 = sinusoide_ich(t_span, 0.01)
_, f3 = sinusoide_ich(t_span, 0.1)

print(np.max(f), np.max(f2))

# Plotting the results
plt.figure(figsize=(10, 6))

plt.plot(tf, f, label='f(t)', linestyle='--', color='g')
plt.plot(tf, f2, label='f(t)', linestyle='--', color='r')
# plt.plot(tf, f3, label='f(t)', linestyle='--', color='b')


plt.title('Functions f(t) and g(t)')
plt.xlabel('t')
plt.ylabel('Amplitude')
plt.grid(True)
plt.tight_layout()
plt.show()