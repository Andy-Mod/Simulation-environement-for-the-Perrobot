import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft, fftfreq

def sinusoide_ich(t, width=0.3, amplitude=0.001):
    f = width * (t**2)
    x = amplitude * (-f/np.max(f) + 1)
    return x

def periodize_function_poisson(func, T, t, *args):
    # Sample the original function
    f_t = func(t, *args)
    
    # Compute the Fourier transform of the original function
    F_f_t = fft(f_t)
    
    # Frequency bins
    freq = fftfreq(t.size, d=(t[1] - t[0]))
    
    # Create a Dirac comb in the frequency domain with period 1/T
    comb_freq = np.zeros_like(F_f_t)
    comb_freq[::int(1/T/(freq[1] - freq[0]))] = 1
    
    # Convolve in the frequency domain (multiplication in time domain)
    F_periodic_f_t = F_f_t * comb_freq
    
    # Inverse Fourier transform to get back to the time domain
    f_periodic_t = ifft(F_periodic_f_t).real
    
    return f_periodic_t

# Parameters
T = 2.0  # Period
t = np.linspace(-5, 5, 1000)  # Time array

# Periodize the sinusoide_ich function using Poisson summation formula
x_periodic = periodize_function_poisson(sinusoide_ich, T, t)

# Plot the results
plt.plot(t, x_periodic)
plt.title('Périodisation de la fonction sinusoide_ich (Formule de Poisson)')
plt.xlabel('Temps (t)')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()
