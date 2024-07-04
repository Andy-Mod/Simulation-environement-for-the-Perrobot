import numpy as np
import matplotlib.pyplot as plt

class AdaptiveOscillator:
    def __init__(self, natural_frequency, coupling_strength):
        self.natural_frequency = natural_frequency
        self.coupling_strength = coupling_strength
        self.phase = 0.0
    
    def update(self, external_phase, dt):
        d_phase = self.natural_frequency + self.coupling_strength * np.sin(external_phase - self.phase)
        self.phase += d_phase * dt
    
    def get_phase(self):
        return self.phase

def create_and_couple_oscillators(natural_frequencies, coupling_strengths, phase_shifts, dt, T):
    num_oscillators = len(natural_frequencies)
    oscillators = [AdaptiveOscillator(natural_frequencies[i], coupling_strengths[i]) for i in range(num_oscillators)]
    
    time = np.arange(0, T, dt)
    phases = np.zeros((num_oscillators, len(time)))
    
    for i, t in enumerate(time):
        for j in range(num_oscillators):
            if j == 0:
                external_phase = oscillators[-1].get_phase() + phase_shifts[-1]
            else:
                external_phase = oscillators[j-1].get_phase() + phase_shifts[j-1]
            oscillators[j].update(external_phase, dt)
            phases[j, i] = oscillators[j].get_phase()
    
    return time, phases

# Parameters
natural_frequencies = [1.0, 1.0, 1.0, 1.0]  # Natural frequencies of the oscillators
coupling_strengths = [1, 1, 1, 1]  # Coupling strengths
phase_shifts = [0, np.pi/2, np.pi, 3*np.pi/2]  # Phase shifts between oscillators
dt = 0.01  # Time step
T = 20  # Total time

# Create and couple oscillators
time, phases = create_and_couple_oscillators(natural_frequencies, coupling_strengths, phase_shifts, dt, T)

# Plotting the results
plt.figure(figsize=(10, 5))
for i in range(len(natural_frequencies)):
    plt.plot(time, np.sin(phases[i]), label=f'Oscillator {i+1}')
plt.xlabel('Time')
plt.ylabel('Phase')
plt.legend()
plt.title('Coupled Adaptive Oscillators')
plt.show()
