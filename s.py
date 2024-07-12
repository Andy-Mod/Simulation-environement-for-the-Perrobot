import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def omega(T, beta, y):
    return (np.pi / (beta * T * (np.exp(-beta * y) + 1))) + (np.pi / ((1 - beta) * T * (np.exp(beta * y) + 1)))

def system(t, variables, alpha, gamma, mu, beta, T):
    x, y = variables
    r_squared = x**2 + y**2
    w = omega(T, beta, y)
    dxdt = alpha * (mu - r_squared) * x - w * y
    dydt = gamma * (mu - r_squared) * y + w * x
    return [dxdt, dydt]

def compute_xy(alpha, gamma, mu, beta, T, initial_conditions, t_span, t_eval):
    sol = solve_ivp(system, t_span, initial_conditions, args=(alpha, gamma, mu, beta, T), t_eval=t_eval)
    return sol.t, sol.y

alpha = 10000  # convergence speed control x : + faster convergence 
gamma = 10000 # convergence speed control y : + faster convergence 
amplitude = 0.025 
speed_conversion = 0.5
length = 6  

x0 = 0.05
y0 = 0.0  
initial_conditions = [x0, y0]

t_span = (0, 2 * length)  
t_eval = np.linspace(t_span[0], t_span[1], 500)


t, sol = compute_xy(alpha, gamma, amplitude**2, speed_conversion, length, initial_conditions, t_span, t_eval)

plt.figure(figsize=(10, 5))
plt.plot(t, sol[0], label='x(t)')
plt.plot(t, sol[1], label='y(t)')
plt.xlabel('Time')
plt.ylabel('Values')
plt.legend()
plt.title('Solution of the System of Differential Equations')
plt.show()
