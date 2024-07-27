import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def omega(T, beta, b, y):
    return (np.pi / (beta * T * (np.exp(-b* y) + 1))) + (np.pi / ((1 - beta) * T * (np.exp(b * y) + 1)))

def r_i(x, y):
    return np.sqrt(x**2 + y**2)


def system(t, variables, alpha, gamma, mu, beta, b, T, delta):
    N = len(variables) // 2
    x = variables[:N]
    y = variables[N:]
    dxdt = np.zeros(N)
    dydt = np.zeros(N)

    for i in range(N):
        r = r_i(x[i], y[i])
        w = omega(T, beta, b, y[i])
        interaction_sum = np.zeros(2)
        
        
        dxdt[i] = alpha * (mu - r**2) * x[i] - w * y[i] 
        dydt[i] = gamma * (mu - r**2) * y[i] + w * x[i] 

    return np.concatenate((dxdt, dydt))

def sinusoide_shape(x, amp, length, phase_shift):
    
    omega = 2 * np.pi / length
    y = amp * np.sin(omega * x - length * phase_shift)

    return y 

def compute_xy(alpha, gamma, mu, beta, b, T, delta, initial_conditions, t_span, t_eval):
    sol = solve_ivp(system, t_span, initial_conditions, args=(alpha, gamma, mu, beta, b, T, delta), t_eval=t_eval)
    return sol.t, sol.y

def shapes(alpha = 50, gamma = 50 , amplitude = 1, beta = 0.75, speed_conversion = 50, length = 1, delta = 1, thetas = [0, np.pi/2, np.pi, 3*np.pi/2], N = 4, it=1):
    
    # alpha = 50  # convergence speed control x : + faster convergence 
    # gamma = 50 # convergence speed control y : + faster convergence 
    # amplitude = 1
    # beta = 0.75
    # speed_conversion = 50
    # length = 1 
    # delta = 1  
    # thetas = [0, np.pi, np.pi/2, 3*np.pi/2]  
    
    initial_conditions = np.array([1, 1, 1, 1, 0, 0, 0, 0])
    # [0.99916672 0.560667   0.17523497 0.66727297 0.279664   0.72604911 0.97347175 0.62076692]
    # [0.19712773 0.53773971 0.11126199 0.70672853 0.659042   0.79878372 0.98920646 0.60917878]
    print(initial_conditions)

    t_span = (0, it*length)  
    t_eval = np.linspace(t_span[0], t_span[1], 1000)
   
    sub_shapes = [sinusoide_shape(t_eval, amplitude, length, theta) for theta in thetas]
    
    plt.figure(figsize=(10, 5))
    for i in range(N):
        x = sub_shapes[i]
        x = x 
        
        # y = sol[N + i]+ sub_shapes[i]
        plt.plot(t_eval, x, label=f'x{i+1}(t)')
        # plt.plot(t, y/np.max(y), label=f'y{i+1}(t)')
    plt.xlabel('Time')
    plt.ylabel('Values')
    plt.legend()
    plt.title('Solution of the System of Differential Equations')
    plt.show()
    
    return sub_shapes

# shapes(length=0.075, amplitude=0.025, it=1)