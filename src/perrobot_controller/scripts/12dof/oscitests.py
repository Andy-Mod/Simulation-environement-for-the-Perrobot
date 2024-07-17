import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def R_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])


def omega(T, beta, b, y):
    return (np.pi / (beta * T * (np.exp(-b* y) + 1))) + (np.pi / ((1 - beta) * T * (np.exp(b * y) + 1)))

def r_i(x, y):
    return np.sqrt(x**2 + y**2)


def system(t, variables, alpha, gamma, mu, beta, b, T, delta, thetas):
    N = len(variables) // 2
    x = variables[:N]
    y = variables[N:]
    dxdt = np.zeros(N)
    dydt = np.zeros(N)

    for i in range(N):
        r = r_i(x[i], y[i])
        w = omega(T, beta, b, y[i])
        interaction_sum = np.zeros(2)
        
        for j in range(4):
            R_theta = R_matrix(thetas[j])
            interaction_sum += R_theta @ np.array([x[i], y[i]])
        
        dxdt[i] = alpha * (mu - r**2) * x[i] - w * y[i] + delta * interaction_sum[0]
        dydt[i] = gamma * (mu - r**2) * y[i] + w * x[i] + delta * interaction_sum[1]

    return np.concatenate((dxdt, dydt))

def compute_xy(alpha, gamma, mu, beta, b, T, delta, thetas, initial_conditions, t_span, t_eval):
    sol = solve_ivp(system, t_span, initial_conditions, args=(alpha, gamma, mu, beta, b, T, delta, thetas), t_eval=t_eval)
    return sol.t, sol.y

def shapes(alpha = 50, gamma = 50 , amplitude = 1, beta = 0.75, speed_conversion = 50, length = 1, delta = 1, thetas = [0, np.pi, np.pi, 0], N = 4, it=1):
    
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
    t_eval = np.linspace(t_span[0], t_span[1], 500)

    t, sol = compute_xy(alpha, gamma, amplitude**2, beta, speed_conversion, length, delta, thetas, initial_conditions, t_span, t_eval)


    plt.figure(figsize=(10, 5))
    for i in range(N):
        plt.plot(t, sol[i], label=f'x{i+1}(t)')
        plt.plot(t, sol[N + i], label=f'y{i+1}(t)')
    plt.xlabel('Time')
    plt.ylabel('Values')
    plt.legend()
    plt.title('Solution of the System of Differential Equations')
    plt.show()
    
    return sol[0:N]

# shapes(length=0.075, it=1)