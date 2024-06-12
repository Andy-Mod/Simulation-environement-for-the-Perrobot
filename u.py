import numpy as np
import matplotlib.pyplot as plt

def periodize_function(f, a, b):
    """
    Periodizes the function f(x) defined on [a, b].

    Parameters:
    f : function
        The non-periodic function to be periodized.
    a : float
        Lower bound of the interval where f is defined.
    b : float
        Upper bound of the interval where f is defined.

    Returns:
    function
        A new function F(x) that is periodic with period (b - a).
    """
    period = b - a

    def F(x):
        # Calculate the equivalent x in the interval [a, b]
        x_mapped = a + (x - a) % period
        return f(x_mapped)

    return F

# Example non-periodic function
def non_periodic_func(x, width=1, amplitude=1):
    f = width * (x**2)
    x = amplitude * (-f/np.max(f) + 1)
    return x

# Define the interval [a, b] where the function is defined
a = 0
b = 2

# Periodize the function
periodic_func = periodize_function(non_periodic_func, a, b)

# Plotting
x_values = np.linspace(-5, 5, 400)
y_values_original = non_periodic_func(x_values)
y_values_periodic = periodic_func(x_values)

plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values_original, label='Original Function f(x) = x', linestyle='--', color='blue')
plt.plot(x_values, y_values_periodic, label='Periodic Function F(x)', color='red')
plt.title('Periodization of a Function')
plt.xlabel('x')
plt.ylabel('y')
plt.axvline(x=a, color='gray', linestyle=':', label=f'a = {a}')
plt.axvline(x=b, color='gray', linestyle=':', label=f'b = {b}')
plt.legend()
plt.grid(True)
plt.show()
