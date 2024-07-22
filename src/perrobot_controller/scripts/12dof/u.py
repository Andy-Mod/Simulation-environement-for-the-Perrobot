from moves12dof import *
from mgi_legs import *


def generate_trajectory_from_shape(start, end, shape, numberofpoints=10):
    x, y, z = start
    xf, yf, zf = end
    xout, yout, zout = 0.0, 0.0, 0.0
    
    xout, yout, zout = np.linspace(x, xf, numberofpoints+1), np.linspace(y, yf, numberofpoints+1), np.linspace(z, zf, numberofpoints+1)
    out = np.column_stack((xout, yout, zout+shape))
    
    return out
    
def transform_and_shift_parabola(amplitude=0.001, num_points=200):
    t_span = 1
    t = np.linspace(-t_span, t_span, num_points)
    x = (t)**2
    z = amplitude*(((x-1))**2)  
    return t, z

numpoint = 10


t, s = transform_and_shift_parabola(amplitude=0.01, num_points=numpoint+1)

q2, q3 = calcul_angles(TARGET_HEIGHT, HALF_LEG_LENGTH)
q = np.array([0, q2, q3])

Xinit = Moves_12dof.MGD(q)
X_arriv = Xinit.copy() + [-0.05, 0, 0]

trajectory = generate_trajectory_from_shape(Xinit, X_arriv, s, numpoint)
mgi_ = [mgi(xyz)[2] for xyz in trajectory]
retraj = [Moves_12dof.MGD(q) for q in mgi_]

plot_3d_points(trajectory, 'g')

plot_3d_points(retraj, 'r')


err = [(np.linalg.norm(t) - np.linalg.norm(tr))**2 for t, tr in zip(trajectory, retraj)]

plt.plot(np.linspace(0, 1, numpoint+1), err, label='Quadratic error')
plt.legend()
plt.show()