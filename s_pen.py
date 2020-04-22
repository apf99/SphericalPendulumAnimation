import numpy as np 
from numpy.linalg import inv
from matplotlib import pyplot as plt
from math import cos, sin, tan, pi


def G(y,t): 
	θ_d, ϕ_d, θ, ϕ = y[0], y[1], y[2], y[3]

	θ_dd = ϕ_d**2 * cos(θ) * sin(θ) - g/l * sin(θ)
	ϕ_dd = -2.0 * θ_d * ϕ_d / tan(θ)

	return np.array([θ_dd, ϕ_dd, θ_d, ϕ_d])


def RK4_step(y, t, dt):
	k1 = G(y,t)
	k2 = G(y+0.5*k1*dt, t+0.5*dt)
	k3 = G(y+0.5*k2*dt, t+0.5*dt)
	k4 = G(y+k3*dt, t+dt)

	return dt * (k1 + 2*k2 + 2*k3 + k4) /6

# variables
m = 2.0
l = 1.0
g = 9.81

delta_t = 0.01
time = np.arange(0.0, 5.0, delta_t)

# initial state
y = np.array([0, 1.0, 1.0 , 0])   # [velocity, displacement]

Y1 = []
Y2 = []

# time-stepping solution
for t in time:
	y = y + RK4_step(y, t, delta_t) 

	Y1.append(y[2])
	Y2.append(y[3])


# plot the result
plt.plot(time,Y1)
plt.plot(time,Y2)
plt.grid(True)
plt.legend(['θ', 'ϕ'], loc='lower right')
plt.show()