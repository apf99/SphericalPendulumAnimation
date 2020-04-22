import pygame
import sys
from pygame.locals import *
from math import sin, cos, tan, pi
import numpy as np
from numpy.linalg import inv

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

def update(θ, ϕ):
	x = scale*l * sin(θ) * cos(ϕ) + offset[0]
	y = scale*l * cos(θ) + offset[1]
	z = scale*l * sin(θ) * sin(ϕ)

	return (int(x), int(y), int(z))

def render(point):
	x, y, z = point[0], point[1], point[2]
	z_scale = (2 - z/(scale*l)) * 10.0

	if prev_point:
		pygame.draw.line(trace, LT_BLUE, prev_point, (x, y), int(z_scale*0.2))

	screen.fill(WHITE)	
	if is_tracing:
		screen.blit(trace, (0,0))

	pygame.draw.line(screen, BLACK, offset, (x,y), int(z_scale*0.3))
	pygame.draw.circle(screen, BLACK, offset, 8)
	pygame.draw.circle(screen, BLUE, (x, y), int(m*z_scale))

	return (x, y)

w, h = 1024, 768
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255,0,0)
BLUE = (0,0,255)
LT_BLUE = (230,230,255)
offset = (w//2, h//4)
scale = 100
is_tracing = True

screen = pygame.display.set_mode((w,h))
screen.fill(WHITE)
trace = screen.copy()
pygame.display.update()
clock = pygame.time.Clock()

# parameters
m = 3.0
l = 4.5
g = 9.81

prev_point = None
t = 0.0
delta_t = 0.02
y = np.array([0.0, 0.5, 1.5, 0.0])

pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 38)

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			sys.exit()

		if event.type == KEYDOWN:
			if event.key == K_t:
				is_tracing = not(is_tracing)
			if event.key == K_c:
				trace.fill(WHITE)

	point = update(y[2], y[3])
	prev_point = render(point)

	time_string = 'Time: {} seconds'.format(round(t,1))
	text = myfont.render(time_string, False, (0, 0, 0))
	screen.blit(text, (10,10))

	t += delta_t
	y = y + RK4_step(y, t, delta_t) 

	clock.tick(60)
	pygame.display.update()
