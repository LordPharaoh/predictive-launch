from __future__ import division
import matplotlib.pyplot as plt

G = -9.8
AREA = .0637 ** 2
DRAG_COEFFICIENT = .41
TIME_SLICE = 0.001
DENSITY = 1.2
ROCKET_MASS = .523
F_GRAVITY = G * ROCKET_MASS

velocities = list(range(480))
velocities = [v / 2 for v in velocities]

def drag(v):
	return (-1 if v > 0 else 1) * DENSITY * DRAG_COEFFICIENT * AREA * (v ** 2)

def predict(v):
	cv = v
	height = 0
	while cv > 0:
		acc = G + drag(v)
		cv += acc * TIME_SLICE
		height += cv * TIME_SLICE
	return height

apogees = []
for v in velocities:
	apogees += [predict(v)]
plt.plot(velocities, apogees)
plt.show()
