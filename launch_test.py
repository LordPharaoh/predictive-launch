from __future__ import division
import matplotlib.pyplot as plt
from math import sin, cos, pi

G = -9.8
AREA = .0637 ** 2
DRAG_COEFFICIENT = .41
DENSITY = 1.2
ROCKET_MASS = .523
F_GRAVITY = G * ROCKET_MASS

INITIAL_ANGLE = pi / 24

IMPULSE = [44.3, 42.3, 34, 30, 24, 17, 11, 6, 4] + ([0] * 80)

INTERVAL = .01

curr_time = 0
curr_v = 0
altitude = 0
total_impulse = 0

velocity = []
time = []
alt = []
predictions = []
acc = [0]
thrust_curve = []


def drag(v):
	return (-1 if v > 0 else 1) * DENSITY * DRAG_COEFFICIENT * AREA * (v ** 2)


def predict(v):
	cv = v
	height = 0
	while cv > 0:
		acc = G + drag(v)
		cv += acc * INTERVAL
		height += cv * INTERVAL
	return height


while altitude >= 0:
	idx = int(curr_time * 4)
	slope = -(IMPULSE[idx] - IMPULSE[idx + 1]) / .25
	thrust = IMPULSE[idx] + slope * (curr_time % .25)
	thrust_curve += [thrust]

	force = thrust + (F_GRAVITY if altitude > 0 else 0) + drag(curr_v)
	impulse = force * INTERVAL

	total_impulse += thrust * INTERVAL
	predictions.append(predict(curr_v) + altitude)

	curr_v += impulse / ROCKET_MASS
	acc += [impulse * 10 / ROCKET_MASS]
	altitude += curr_v * INTERVAL

	velocity += [curr_v]
	time += [curr_time]
	alt += [altitude]

	curr_time += INTERVAL

print(total_impulse)
plt.plot(time, alt, c="b")
plt.plot(time, velocity, c="r")
plt.plot(time, acc[1:], c="g")
plt.plot(time, predictions, c="y")
plt.show()
