import numpy as np

gravity = -9.8
num_steps = 1000
delta_time = 1/200

backLeg_amplitude = np.pi/4
backLeg_frequency = 10
backLeg_phaseOffset = 0
torso_backleg_max_force = 50

frontLeg_amplitude = np.pi/4
frontLeg_frequency = 10
frontLeg_phaseOffset = np.pi/2
torso_frontleg_max_force = 50

numberOfGenerations = 10
populationSize = 10