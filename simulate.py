import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
from simulation import SIMULATION
import sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]

simulation = SIMULATION(directOrGUI,solutionID)
simulation.Run()
simulation.Get_Fitness()

