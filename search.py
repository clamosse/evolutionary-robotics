import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import random

"""
for i in range(0,5):
    os.system("python generate.py")
    os.system("python simulate.py")
"""

phc = PARALLEL_HILL_CLIMBER()

phc.Evolve()
phc.Show_Best()