import os
from hillclimber import HILLCLIMBER

"""
for i in range(0,5):
    os.system("python generate.py")
    os.system("python simulate.py")
"""

hc = HILLCLIMBER()

hc.Evolve()

hc.Show_Best()