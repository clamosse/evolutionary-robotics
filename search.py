import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import random
import constants as c

"""
for i in range(0,5):
    os.system("python generate.py")
    os.system("python simulate.py")
"""

phc = PARALLEL_HILL_CLIMBER()

phc.Evolve()

phc.Show_Best()
phc.Save_Best_Brain()

phc.Plot_Comparison()
phc.Plot_Mutation_Rate_Trend()


#phc.Plot_Fitness_vs_Generation_Colored_By_Age(c.plotSampleSize)
#phc.Plot_Mutation_Rate_Trend()
#phc.Animate_Pareto_Progression()
