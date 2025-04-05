from solution import SOLUTION
import constants as c
import copy
import os
import numpy as np
import matplotlib.pyplot as plt

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        os.system("del best_brain.nndf")
        
        self.parents = {}
        self.nextAvailableID = 0
        self.best_fitness_over_time = []  # Track best fitness per generation
        self.avg_fitness_over_time = []   # Track avg fitness per generation

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
            self.Save_Best_Brain()

            if currentGeneration % 5 == 0:
                self.Save_Random_Brains(1,currentGeneration)

            self.Clean_Brains()

        self.Plot_Fitness_Trend()

    def Save_Random_Brains(self, n, currentGeneration):
        selected = np.random.choice(list(self.parents.values()), size=n, replace=False)
        for i, sol in enumerate(selected):
            filename = f"saved_brain_{i+1}_{currentGeneration}.nndf"
            os.system(f"copy brain{sol.myID}.nndf {filename}")
            print(f"Saved brain of Solution {sol.myID} as {filename}")


    def Save_Best_Brain(self):
        best = max(self.parents.values(), key=lambda sol: sol.fitness)
        os.system(f"copy brain{best.myID}.nndf best_brain.nndf")


    def Clean_Brains(self):
        os.system("del brain*.nndf")


    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Introduce_Random_Individual()
        self.Mutate()
        self.Evaluate(self.children)
        self.Record_Fitness()
        self.Print()
        self.Select()


    def Evaluate(self, solutions):
        # Evaluate every solution that doesn't have a fitness
        solution_list = list(solutions.values())
        batch_size = 10
        for i in range(0, len(solution_list), batch_size):
            for sol in solution_list[i:i+batch_size]:
                if sol.fitness is None:
                    sol.Start_Simulation("DIRECT")
            for sol in solution_list[i:i+batch_size]:
                if sol.fitness is None:
                    sol.Wait_For_Simulation_To_End()
                    print(f"Solution {sol.myID} evaluated with fitness {sol.fitness}")
    
    def Print(self):
        for i in self.parents.keys():
            print(f"\nparent:{self.parents[i].fitness} child:{self.children[i].fitness}")

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            child = copy.deepcopy(self.parents[key])
            child.Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            child.fitness = None
            self.children[key] = child

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()   

    """
    def Select(self):
        for i in self.parents.keys():
            if self.children[i].fitness > self.parents[i].fitness:
                self.parents[i] = self.children[i]
    
    """
    def Select(self):
        # Combine parents and children into one list
        combined = list(self.parents.values()) + list(self.children.values())
        
        # Ensure every solution has been evaluated.
        for sol in combined:
            if sol.fitness is None:
                raise ValueError(f"Solution {sol.myID} has no fitness!")
        
        # Perform non-dominated sorting using fitness and age.
        survivors = self.Get_Non_Dominated(combined)
        
        # If survivors are too few, fill with the best individuals from the remaining solutions.
        if len(survivors) < c.populationSize:
            survivors_ids = set(sol.myID for sol in survivors)
            remaining = [sol for sol in combined if sol.myID not in survivors_ids]
            # Sort remaining individuals by fitness (best first)
            remaining.sort(key=lambda sol: sol.fitness, reverse=True)
            # Add until we reach the target population size
            while len(survivors) < c.populationSize and remaining:
                survivors.append(remaining.pop(0))
        
        # If survivors are too many, trim them (here, randomly)
        while len(survivors) > c.populationSize:
            survivors.pop(np.random.randint(len(survivors)))
        
        # Update parents: increment age and reset the parent dictionary.
        self.parents = {}
        for i, sol in enumerate(survivors):
            sol.age += 1
            self.parents[i] = sol


    def Get_Non_Dominated(self, solutions):
        non_dominated = []
        for s in solutions:
            dominated = False
            for other in solutions:
                if self.Dominates(other, s):
                    dominated = True
                    break
            if not dominated:
                non_dominated.append(s)
        return non_dominated

    def Dominates(self, a, b):
        return (a.fitness >= b.fitness and a.age <= b.age) and (a.fitness > b.fitness or a.age < b.age)

    def Introduce_Random_Individual(self):
        id = self.nextAvailableID
        self.children[id] = SOLUTION(id)
        self.nextAvailableID += 1

    def Record_Fitness(self):
        fitness_values = [sol.fitness for sol in self.parents.values()]
        self.best_fitness_over_time.append(max(fitness_values))
        self.avg_fitness_over_time.append(np.mean(fitness_values))
        print(f"Generation recorded: best = {max(fitness_values)}, avg = {np.mean(fitness_values)}")

    def Plot_Fitness_Trend(self):
        plt.figure(figsize=(10,5))
        plt.plot(self.best_fitness_over_time, label='Best Fitness')
        plt.plot(self.avg_fitness_over_time, label='Average Fitness')
        plt.xlabel("Generations")
        plt.ylabel("Fitness")
        plt.title("Fitness Trend Over Generations")
        plt.legend()
        plt.show()

    def Show_Best(self):
        best = max(self.parents.values(), key=lambda sol: sol.fitness)
        best.Start_Simulation("GUI")
