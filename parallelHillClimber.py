from solution import SOLUTION
import constants as c
import copy
import os
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import matplotlib.animation as animation
import random

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        os.system("del best_brain.nndf")
        os.system("del saved_brain*")
        
        self.nextAvailableID = 0

        self.parents_A = {}
        self.parents_B = {}
        self.all_fitnesses_A = []
        self.all_fitnesses_B = []
        self.best_fitness_over_time_A = []
        self.avg_fitness_over_time_A = []
        self.best_fitness_over_time_B = []
        self.avg_fitness_over_time_B = []
        self.age_fitness_log = []
        self.mutation_rates_over_time = []

        for i in range(c.populationSize):
            self.parents_A[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
            self.parents_B[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents_A)
        self.Evaluate(self.parents_B)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_One_Gen_A(currentGeneration)
            self.Evolve_One_Gen_B(currentGeneration)

            self.Clean_Brains()


    def Save_Best_Brain(self):
        best_A = max(self.parents_A.values(), key=lambda sol: sol.fitness)
        os.system(f"copy brain{best_A.myID}.nndf best_brain_A.nndf")

        best_B = max(self.parents_B.values(), key=lambda sol: sol.fitness)
        os.system(f"copy brain{best_B.myID}.nndf best_brain_B.nndf")


    def Clean_Brains(self):
        os.system("del brain*.nndf")


    def Evolve_For_One_Generation(self,currentGeneration):
        self.Spawn()
        self.Introduce_Random_Individual(n = c.randomIndPerGen)
        self.currentGeneration = currentGeneration
        self.Mutate()
        self.Evaluate(self.children)
        self.Record_Fitness(currentGeneration)
        self.Print()
        self.Select()

    def Evolve_One_Gen_A(self, currentGeneration):
        self.children_A = self.Spawn(self.parents_A)
        self.currentGeneration = currentGeneration
        self.Mutate_A()
        self.Evaluate(self.children_A)
        self.Record_Fitness('A', currentGeneration)
        self.Select_A()


    def Evolve_One_Gen_B(self, currentGeneration):
        self.children_B = self.Spawn(self.parents_B)
        #self.Introduce_Random_Individual(self.children_B, c.randomIndPerGen)
        self.currentGeneration = currentGeneration
        self.Mutate_B()
        self.Evaluate(self.children_B)
        self.Record_Fitness('B', currentGeneration)
        self.Select_C()
    

    
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


    def Spawn(self, parents):
        children = {}
        for key in parents.keys():
            child = copy.deepcopy(parents[key])
            child.Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            child.fitness = None
            children[key] = child
        return children

    # default algo
    def Mutate_A(self):
        for child in self.children_A.values():
            child.Mutate_A()


    def Mutate_B(self):
        curr_avg_fitness = np.mean([parent.fitness for parent in self.parents_B.values()])
        recent_history = self.avg_fitness_over_time_B[-5:] if len(self.avg_fitness_over_time_B) >= 5 else self.avg_fitness_over_time_B
        prev_avg_fitness = np.mean(recent_history) if recent_history else curr_avg_fitness

        normalized_delta = abs(curr_avg_fitness - prev_avg_fitness) / (abs(prev_avg_fitness) + 1e-6)
        #adjusted_delta = np.log1p(normalized_delta) 
        exploration_factor = (1 / (1 + np.exp(5  * (normalized_delta - 0.05))))
        
        base_rate = 0.4
        gen_decay = np.exp(-0.01 * self.currentGeneration) 
        mutation_rate = base_rate * exploration_factor * gen_decay

        if self.currentGeneration == 0:
            mutation_rate = base_rate


        # Apply mutation
        for child in self.children_B.values():
            child.Mutate_B(mutation_rate)

        self.mutation_rates_over_time.append(mutation_rate)


    def Mutate_C(self):
        for child in self.children_B.values():
            child.Mutate_B()


    def Select_A(self):
        for i in self.parents_A.keys():
            if self.children_A[i].fitness > self.parents_A[i].fitness:
                self.parents_A[i] = self.children_A[i]


    def Select_B(self):
        # Combine parents and children into one list
        combined = list(self.parents_B.values()) + list(self.children_B.values())
        
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
        self.parents_B = {}
        for i, sol in enumerate(survivors):
            sol.age += 1
            self.parents_B[i] = sol

    def Select_C(self):
        for i in self.parents_B.keys():
            if self.children_B[i].fitness > self.parents_B[i].fitness:
                self.parents_B[i] = self.children_B[i]


    def Get_Non_Dominated(self, solutions):
        random.shuffle(solutions)  # Randomize input to break ties nondeterministically
        non_dominated = []
        for s in solutions:
            if not any(self.Dominates(other, s) for other in solutions if other != s):
                non_dominated.append(s)
        return non_dominated


    def Dominates(self, a, b):
        return (a.fitness >= b.fitness and a.age <= b.age) and (a.fitness > b.fitness or a.age < b.age)

    def Introduce_Random_Individual(self, children, n):
        for _ in range(n):
            id = self.nextAvailableID
            children[id] = SOLUTION(id)
            self.nextAvailableID += 1
    

    def Record_Fitness(self, label, currentGeneration):
        if label == 'A':
            fitness_values = [sol.fitness for sol in self.parents_A.values()]
            self.best_fitness_over_time_A.append(max(fitness_values))
            self.avg_fitness_over_time_A.append(np.mean(fitness_values))
            self.all_fitnesses_A.append(fitness_values)

        elif label == 'B':
            fitness_values = [sol.fitness for sol in self.parents_B.values()]
            self.best_fitness_over_time_B.append(max(fitness_values))
            self.avg_fitness_over_time_B.append(np.mean(fitness_values))
            self.all_fitnesses_B.append(fitness_values)

            for sol in self.parents_B.values():
                self.age_fitness_log.append((currentGeneration, sol.age, sol.fitness))



    def Plot_Comparison(self):

        plt.figure(figsize=(10, 6))
        generations = range(len(self.best_fitness_over_time_A))

        # Compute std deviation assuming self.all_fitnesses_A and self.all_fitnesses_B hold fitness values per generation
        def compute_std(fitness_lists):
            return [np.std(gen) if gen else 0 for gen in fitness_lists]

        std_A = compute_std(self.all_fitnesses_A)
        std_B = compute_std(self.all_fitnesses_B)

        # Strategy A (blue)
        plt.plot(generations, self.best_fitness_over_time_A, label='Default', color='blue', linestyle='-')
        plt.plot(generations, self.avg_fitness_over_time_A, label='Default', color='blue', linestyle='--')
        plt.fill_between(generations,
                        np.array(self.avg_fitness_over_time_A) - np.array(std_A),
                        np.array(self.avg_fitness_over_time_A) + np.array(std_A),
                        color='blue', alpha=0.1, label=' +- 2 Std Dev')

        # Strategy B (red)
        plt.plot(generations, self.best_fitness_over_time_B, label='Adaptive mutation', color='red', linestyle='-')
        plt.plot(generations, self.avg_fitness_over_time_B, label='Adaptive mutation', color='red', linestyle='--')
        plt.fill_between(generations,
                        np.array(self.avg_fitness_over_time_B) - np.array(std_B),
                        np.array(self.avg_fitness_over_time_B) + np.array(std_B),
                        color='red', alpha=0.1, label='+- 2 Std Dev')

        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        plt.title("Comparison of Fitness Across Strategies default vs adaptive mutation")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("comparison_fitness.png")
        plt.show()


    def Plot_Fitness_vs_Generation_Colored_By_Age(self, sample_size_per_gen):
        # Create DataFrame from the log
        df = pd.DataFrame(self.age_fitness_log, columns=["Generation", "Age", "Fitness"])

        df = df.groupby("Generation", group_keys=False).apply(
            lambda x: x.sample(min(len(x), sample_size_per_gen)))

        plt.figure(figsize=(10, 6))
        scatter = sns.stripplot(
            data=df,
            x="Generation",
            y="Fitness",
            hue="Age",
            jitter=0.4,
            alpha=0.7,
            palette="viridis",
            size=5,
            linewidth=0,
            legend=False
        )

        # Gradient color bar
        norm = plt.Normalize(df["Age"].min(), df["Age"].max())
        sm = plt.cm.ScalarMappable(cmap="viridis", norm=norm)
        sm.set_array([])

        # Explicitly pass 'ax' for colorbar
        cbar = plt.colorbar(sm, ax=plt.gca())  # Add ax=plt.gca() here
        cbar.set_label("Age")

        # Clean up x-axis
        scatter.set_xlabel("Generation")
        scatter.set_ylabel("Fitness")
        scatter.set_title("Fitness vs Generation (Age as Color)")

        # Show only some x ticks
        xticks = sorted(df["Generation"].unique())
        spacing = max(1, len(xticks) // 10)
        scatter.set_xticks(xticks[::spacing])

        plt.grid(True)
        plt.tight_layout()
        plt.savefig("Fitness_vs_Generation_Colored_By_Age.png")
        plt.show()


    def Animate_Pareto_Progression(self, frame_skip=1):
        # Convert log to DataFrame
        df = pd.DataFrame(self.age_fitness_log, columns=["Generation", "Age", "Fitness"])
        generations = sorted(df["Generation"].unique())[::frame_skip]

        fig, ax = plt.subplots(figsize=(8, 6))

        def update(frame_gen):
            ax.clear()
            gen_df = df[df["Generation"] == frame_gen]

            # Plot all individuals
            ax.scatter(gen_df["Age"], gen_df["Fitness"], c='gray', label="Individuals", alpha=0.6)

            # Compute and plot Pareto front
            points = gen_df[["Age", "Fitness"]].to_numpy()
            pareto_points = []
            for i, a in gen_df.iterrows():
                dominated = False
                for j, b in gen_df.iterrows():
                    if ((b["Fitness"] >= a["Fitness"] and b["Age"] <= a["Age"]) and
                        (b["Fitness"] > a["Fitness"] or b["Age"] < a["Age"])):
                        dominated = True
                        break
                if not dominated:
                    pareto_points.append((a["Age"], a["Fitness"]))
            pareto_points = np.array(pareto_points)
            if len(pareto_points) > 0:
                sorted_pf = pareto_points[np.argsort(pareto_points[:,0])]
                ax.plot(sorted_pf[:,0], sorted_pf[:,1], 'r-o', label="Pareto Front")

            ax.set_title(f"Generation {frame_gen}")
            ax.set_xlabel("Age")
            ax.set_ylabel("Fitness")
            ax.legend()
            ax.grid(True)
            ax.set_xlim(left=0)
            ax.set_ylim(bottom=0)

        ani = animation.FuncAnimation(fig, update, frames=generations, repeat=False)
        ani.save("pareto_front_progression.gif", fps=2)
        plt.close()


    def Plot_Mutation_Rate_Trend(self):
        plt.figure(figsize=(10, 5))
        plt.plot(self.mutation_rates_over_time, label="Avg Mutation Rate", color="darkorange")
        plt.xlabel("Generation")
        plt.ylabel("Mutation Rate")
        plt.title("Mutation Rate Decay Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("Mutation Rate Trend.png")
        plt.show()


    def Plot_Mutation_Rate_Trend(self):
        # Compute normalized delta over time
        normalized_deltas = []
        for i in range(len(self.avg_fitness_over_time_B)):
            curr_avg = self.avg_fitness_over_time_B[i]
            history = self.avg_fitness_over_time_B[max(0, i-5):i]
            prev_avg = np.mean(history) if history else curr_avg
            delta = abs(curr_avg - prev_avg) / (abs(prev_avg) + 1e-6)
            adjusted_delta = np.log1p(delta) 
            normalized_deltas.append(adjusted_delta)

        fig, ax1 = plt.subplots(figsize=(10, 5))

        # Plot mutation rate on the left y-axis
        ax1.plot(self.mutation_rates_over_time, label="Mutation Rate", color="darkorange")
        ax1.set_xlabel("Generation")
        ax1.set_ylabel("Mutation Rate", color="darkorange")
        ax1.tick_params(axis='y', labelcolor="darkorange")
        ax1.grid(True)

        # Plot normalized delta on the right y-axis
        ax2 = ax1.twinx()
        ax2.plot(normalized_deltas, label="Normalized Delta", color="steelblue")
        ax2.set_ylabel("Normalized Delta", color="steelblue")
        ax2.tick_params(axis='y', labelcolor="steelblue")

        # Title and layout
        plt.title("Mutation Rate and Normalized Delta Over Time")
        fig.tight_layout()
        plt.savefig("Mutation Rate and Delta Trend.png")
        plt.show()



    def Show_Best(self):
        best_A = max(self.parents_A.values(), key=lambda sol: sol.fitness)
        best_A.Start_Simulation("GUI")
        print(f"age: {best_A.age}")
        print(f"fitness: {best_A.fitness}")

        best_B = max(self.parents_B.values(), key=lambda sol: sol.fitness)
        best_B.Start_Simulation("GUI")
        print(f"age: {best_B.age}")
        print(f"fitness: {best_B.fitness}")
