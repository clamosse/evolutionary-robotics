import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time

class SOLUTION: 

    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(3, 2)
        self.weights = (self.weights * 2) - 1
        self.myID = nextAvailableID

    def Evaluate(self, directOrGUI):
        pass

    def Start_Simulation(self,directOrGUI):
        self.Create_World() 
        self.Generate_Body()
        self.Generate_Brain()
        self.Create_Robot

        os.system(f"start /B python simulate.py {directOrGUI} {self.myID} 2>&1 &")
    
    def Wait_For_Simulation_To_End(self):
        while not os.path.exists(f"fitness{self.myID}.txt"):
            time.sleep(0.02)

        f = open(f"fitness{self.myID}.txt", "r")
        self.fitness = f.read()
        f.close()
        os.system(f"del fitness{self.myID}.txt")

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID


    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[-2,2,.5], size=[1,1,1])
        pyrosim.End()  

    def Create_Robot():
        pass  

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1.5], size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [.5,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=  [.5,0,-.5], size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-.5,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=  [-.5,0,-.5], size=[1,1,1])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        for currentRow in range(0,3):
            for currentColumn in range(0,2):
                    pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = currentColumn + 3 , weight = self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        self.weights[random.randint(0,2)][random.randint(0,1)] = (random.random() * 2) - 1