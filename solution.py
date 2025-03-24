import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c

class SOLUTION: 

    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
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
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=  [0,.5,0], size=[.2,1,.2])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-0.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=  [0,-.5,0], size=[.2,1,.2])
        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-.5,0,1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=  [-.5,0,0], size=[1,.2,.2])
        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [.5,0,1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=  [.5,0,0], size=[1,.2,.2])

        pyrosim.Send_Joint( name = "FrontLeg_LowerFrontLeg" , parent= "FrontLeg" , child = "LowerFrontLeg" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=  [0,0,-.5], size=[.2,.2,1])
        pyrosim.Send_Joint( name = "BackLeg_LowerBackLeg" , parent= "BackLeg" , child = "LowerBackLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeg", pos=  [0,0,-.5], size=[.2,.2,1])
        pyrosim.Send_Joint( name = "LeftLeg_LowerLeftLeg" , parent= "LeftLeg" , child = "LowerLeftLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=  [0,0,-.5], size=[.2,.2,1])
        pyrosim.Send_Joint( name = "RightLeg_LowerRightLeg" , parent= "RightLeg" , child = "LowerRightLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LowerRightLeg", pos=  [0,0,-.5], size=[.2,.2,1])
        
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        """
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightLeg")"
        """
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name = 5 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 6 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 8 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "BackLeg_LowerBackLeg")
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = "LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = "RightLeg_LowerRightLeg")

        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                    pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = currentColumn + c.numSensorNeurons , weight = self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        self.weights[random.randint(0,c.numSensorNeurons - 1)][random.randint(0,c.numMotorNeurons - 1)] = (random.random() * 2) - 1