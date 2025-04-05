import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c

class ROBOT:
    def __init__(self, solutionID):
        self.robotId = p.loadURDF("body.urdf")
        self.solutionID = solutionID
        if solutionID is None:
            brain_file = "best_brain.nndf"
        else:
            brain_file = f"brain{solutionID}.nndf"

        self.nn = NEURAL_NETWORK(brain_file)


    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self,i):
        for linkName in self.sensors:
            self.sensors[linkName].Get_Value(i)

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices :
            self.motors[jointName] = MOTOR(jointName)

    def Act(self,i):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self.robotId,desiredAngle)
                """
                print(jointName)
                print(neuronName)
                print(desiredAngle)
                """
    
        """
        for jointName in self.motors:
            self.motors[jointName].Set_Value(self.robotId,i)
        """

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]

        with open(f"tmp{self.solutionID}.txt", "w") as f:
            f.write(str(xPosition))

        os.system(f"ren tmp{self.solutionID}.txt fitness{self.solutionID}.txt")


    def Think(self):
        self.nn.Update()
        self.nn.Print()