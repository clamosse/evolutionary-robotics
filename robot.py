import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

class ROBOT:

    def __init__(self,solutionID):
        self.robotId = p.loadURDF("body.urdf")
        self.solutionID = solutionID
        self.nn = NEURAL_NETWORK(f"brain{solutionID}.nndf")
        os.system(f"del brain{solutionID}.nndf")


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
                desiredAngle = self.nn.Get_Value_Of(neuronName)
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
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]


        with open(f"tmp{self.solutionID}.txt", "w") as f:
            f.write(str(xCoordinateOfLinkZero))

        os.system(f"ren tmp{self.solutionID}.txt fitness{self.solutionID}.txt")


    def Think(self):
        self.nn.Update()
        self.nn.Print()