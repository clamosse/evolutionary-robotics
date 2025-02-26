import numpy as np
import constants as c
import pybullet as p
import pyrosim.pyrosim as pyrosim

class MOTOR:

    def __init__(self,jointName):
        self.jointName = jointName

        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        if self.jointName == "Torso_BackLeg":
            self.frequency = 5

        else:
             self.frequency = 10

        self.amplitude = np.pi/4
        self.offset = 0

        self.motorValues = self.amplitude * np.sin(self.frequency * np.linspace(0, 2 * np.pi, c.num_steps) + self.offset)

    def Set_Value(self,robotId,desiredAngle):
            pyrosim.Set_Motor_For_Joint(
                bodyIndex = robotId,
                jointName = self.jointName,
                controlMode = p.POSITION_CONTROL,
                targetPosition = desiredAngle,
                maxForce = c.torso_backleg_max_force)
        
        
    def Save_Values(self):
        filename = f"data/{self.linkName}MotorValues.npy"
        np.save(filename, self.motorValues)