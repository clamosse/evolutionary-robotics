from robot import ROBOT
from world import WORLD
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import time as t

class SIMULATION:

    def __init__(self):

        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        #p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

        self.world = WORLD()
        p.setGravity(0,0,c.gravity)
        self.robot = ROBOT()

        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.robot.Prepare_To_Sense()
        self.robot.Prepare_To_Act()
    

    def Run(self):
        for i in range(0,c.num_steps):
            print(i)
            
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)   

            t.sleep(c.delta_time)
            
    def __del__(self):
        p.disconnect()