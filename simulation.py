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

        p.setGravity(0,0,c.gravity)

        self.world = WORLD()
        self.robot = ROBOT()

        pyrosim.Prepare_To_Simulate(self.robot.robotId)
    

    def Run():
        for i in range(0,c.num_steps):
            print(i)

            
            p.stepSimulation()

            """

            backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

            pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = "Torso_BackLeg",
            controlMode = p.POSITION_CONTROL,
            targetPosition = backLeg_targetAngles[i],
            maxForce = c.torso_backleg_max_force)

            pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = "Torso_FrontLeg",
            controlMode = p.POSITION_CONTROL,
            targetPosition = frontLeg_targetAngles[i],
            maxForce = c.torso_frontleg_max_force)
            """
            t.sleep(c.delta_time)
            
    def __del__(self):
        p.disconnect()