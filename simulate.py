import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
from simulation import SIMULATION


"""
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0,0,c.gravity)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(c.num_steps)
frontLegSensorValues = np.zeros(c.num_steps)


backLeg_targetAngles = c.backLeg_amplitude * np.sin(c.backLeg_frequency * np.linspace(0, 2 * np.pi, c.num_steps) + c.backLeg_phaseOffset)
np.save('data/backLeg_targetAngles.npy', backLeg_targetAngles)


frontLeg_targetAngles = c.frontLeg_amplitude * np.sin(c.frontLeg_frequency * np.linspace(0, 2 * np.pi, c.num_steps) + c.frontLeg_phaseOffset)
np.save('data/frontLeg_targetAngles.npy', frontLeg_targetAngles)

for i in range(0,c.num_steps):
    p.stepSimulation()

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

    t.sleep(c.delta_time)

np.save('data/backLegSensorValues.npy', backLegSensorValues)
np.save('data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()

"""

simulation = SIMULATION()

SIMULATION.Run()