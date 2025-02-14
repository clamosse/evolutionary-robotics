import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)


backLeg_amplitude = np.pi/4
backLeg_frequency = 10
backLeg_phaseOffset = 0

backLeg_targetAngles = backLeg_amplitude * np.sin(backLeg_frequency * np.linspace(0, 2 * np.pi, 1000) + backLeg_phaseOffset)
np.save('data/backLeg_targetAngles.npy', backLeg_targetAngles)


frontLeg_amplitude = np.pi/4
frontLeg_frequency = 10
frontLeg_phaseOffset = np.pi/2

frontLeg_targetAngles = frontLeg_amplitude * np.sin(frontLeg_frequency * np.linspace(0, 2 * np.pi, 1000) + frontLeg_phaseOffset)
np.save('data/frontLeg_targetAngles.npy', frontLeg_targetAngles)

for i in range(0,1000):
    p.stepSimulation()

    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
    bodyIndex = robotId,
    jointName = "Torso_BackLeg",
    controlMode = p.POSITION_CONTROL,
    targetPosition = backLeg_targetAngles[i],
    maxForce = 50)

    pyrosim.Set_Motor_For_Joint(
    bodyIndex = robotId,
    jointName = "Torso_FrontLeg",
    controlMode = p.POSITION_CONTROL,
    targetPosition = frontLeg_targetAngles[i],
    maxForce = 50)

    t.sleep(1/60)

np.save('data/backLegSensorValues.npy', backLegSensorValues)
np.save('data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()