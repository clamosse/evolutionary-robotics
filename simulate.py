import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np
import math

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

for i in range(0,1000):
    p.stepSimulation()

    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
    bodyIndex = robotId,
    jointName = "Torso_BackLeg",
    controlMode = p.POSITION_CONTROL,
    targetPosition = -(math.pi)/6.0,
    maxForce = 500)

    pyrosim.Set_Motor_For_Joint(
    bodyIndex = robotId,
    jointName = "Torso_FrontLeg",
    controlMode = p.POSITION_CONTROL,
    targetPosition = (math.pi)/6.0,
    maxForce = 500)

    t.sleep(1/60)

np.save('data/backLegSensorValues.npy', backLegSensorValues)
np.save('data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()