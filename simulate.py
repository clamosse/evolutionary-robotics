import pybullet as p
import time as t

physicsClient = p.connect(p.GUI)
#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

for i in range(0,1000):
    p.stepSimulation()
    t.sleep(1/60)
    print(i)

p.disconnect()