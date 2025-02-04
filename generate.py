import pyrosim.pyrosim as pyrosim

def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[-2,2,.5], size=[1,1,1])
    pyrosim.End()


def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[0,0,1.5], size=[1,1,1])
    pyrosim.Send_Joint( name = "Torso_Front_Leg" , parent= "Torso" , child = "Front Leg" , type = "revolute", position = [.5,0,1])
    pyrosim.Send_Joint( name = "Torso_Back_Leg" , parent= "Torso" , child = "Back Leg" , type = "revolute", position = [-.5,0,1])
    pyrosim.Send_Cube(name="Front Leg", pos=  [.5,0,-.5], size=[1,1,1])
    pyrosim.Send_Cube(name="Back Leg", pos=  [-.5,0,-.5], size=[1,1,1])
    pyrosim.End()

Create_World()

Create_Robot()


