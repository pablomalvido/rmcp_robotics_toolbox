from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3

#Defining the robot
planarRobot = DHRobot(
    [
        RevoluteMDH(),
        RevoluteMDH(a=0.75),
    ], name="planarRobot")

print(planarRobot)

planarRobot.tool = SE3.Tx(0.5)

x = SE3(0.75, 0.4, 0.0)
q = planarRobot.ikine_LM(x,mask=[1,1,0,0,0,0],q0=[np.pi/2, -np.pi/2]) #target, mask cartesian, init search values
print(q.q)

x_fk = planarRobot.fkine(q.q)
print(x_fk)

planarRobot.teach(q.q, backend="pyplot3")