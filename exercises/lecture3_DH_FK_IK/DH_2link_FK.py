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

planarRobot.teach([0,0], backend="pyplot3")

# Forward kinematics
q = [0, np.pi/2]
print(planarRobot.fkine(q))
planarRobot.teach(q, backend="pyplot3")