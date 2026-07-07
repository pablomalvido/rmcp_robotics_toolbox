from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np

#Defining the robot
planarRobot = DHRobot(
    [
        RevoluteMDH(),
        RevoluteMDH(a=0.75),
    ], name="planarRobot")

print(planarRobot)

scaraRobot = DHRobot(
    [
        RevoluteMDH(d=0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(alpha=np.pi, qlim=[-1,1]),
    ], name="scaraRobot")

print(scaraRobot)

scaraRobot.teach([0,0,0,0], backend="pyplot3")