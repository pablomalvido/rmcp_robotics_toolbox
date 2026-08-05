from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np

#Defining the robot
planarRobot = DHRobot(
    [
        RevoluteMDH(),
        RevoluteMDH(a=0.75),
    ], name="planarRobot")

print(planarRobot)

planarRobot.teach([0,0], backend="pyplot3")