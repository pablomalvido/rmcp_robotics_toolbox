from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np

# Scara: RRRP
# Case 1: RRR positive CCW, P positive UP
scaraRobot = DHRobot(
    [
        RevoluteMDH(d=0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(qlim=[-1,1]), #Limits for the prismatic movement
    ], name="scaraRobot")

print(scaraRobot)
scaraRobot.teach([0,0,0,0], backend="pyplot3")

# Case 2: RRR positive CCW, P positive DOWN
scaraRobot2 = DHRobot(
    [
        RevoluteMDH(d=0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(alpha=np.pi, qlim=[-1,1]), #Limits for the prismatic movement
    ], name="scaraRobot2")

print(scaraRobot2)
scaraRobot2.teach([0,0,0,0], backend="pyplot3")

# Case 3: RRR positive CW, P positive DOWN
scaraRobot3 = DHRobot(
    [
        RevoluteMDH(alpha=np.pi, d=-0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(qlim=[-1,1]), #Limits for the prismatic movement
    ], name="scaraRobot3")

print(scaraRobot3)
scaraRobot3.teach([0,0,0,0], backend="pyplot3")