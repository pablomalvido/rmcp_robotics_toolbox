from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np

DH_Robot1 = DHRobot(
    [
        RevoluteMDH(d=0.4),
        RevoluteMDH(a=0.75),
        RevoluteMDH(a=0.5),
    ], name="DH_Robot")

DH_Robot2 = DHRobot(
    [
        RevoluteMDH(alpha=-np.pi/2),
        RevoluteMDH(alpha=np.pi/2),
        RevoluteMDH(a=0.75),
        RevoluteMDH(a=0.5),
    ], name="DH_Robot2")

DH_Robot3 = DHRobot(
    [
        RevoluteMDH(d=0.4),
        RevoluteMDH(alpha=-np.pi/2, d=0.15),
        RevoluteMDH(a=0.4, d=-0.15),
        RevoluteMDH(alpha=np.pi/2, d=0.3),
        RevoluteMDH(alpha=-np.pi/2),
    ], name="DH_Robot3")

#DH_Robot1.teach([0,0,0], backend="pyplot3")
#DH_Robot2.teach([0,0,0,0], backend="pyplot3")
DH_Robot3.teach([0,0,0,0,0], backend="pyplot3")