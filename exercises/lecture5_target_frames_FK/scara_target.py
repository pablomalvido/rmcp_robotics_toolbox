from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
from pathlib import Path
import sys
import math

sys.path.append(str(Path(__file__).resolve().parents[1])) # add ws directory to the path to import from utils
from utils.plotHelper import plotRobotAndFrame, conditional_teach

# Scara: RRRP. RRR positive CCW, P positive DOWN
scaraRobot2 = DHRobot(
    [
        RevoluteMDH(d=0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(alpha=np.pi, qlim=[-1,1]), #Limits for the prismatic movement
    ], name="scaraRobot2")

print(scaraRobot2)

scaraRobot2.tool = SE3.Trans(-0.03, 0, 0.135)
scaraRobot2.base = SE3.Trans(0.1,0.1,0.2)

#Some target
greenTarget=SE3.Trans(.6,.6,0.025/2) * SE3.Rz(-math.pi/4) * SE3.Ry(math.pi)

q = scaraRobot2.ikine_LM(greenTarget,mask=[1,1,1,0,0,1],q0=[np.pi/2, np.pi/2, np.pi/2, .4]) #Mask: We cannot control Rx and Ry cartesian. q0: Initial values, not any specific reason
print(q.q)
x_fk = scaraRobot2.fkine(q.q)
print(x_fk)
plotRobotAndFrame(scaraRobot2,q.q,greenTarget,plt)