from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
from pathlib import Path
import sys

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

#scaraRobot2.teach([0,0,0,0], backend="pyplot3")

#Some target
someTarget=SE3(.5,.5,.5)
print("Plot green target")
plotRobotAndFrame(scaraRobot2,[0,0,0,0],someTarget,plt)