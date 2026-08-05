import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib
matplotlib.use("TkAgg")

robot = rtb.models.Panda()
print(robot)

robot.teach([0,0.2,0,-0.96,0,1.22,0], backend='pyplot4')