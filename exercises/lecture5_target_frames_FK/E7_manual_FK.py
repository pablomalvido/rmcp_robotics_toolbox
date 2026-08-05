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

#scaraRobot2.teach([0,0,0,0], backend="pyplot3")

#Some target
greenTarget=SE3.Trans(.6,.6,0.025/2) * SE3.Rz(-math.pi/4) * SE3.Ry(math.pi)
redTarget=SE3(.3,.8,.1)
#plotRobotAndFrame(scaraRobot2,[0,0,0,0],greenTarget,plt)

q = scaraRobot2.ikine_LM(greenTarget,mask=[1,1,1,0,0,1],q0=[np.pi/2, np.pi/2, np.pi/2, .4]) #Mask: We cannot control Rx and Ry cartesian. q0: Initial values, not any specific reason
print(q.q)
x_fk = scaraRobot2.fkine(q.q)
print(x_fk)
#plotRobotAndFrame(scaraRobot2,q.q,greenTarget,plt)

##### Manual FK #### 

def define_T(alpha, a, d, theta):
    rotX_aplha = SE3.Rx(alpha)
    translX_a = SE3.Trans(a, 0, 0)
    translZ_d = SE3.Trans(0, 0, d)
    rotZ_theta = SE3.Rz(theta)
    return rotX_aplha * translX_a * translZ_d * rotZ_theta

def manual_fk_scara(t1,t2,t3,d4):
    T_0_1 = define_T(0,0,0.5,t1)
    T_1_2 = define_T(0,0.4,0,t2)
    T_2_3 = define_T(0,0.3,0,t3)
    T_3_4 = define_T(np.pi,0,d4,0)
    T_0_4 = T_0_1*T_1_2*T_2_3*T_3_4

    T_world_0 = SE3.Trans(0.1,0.1,0.2) # From world to base
    T_4_tool = SE3.Trans(-0.03, 0, 0.135) # From last link to TCP
    T_world_tool = T_world_0 * T_0_4 * T_4_tool
    return T_world_tool

print(manual_fk_scara(*q.q))