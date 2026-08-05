from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
from pathlib import Path
import sys
import math
import sympy as sp
from sympy import symbols, cos, sin

t1 = symbols('t1')
t2 = symbols('t2')
t3 = symbols('t3')
d4 = symbols('d4')

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

##### Manual FK symbolic#### 
def RotX(alpha):
    return sp.Matrix([
        [1, 0,          0,           0],
        [0, cos(alpha), -sin(alpha), 0],
        [0, sin(alpha),  cos(alpha), 0],
        [0, 0,          0,           1]
    ])

def RotZ(theta):
    return sp.Matrix([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta),  cos(theta), 0, 0],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])

def TransX(a):
    return sp.Matrix([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def TransY(b):
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, b],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def TransZ(d):
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

def Trans(a,b,d):
    return TransX(a)*TransY(b)*TransZ(d)

def define_T(alpha, a, d, theta):
    return RotX(alpha) * TransX(a) * TransZ(d) * RotZ(theta)

T_0_1 = define_T(0,0,0.5,t1)
T_1_2 = define_T(0,0.4,0,t2)
T_2_3 = define_T(0,0.3,0,t3)
T_3_4 = define_T(np.pi,0,d4,0)
T_0_4 = T_0_1*T_1_2*T_2_3*T_3_4

T_world_0 = Trans(0.1,0.1,0.2) # From world to base
T_4_tool = Trans(-0.03, 0, 0.135) # From last link to TCP
T_world_tool = T_world_0 * T_0_4 * T_4_tool
sp.pprint(T_world_tool)

# Jacobian computation
p = T_world_tool[:3,3]

# another option, using the jacobian() method
#q = sp.Matrix([t1, t2, t3, d4])
#J = p.jacobian(q)

# Jacobian expression
J = sp.Matrix([
    [sp.diff(p[0], t1), sp.diff(p[0], t2), sp.diff(p[0], t3), sp.diff(p[0], d4)],
    [sp.diff(p[1], t1), sp.diff(p[1], t2), sp.diff(p[1], t3), sp.diff(p[1], d4)],
    [sp.diff(p[2], t1), sp.diff(p[2], t2), sp.diff(p[2], t3), sp.diff(p[2], d4)]
])
#sp.pprint(J)
J = sp.simplify(J)
sp.pprint(J)