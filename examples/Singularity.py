from roboticstoolbox import DHRobot,RevoluteMDH
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
import sympy as sp
from sympy import symbols, cos, sin

t1 = symbols('t1')
t2 = symbols('t2')

#RR
serialRobot = DHRobot(
    [
        RevoluteMDH(),
        RevoluteMDH(a=0.3),
    ], name="serialRobot")

serialRobot.tool = SE3.Trans(0.3, 0, 0)
serialRobot.base = SE3.Trans(0,0,0)

#serialRobot.teach([0,0],backend='pyplot3')

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

T_0_1 = define_T(0,0,0,t1)
T_1_2 = define_T(0,0.3,0,t2)
T_2_tool = Trans(0.3, 0, 0) # From last link to TCP

FK = T_0_1 * T_1_2 * T_2_tool

p = FK[:2,3] # :2 because x, y only
#q = sp.Matrix([t1, t2])
#J = p.jacobian(q)
#sp.pprint(J)

J = sp.Matrix([
    [sp.diff(p[0], t1), sp.diff(p[0], t2)],
    [sp.diff(p[1], t1), sp.diff(p[1], t2)]
])

J = sp.simplify(J)
print("Jacobian")
sp.pprint(J)

Jac = J.subs({t1: 1.5, t2: 0.0}).evalf() # Compute Jacobian
sp.pprint(Jac)
sp.pprint(Jac.inv()) #Not invertible due to singularity (Det(J)=0)