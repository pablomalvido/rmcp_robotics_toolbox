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

# Jacobian expression derivation
J = sp.Matrix([
    [sp.diff(p[0], t1), sp.diff(p[0], t2)],
    [sp.diff(p[1], t1), sp.diff(p[1], t2)]
])

J = sp.simplify(J)
print("Jacobian")
sp.pprint(J)

# Numeric IK algorithm
g = sp.Matrix([0.456, 0.17]) # goal position [X,Y]
#g = sp.Matrix([1.0, 0]) #Unreachable goal
Q = sp.Matrix([np.pi/10, np.pi/10]) # joint values (arbitrary initial values)
thr = 0.001 # Threshold error
k = 0.01 # Iteration step
max_iter = 1000

e = FK[:2,3].subs({t1:Q[0], t2:Q[1]}).evalf()
position_error = g - e
error = float(position_error.norm())
error_history = [error]
i=0

while error > thr and i<max_iter:
    Jac = J.subs({t1: Q[0], t2: Q[1]}).evalf() # Compute Jacobian
    delta_Q = k * Jac.inv() * position_error # Compute new incremental step for the Q
    Q += delta_Q 
    e = FK[:2,3].subs({t1:Q[0], t2:Q[1]}).evalf() # FK to get the new end effector position
    position_error = g - e # Error from current end effector position to the goal position
    error = float(position_error.norm())
    error_history.append(error)
    i+=1

plt.figure()
plt.plot(error_history)
plt.xlabel("Iteration")
plt.ylabel("Position error")
plt.title("IK convergence")
plt.grid(True)
plt.show()

serialRobot.teach([float(Q[0]),float(Q[1])],backend='pyplot3')