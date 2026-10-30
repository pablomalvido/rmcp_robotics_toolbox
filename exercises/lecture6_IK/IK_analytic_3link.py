from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3

#Defining the robot
l1 = 0.5
l2 = 0.4
l3 = 0.1
tx = 0.6
ty = 0.3
rz = -np.pi/4

robot3 = DHRobot(
    [
        RevoluteMDH(),
        RevoluteMDH(a=l1),
        RevoluteMDH(a=l2),
    ], name="robot3")

# print(robot3)

robot3.tool = SE3.Tx(l3)

## Numeric ##
x = SE3(tx, ty, 0.0) * SE3.Rz(rz)
q = robot3.ikine_LM(x,mask=[1,1,0,0,0,1],q0=[-np.pi/2, np.pi/2, -np.pi/4]) #Numeric IK: requires target, mask cartesian, init search values
print(q.q)

x_fk = robot3.fkine(q.q)
print(x_fk)
robot3.teach(q.q, backend="pyplot3")

## Analytic IK ##
wx = tx - l3 * np.cos(-rz)
wy = ty + l3 * np.sin(-rz)

k = np.sqrt(wx**2 + wy**2)
beta = np.arctan(wy/wx)
alpha = np.arccos((l1**2 + k**2 - l2**2)/(2*l1*k))
q1 = beta + alpha

gamma = np.arccos((l1**2 + l2**2 - k**2)/(2*l1*l2))
q2 = -(np.pi - gamma)

q3 = rz - q1 - q2

print([q1,q2,q3])
x_fk = robot3.fkine([q1,q2,q3])
print(x_fk)
robot3.teach([q1,q2,q3], backend="pyplot3")