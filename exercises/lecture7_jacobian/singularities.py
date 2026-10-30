import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3

#IRB 1100

L1 = 0.327
L2 = 0.225
L3 = 0.25

robot = DHRobot(
    [
        RevoluteMDH(d=L1),
        RevoluteMDH(alpha=np.pi/2),
        RevoluteMDH(a=L2),
        RevoluteMDH(alpha=np.pi/2, d=L3),
        RevoluteMDH(alpha=-np.pi/2),
        RevoluteMDH(alpha=np.pi/2),
    ]
)
print(robot)

# Non singular configuration
q = np.deg2rad([30, 60, 0, 0, 90, 0]) #Random

#Singular configurations
q2 = 130*np.pi/180
q3 = (np.pi - np.arcsin((L2*np.sin(q2-np.pi/2))/L3) - (q2-np.pi/2)) - np.pi/2
q5 = np.pi - q2 - q3
q = [0, q2, q3, 0, q5, 0] #Shoulder singularity

q = np.deg2rad([0,30,90.00000001,0,90,0]) #Elbow singularity

#q = np.deg2rad([0,30,30,0,0,0]) #Wrist singularity

# Jacobian metrics
J = robot.jacob0(q)
print("Jacobian")
print(J)
print()
#print(np.linalg.inv(J))
#print()

if J.shape[0] == J.shape[1]:
    print("det(J):", np.linalg.det(J))
    print()

U, S, Vt = np.linalg.svd(J)
print("Singular values:")
print(S)

# Plot the robot
robot.teach(q, backend="pyplot3")