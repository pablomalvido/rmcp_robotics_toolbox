import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3

#Defining the robot: SCARA
robot = DHRobot(
    [
        RevoluteMDH(alpha=np.pi,d=-0.5),
        RevoluteMDH(a=0.4),
        RevoluteMDH(a=0.3),
        PrismaticMDH(qlim=[-1,1]), #Limits for the prismatic movement
    ], name="scaraRobot")

q = [0, np.pi/4, 0, 0.3] #Singularity at q2=0
#q = [0, 0.0001, 0, 0.3] #Singularity at q2=0  

# Defining the robot: 3R
# robot = DHRobot(
#     [
#         RevoluteMDH(d=0.615),
#         RevoluteMDH(alpha=np.pi/2, a=0.1),
#         RevoluteMDH(a=0.705),
#     ], name="scaraRobot")
# robot.tool = SE3.Trans(0.2, 0, 0)

# q = [0, 0.5, -1.4] #Singularity at q3=0
# q = [0, 0.5, 0.0001] #Singularity at q3=0

# Jacobian metrics
J = robot.jacob0(q)
Jv = J[:3, :]

if J.shape[0] == J.shape[1]:
    print()
    print("det(J): ", np.linalg.det(J))
    w = np.sqrt(np.linalg.det(J@J.T))
    print()
    print("Manipulability index: ", w)

elif Jv.shape[0] == Jv.shape[1]:
    print()
    print("det(Jv): ", np.linalg.det(Jv))
    w = np.sqrt(np.linalg.det(Jv@Jv.T))
    print()
    print("Manipulability index: ", w)

else: # For computing the SCARA, as it has 4 joints
    print()
    print("det(J): ", np.linalg.det(Jv@Jv.T))
    w = np.sqrt(np.linalg.det(Jv@Jv.T))
    print()
    print("Manipulability index: ", w)

# Plot the robot
robot.plot(q, backend="pyplot3")

# Plot the velocity manipulability ellipsoid
robot.plot_vellipse(q)