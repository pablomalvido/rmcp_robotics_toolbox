import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib
matplotlib.use("TkAgg")

robot = rtb.models.Panda()
print(robot)

Te = robot.fkine(robot.qr)  # forward kinematics (qr is q ready, qz is q at zero...)
print(Te)

Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ik_LM(Tep)         # solve IK
print(sol)

q_pickup = sol[0]
print(robot.fkine(q_pickup))    # FK shows that desired end-effector pose was achieved

qt = rtb.jtraj(robot.qr, q_pickup, 50)
#robot.plot(qt.q, backend='pyplot4') #pyplot backend (simplified visualization)
#robot.plot(qt.q) #Normal robot

robot.teach([0,0,0,0,0,0,0], backend='pyplot4')