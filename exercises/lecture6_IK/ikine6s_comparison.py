import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot,RevoluteMDH,PrismaticMDH
import numpy as np
from spatialmath import SE3
import time

robot = rtb.models.DH.Puma560()

q=[0, np.pi/6, -np.pi/4, 0, np.pi/4, 0]
T = robot.fkine(q)

t_init = time.perf_counter()
sol = robot.ikine_a(T, config="lun") #Same than ikine_6s for PUMA. Internally calls ikine_6s for a specific ikfunc
print("IKINE 6S")
elapsed_ms = (time.perf_counter() - t_init)*1000
print("Computation time: " + str(elapsed_ms) + " ms")
print(sol.q)
print(robot.fkine(sol.q))
print()

t_init = time.perf_counter()
sol_num = robot.ikine_LM(T, mask=[1,1,1,1,1,1], q0=[0,0,0,0,0,0])
print("IKINE Numerical")
elapsed_ms = (time.perf_counter() - t_init)*1000
print("Computation time: " + str(elapsed_ms) + " ms")
print(sol_num.q)
print(robot.fkine(sol_num.q))
print()

print("TARGET")
print(T)

robot.teach(sol.q, backend='pyplot3')