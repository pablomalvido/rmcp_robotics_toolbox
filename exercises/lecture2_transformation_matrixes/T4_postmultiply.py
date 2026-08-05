from spatialmath import SE3
import numpy as np

theta = 90*np.pi/180 

T1 = SE3.Rz(theta)
T2 = SE3.Trans(0.2,0,0)
T = T1 * T2
print(T)