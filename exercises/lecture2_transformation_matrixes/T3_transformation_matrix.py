from spatialmath import SE3
import numpy as np

theta = 20*np.pi/180 

T = SE3.Rz(theta)
print(T)

T2 = SE3.Trans(5,2,1)
print(T2)