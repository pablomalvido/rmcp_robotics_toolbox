from spatialmath import SE3
import numpy as np

theta = 90*np.pi/180 

T = SE3.Trans(0.2,0,0) * SE3.Rz(theta)
print(T)