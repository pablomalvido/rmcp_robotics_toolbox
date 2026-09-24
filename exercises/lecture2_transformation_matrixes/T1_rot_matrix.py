import math
import numpy as np

t = 90*math.pi/180

rot_Y=np.matrix([[math.cos(t),0, math.sin(t)],
              [0, 1, 0],
              [-math.sin(t),0,math.cos(t)]])

print(rot_Y)