from spatialmath import SE3
import numpy as np
import sympy as sp

alpha = sp.symbols('alpha')
a = sp.symbols('a')
d = sp.symbols('d')
theta = sp.symbols('theta')

T = SE3.Rx(alpha) * SE3.Trans(a,0,0) * SE3.Trans(0,0,d) * SE3.Rz(theta)
sp.pprint(T)