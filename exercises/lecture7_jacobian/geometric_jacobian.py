from spatialmath import SE3
import numpy as np
import sympy as sp
from sympy import sin, cos

t1 = sp.symbols('t1')
t2 = sp.symbols('t2')
t3 = sp.symbols('t3')
t4 = sp.symbols('t4')
t5 = sp.symbols('t5')
t6 = sp.symbols('t6')

def RotX(alpha):
    return sp.Matrix([
        [1, 0,          0,           0],
        [0, cos(alpha), -sin(alpha), 0],
        [0, sin(alpha),  cos(alpha), 0],
        [0, 0,          0,           1]
    ])

def RotZ(theta):
    return sp.Matrix([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta),  cos(theta), 0, 0],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])

def Trans(x,y,z):
    return sp.Matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def get_transform(alpha=0, a=0, d=0, theta=0):
    return RotX(alpha) * Trans(a,0,0) * Trans(0,0,d) * RotZ(theta)

L1 = 0.327
L2 = 0.225
L3 = 0.25

T00 = get_transform()
T01 = get_transform(d=L1, theta=t1)
T12 = get_transform(alpha=np.pi/2, theta=t2)
T23 = get_transform(a=L2, theta=t3)
T34 = get_transform(alpha=np.pi/2, d=L3, theta=t4)
T45 = get_transform(alpha=-np.pi/2, theta=t5)
T56 = get_transform(alpha=np.pi/2, theta=t6)

T02 = T01 * T12
T03 = T02 * T23
T04 = T03 * T34
T05 = T04 * T45
T06 = T05 * T56

p0 = T00[:3,3]
p1 = T01[:3,3]
p2 = T02[:3,3]
p3 = T03[:3,3]
p4 = T04[:3,3]
p5 = T05[:3,3]
p6 = T06[:3,3]

z0 = T00[:3,2]
z1 = T01[:3,2]
z2 = T02[:3,2]
z3 = T03[:3,2]
z4 = T04[:3,2]
z5 = T05[:3,2]
z6 = T06[:3,2]

Jv1 = sp.Matrix([z1.cross(p6 - p1)])
Jw1 = z1

Jv2 = sp.Matrix([z2.cross(p6 - p2)])
Jw2 = z2

Jv3 = sp.Matrix([z3.cross(p6 - p3)])
Jw3 = z3

Jv4 = sp.Matrix([z4.cross(p6 - p4)])
Jw4 = z4

Jv5 = sp.Matrix([z5.cross(p6 - p5)])
Jw5 = z5

Jv6 = sp.Matrix([z6.cross(p6 - p6)])
Jw6 = z6

Jv = sp.Matrix.hstack(
    Jv1, Jv2, Jv3, Jv4, Jv5, Jv6
)

Jw = sp.Matrix.hstack(
    Jw1, Jw2, Jw3,
    Jw4, Jw5, Jw6
)

J = sp.Matrix.vstack(Jv, Jw)

print("Geometric Jacobian:")
sp.pprint(J)
print()

sp.pprint("Position Jacobian shape: " + str(Jv.shape))
sp.pprint("Position Jacobian q4,q5,q6 columns: " + str(Jv[:,3:])) #All 0s, as q4,q5,q6 are not part of the positional chain
print()

Jv_det_sym = Jv[:,:3].det()
sp.pprint(Jv_det_sym)
print()

Jv_det = Jv_det_sym.subs({t1:0, t2:np.pi/4, t3:np.pi/4}).evalf()
sp.pprint("Determinant Position Jacobian: " + str(Jv_det))

J_det = J.det().subs({t1:0, t2:np.pi/4, t3:np.pi/4, t4:0, t5:0, t6:0})
sp.pprint("Determinant Jacobian: " + str(J_det))