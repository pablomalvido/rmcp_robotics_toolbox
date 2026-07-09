import sympy as sp
from sympy import symbols, cos, sin

t = symbols('t')

rot_Z=sp.Matrix([[cos(t),-sin(t),0],
              [sin(t), cos(t), 0],
              [0,0,1]])

sp.pprint(rot_Z)

t_value = 20*sp.pi/180

evaluated_matrix = rot_Z.subs(t, t_value)

print()
sp.pprint(evaluated_matrix)

print()
sp.pprint(evaluated_matrix.evalf())