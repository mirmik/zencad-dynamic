#!/usr/bin/env python3

import zencad
from zencad import *
import zencad_dynamic

from zencad_dynamic.solver import matrix_solver
from zencad_dynamic.rigid_body import rigid_body


a = rigid_body(translate(10,0,0), zencad_dynamic.inertia(1))

solver = matrix_solver([a],[], gravity=(0,0,1))

while True:
	acc, react = solver.solve()
	print(acc, react)