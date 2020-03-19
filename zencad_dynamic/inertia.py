import zencad.libs.screw
import pyservoce
from zencad.libs.screw import screw
import numpy

class inertia:
	def __init__(self, mass=1, matrix=pyservoce.matrix33(1,1,1), radius=pyservoce.vector3(0,0,0)):
		self.radius = pyservoce.vector3(radius)
		self.matrix = matrix
		self.invmatrix = self.matrix.inverse()
		self.mass = mass

	def rotate(self, trans):
		#trans = from_pose.inverse() * to_pose
		rot = trans.rotation().to_matrix()
		invrot = rot.transpose()

		#print(trans, self.radius, trans(self.radius))
		return inertia(
			self.mass,
			invrot * self.matrix * rot,
			trans(self.radius)
		)

	def fulltrans(self, trans):
		#trans = from_pose.inverse() * to_pose
		rot = trans.rotation().to_matrix()
		invrot = rot.transpose()
		return inertia(
			self.mass,
			invrot * self.matrix * rot,
			trans(self.radius)
		)

	def transform(self, trans):
		""" Преобразование инерции в другую систему координат.
			trans - оператор преобразования текущей системы в новую.
			Для выполнения операции мы поворачиваем матрицу и меняем радиус вектор.
			Трансляция радиус вектора происходит по закону преобразования точки.
		"""

		rot = trans.rotation().to_matrix()
		invrot = rot.transpose()
		return inertia(
			self.mass,
			invrot * self.matrix * rot,
			trans(self.radius) + trans.translation() 
		)

	def to_mass_matrix(self):
		m = self.mass
		I = self.matrix
		p = self.radius
		
		return numpy.array([
			[     m,      0,      0,      0,  m*p.z, -m*p.y],
			[     0,      m,      0, -m*p.z,      0,  m*p.x],
			[     0,      0,      m,  m*p.y, -m*p.x,      0],
			[     0, -m*p.z,  m*p.y, I[0,0], I[0,1], I[0,2]],
			[ m*p.z,      0, -m*p.x, I[1,0], I[1,1], I[1,2]],
			[-m*p.y,  m*p.x,      0, I[2,0], I[2,1], I[2,2]]
		])

		#return numpy.array([
		#	[     m,      0,      0,      0,  0, -0],
		#	[     0,      m,      0, -0,      0,  0],
		#	[     0,      0,      m,  0, -0,      0],
		#	[     0, -0,  0, I[0,0], I[0,1], I[0,2]],
		#	[ 0,      0, -0, I[1,0], I[1,1], I[1,2]],
		#	[-0,  0,      0, I[2,0], I[2,1], I[2,2]]
		#])

	def koefficient_for(self, sens):
		return (sens.lin * self.mass).length() +  (self.matrix * sens.ang).length() 

	def koefficient_for_with_guigens(self, sens):
		iner = self.guigens_transform(-self.radius)
		#print(self.radius)
		#print(iner)
		return (sens.lin * iner.mass).length() +  (iner.matrix * sens.ang).length() 

	def guigens_transform(self, mov):
		#print(mov.outerprod(mov))
		sqr = mov.length2()
		return inertia(
			matrix = self.matrix \
				+ self.mass \
					* ( pyservoce.matrix33(sqr, sqr, sqr) - mov.outerprod(mov)),
			mass = self.mass,
			radius = self.radius# + mov
		)

	def impulse_to_speed(self, impulse_screw):
		lin = impulse_screw.lin / self.mass
		ang = self.invmatrix * impulse_screw.ang
		return screw(ang=zencad.vector3(ang[0,0], ang[1,0], ang[2,0]), lin=lin)

	def force_to_acceleration(self, fscr):
		return screw(
			ang=self.invmatrix * fscr.ang, 
			lin=fscr.lin / self.mass)

	def acceleration_to_force(self, accscr):
		return screw(
			ang=self.matrix * accscr.ang, 
			lin=self.mass   * accscr.lin)


	def __repr__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.radius
		).split())

def guigens_transform(matrix, mov, mass):
	sqr = mov.length2()
	return matrix \
		+ mass * ( pyservoce.matrix33(sqr, sqr, sqr) - mov.outerprod(mov) )


def complex_inertia(lst):
	cm = zencad.vector3(0,0,0)
	matrix = pyservoce.matrix33()

	mass = sum([ i.mass for i in lst])
	for I in lst:
		cm += I.mass * I.radius
	cm = cm / mass
	
	for I in lst:
		matrix += guigens_transform(I.matrix, cm - I.radius, I.mass)

	return inertia(
		mass, matrix, cm
	)

class inertial_object:
	def __init__(self, unit, radius, mass, Ix, Iy, Iz, Ixy, Ixz, Iyz):
		if not isinstance(radius, pyservoce.vector3):
			raise Exception("radius is not vector3")

		self.unit = unit
		self.radius = radius
		self.mass = mass
		self.global_impulse = screw()
		#print(Ix, Iy, Iz)
		self.matrix = pyservoce.matrix33(
			Ix, Ixy,Ixz,
			Ixy,Iy ,Iyz,
			Ixz,Iyz,Iz 
		)
		#print(self.matrix)
		self.update_globals()

	#def global_inertia(self):
	#	cm = pyservoce.point3(*self.global_pose.translation())
	#	return inertia(mass=self.mass, matrix=self.global_matrix, cm=cm)

	def transformed_matrix(self, trans):
		rot = trans.rotation().to_matrix()
		transrot = rot.transpose()
		return transrot * self.matrix * rot
		#return self.matrix

	def get_rotated_inertia(self, trans):
		#print(trans)
		#print(self.transformed_matrix(trans))
		#print(self.matrix)
		return inertia(
			mass=self.mass, 
			matrix=self.transformed_matrix(trans), 
			radius=trans(self.radius)
		)

	def update_globals(self):
		#self.global_pose = self.unit.global_pose * self.pose
		#self.global_matrix = self.transformed_matrix(self.unit.global_pose)
		pass

	def __repr__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.radius
		).split())

	#def update_global_impulse_with_global_speed(self, global_spdscr):
	#	self.global_impulse = screw(
	#		lin = self.mass * global_spdscr.lin,
	#		ang = self.global_matrix * global_spdscr.ang
	#	)




#def attach_inertia(unit, 
#		radius=pyservoce.vector3(), 
#		mass=1, 
#		Ix=1, Iy=1, Iz=1, Ixy=0, Ixz=0, Iyz=0):
	#unit.inertial_object = zencad.libs.inertia.inertial_object( 
	#	unit=unit, radius=radius, mass=mass, Ix=Ix, Iy=Iy, Iz=Iz, Ixy=Ixy, Ixz=Ixz, Iyz=Iyz)

def attach_inertia(unit, iner):
	unit.inertia = iner