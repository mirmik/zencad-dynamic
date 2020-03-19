import pyservoce
import numpy
import numpy as np
from zencad.libs.screw import screw
import zencad.libs.screw

class constrait:
	def __init__(self, T_comp=0.1, ksi_comp=0.73):
		self.connections = []
		self.lin = []
		self.ang = []

		self.compensation_koeff_position = - (1 / T_comp**2)
		self.compensation_koeff_speed = - 2 * ksi_comp / T_comp

		self.reference_pose = None

	def attach(self,abody, apose, bbody, bpose):
		self.attach_positive_connection(abody, apose)
		self.attach_negative_connection(bbody, bpose)

	def attach_reference(self, body, pose):
		self.attach_positive_connection(body, pose)
		self.reference_pose = body.pose * pose

	def attach_positive_connection(self, body, pose):
		c = constrait_connection(self, body, pose,  1)
		self.connections.append(c)

	def attach_negative_connection(self, body, pose):
		c = constrait_connection(self, body, pose, -1)
		self.connections.append(c)

	def update_globals(self):
		self.self_update_globals()
		for c in self.connections:
			c.update_globals()

		if len(self.connections) == 1:
			carry_arm_0 = self.connections[0].body.pose(self.connections[0].pose.translation())

			pose_error = self.reference_pose.inverse() * self.connections[0].fullpose
			speed_error = self.connections[0].body.speed.kinematic_carry(carry_arm_0)

			position_error = screw.from_trans(pose_error).rotate_by(self.reference_pose).npvec_lin_first()
			speed_error = speed_error.npvec_lin_first()

			position_compensate = np.matmul(self.constrait_matrix, position_error)
			speed_compensate = np.matmul(self.constrait_matrix, speed_error)

			self.closing_compensate_vector = (
				- speed_compensate * self.compensation_koeff_speed 
				- position_compensate * self.compensation_koeff_position 
			)
			#print(self.closing_compensate_vector)

		elif len(self.connections) == 2:
			carry_arm_0 =  self.connections[0].body.pose(self.connections[0].pose.translation())
			carry_arm_1 =  self.connections[1].body.pose(self.connections[1].pose.translation())

			pose_error = self.connections[1].fullpose.inverse() * self.connections[0].fullpose
			speed_error = self.connections[0].body.speed.kinematic_carry(carry_arm_0) - self.connections[1].body.speed.kinematic_carry(carry_arm_1)

			position_error = screw.from_trans(pose_error).rotate_by(self.connections[1].fullpose).npvec_lin_first()			
			speed_error = speed_error.npvec_lin_first()
			
			position_compensate = np.matmul(self.constrait_matrix, position_error)
			speed_compensate = np.matmul(self.constrait_matrix, speed_error)

			self.closing_compensate_vector = ( #numpy.zeros((3))
				- speed_compensate * self.compensation_koeff_speed 
				- position_compensate * self.compensation_koeff_position 
			)

		elif len(self.connections) == 0:
			self.closing_compensate_vector = None

		else:
			raise Exception("Constrait has more then two connections")

class constrait_connection:
	def __init__(self, constrait, body, pose, mul):
		self.constrait = constrait
		self.body = body
		self.pose = pose
		self.mul = mul

	def update_globals(self):
		radius = self.body.pose(self.pose.translation())
		self.rank = self.constrait.rank
		w = self.body.speed.ang.vecmul_matrix()
		
		rcml = self.constrait.constrait_matrix_linear
		rcma = self.constrait.constrait_matrix_angular

		cml = rcml
		cma = rcma - numpy.matmul(rcml, radius.vecmul_matrix())
		h = numpy.matmul(rcml, w * (w * radius))

		self.constrait_matrix = np.concatenate((cml, cma), axis=1) * self.mul
		self.compensate_vector = h * self.mul

		self.fullpose = self.body.pose * self.pose

#class constrait_connection_positive(constrait_connection):
#	def __init__(self, constrait, body, radius):
#		super().__init__(constrait, body, radius)	
#
#	#def constrait_screws(self):
#	#	return self.constrait.constrait_screws()
#
#	def constrait_matrix(self):
#		return self.constrait.half_constrait_matrix(
#			con=self,
#			mul=1)
#
#	def compensate_vector(self):
#		return self.constrait.half_compensate_vector(
#			con=self,
#			mul=1)
#
#class constrait_connection_negative(constrait_connection):
#	def __init__(self, constrait, body, radius):
#		super().__init__(constrait, body, radius)	
#
#	#def constrait_screws(self):
#	#	return [ 
#	#		-scr for scr in self.constrait.constrait_screws()
#	#	]
#
#	def constrait_matrix(self):
#		return self.constrait.half_constrait_matrix(
#			con=self,
#			mul=-1)
#
#	def compensate_vector(self):
#		return self.constrait.half_compensate_vector(
#			con=self,
#			mul=-1)





#class rotator_constrait(constrait):
#	def __init__(self, ax):
#		super().__init__()
#		self.axis = pyservoce.vector3(ax)
#		self.__dbg = 0
#
#	def rank(self):
#		return 5
#
#	def constrait_screws(self):
#		ang = self.axis
#
#		if ang.early(pyservoce.vector3(1,0,0), 0.00001):
#			r = pyservoce.vector3(0,1,0)
#		else:
#			r = pyservoce.vector3(1,0,0)
#
#		f = ang.cross(r).normalize()
#		s = ang.cross(f)
#
#		scrs = [
#			screw(lin=(1,0,0), ang=(0,0,0)),
#			screw(lin=(0,1,0), ang=(0,0,0)),
#			screw(lin=(0,0,1), ang=(0,0,0)),
#			screw(lin=(0,0,0), ang=f),
#			screw(lin=(0,0,0), ang=s),
#		]
#
#		return scrs

class spherical_rotator(constrait):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	def self_update_globals(self):
		self.constrait_matrix_linear = numpy.array([
			[1,0,0],
			[0,1,0],
			[0,0,1]
		])

		self.constrait_matrix_angular = [
			[0,0,0],
			[0,0,0],
			[0,0,0]
		]

		self.constrait_matrix = np.concatenate((self.constrait_matrix_linear, self.constrait_matrix_angular), axis=1)

		self.rank = 3

	#def half_constrait_matrix(self, con, mul):
	#	r = con.body.pose(con.radius)
	#	#r = pyservoce.vector3(0,0,0)
	#	return numpy.matrix([
	#		[1,  0,  0,    0,   r.z, -r.y],
	#		[0,  1,  0,  -r.z,    0,  r.x],
	#		[0,  0,  1,   r.y, -r.x,    0]
	#	]) * mul
#
	#	#return numpy.matrix([
	#	#	[1,  0,  0,    0,   -r.z, r.y],
	#	#	[0,  1,  0,  r.z,    0,  -r.x],
	#	#	[0,  0,  1,   -r.y, r.x,    0]
	#	#]) * mul
#
	#def half_compensate_vector(self, con, mul):
	#	r = con.body.pose(con.radius)
	#	aspd = con.body.speed.ang
	#	vec = aspd.cross(aspd.cross(r))
#
	#	#print(con)
	#	#print(con.body)
	#	#print("aspd",aspd)
	#	#print("r",r)
#
	#	#return numpy.matrix([0,0,0]).transpose()
#
	#	return numpy.matrix([
	#		[vec.x],
	#		[vec.y],
	#		[vec.z]	
	#	]) * mul