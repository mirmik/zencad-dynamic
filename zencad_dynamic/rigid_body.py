from zencad_dynamic.body import body
from zencad.libs.screw import screw

class rigid_body(body):
	def __init__(self, pose, inertia, views=None):
		super().__init__()

		self.speed= screw()
		self.acceleration= screw()

		self.pose = pose
		self.inertia = inertia
		self.views = views if views is not None else [] 

	def add_view(self, view):
		self.views.append(view)

	def update_views(self):
		for v in self.views:
			v.relocate(self.pose)

	def set_speed(self, scr):
		self.speed = scr

	def set_acceleration(self, scr):
		self.acceleration = scr

	def update_globals(self):
		self.reference_inertia = self.inertia.rotate(self.pose)

		moved_inertia = self.reference_inertia.guigens_transform(-self.reference_inertia.radius)
		self.reference_mass_matrix = moved_inertia.to_mass_matrix()	

	def inertia_force(self):
		aspd = self.speed.ang
		mass = self.reference_inertia.mass
		rho = self.reference_inertia.radius
		imat = self.reference_inertia.matrix

		ret = screw(
			lin= - mass * aspd.cross(aspd.cross(rho)),
			ang= - aspd.cross(imat*aspd)
		)

		return ret

	#def inertia_force_in_body_frame(self):
	#	arm = -self.reference_inertia.radius
	#	return self.inertia_force().force_carry(arm)