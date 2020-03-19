#!/usr/bin/env

from zencad.libs.screw import screw
import zencad.mbody.kinematic as kinematic

class unit_section:
	def __init__(self, base, kinframe = None):
		self.base = base
		self.group = self.collect_group(self.base)
		self.parent = None
		self.kinframe = kinframe
		if self.kinframe:
			kinframe.linked_unit_section = self

	def restore_tree(self):
		self.childs = []
		def foo(u):
			for c in u.childs:
				if isinstance(u, kinematic_frame):
					u.linked_unit_section.parent = self
					self.childs.append(u.linked_unit_section)
					continue
				foo(c)

		foo(self.unit)

	def collect_group(self):
		ret = []

		def foo(u):
			for c in u.childs:
				if isinstance(u, kinematic_frame):
					continue
				ret.append(u)
				foo(c)

		foo(self.base)
		return ret

	def make_body(self):
		self.iners = []
		
		for u in self.group:
			if hasattr(u, "inertia"):
				trans = self.base.global_pose.inverse() * u.global_pose()
				self.iners.append(u.inertia.transform(trans))

		self.inertia = inertia.complex_inertia(self.iners)
		self.body = rigid_body.rigid_body(pose=self.baseunit, inertia=self.inertia)

	def attach_constrait_connections(self):
		self.connections = []

		if self.kinframe:
			self.connections.append(self.kinframe.output_constrait_connection())
		
		for s in self.childs:
			self.connections.append(s.kinframe.input_constrait_connection())

class kintranslator:
	"""Решает задачу построения динамической модели 
	по кинематическому дерева.
	"""

	FREE_SPACE_MODE = 0
	CONSTRAIT_MODE = 1

	def __init__(self, baseunit, mode=FREE_SPACE_MODE):
		self.baseunit = baseunit
		self.mode = mode

	def build(self):
		self.baseunit.update_pose()
		self.kinframes = kinematic.find_all_kinframes(self.baseunit)

		self.base_section = unit_section(baseunit)
		self.sections = [ self.base_section ]
		for kin in self.kinframes:
			self.sections.append(unit_section(kin.output, kinframe=kin))

		for sect in self.sections:
			sect.make_body()



		if self.mode == self.FREE_SPACE_MODE:
			pass

		elif self.mode == self.CONSTRAIT_MODE:
			c = constraits.close(self.baseunit.)
			self.constraits.append()


		return self.rigids, self.constraits

	def collect_all_kinframe_inertia(self):
		for k in self.kinframes:
			k.collect_inertia()

	def make_constraits(self):
		ret = []
		for k in self.kinframes:
			k.init_constrait()
			ret.append(k.constrait)

	def make_bodies(self):
		ret = []
		for k in self.kinframes:
			k.init_body()
			ret.append(k.constrait)

	def produce_bodies_spdacc(self):
		pass

	def reduce_bodies_spdacc(self):
		pass