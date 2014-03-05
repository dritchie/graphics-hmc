
local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand = terralib.require("prob.random")
local gl = terralib.require("gl")
local colors = terralib.require("colors")
local ad = terralib.require("ad")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)

----------------------------------

local max = macro(function(a,b)
	return quote
		var result = a
		if b > a then result = b end
	in
		result
	end
end)

local min = macro(function(a,b)
	return quote
		var result = a
		if b < a then result = b end
	in
		result
	end
end)

local perp = macro(function(vec)
	local VecT = vec:gettype()
	return `VecT.stackAlloc(-vec(1), vec(0))
end)

-- Check whether two vectors are on the same line
-- (Uses the z-component of the cross product)
local collinear = macro(function(v1, v2)
	return `v1(0)*v2(1) - v1(1)*v2(0) == 0.0
end)

local terra drawLine(bot: Vec2d, top: Vec2d, width: double, color: Color3d)
	gl.glLineWidth(width)
	gl.glColor3d(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_LINES())
	gl.glVertex2d(bot(0), bot(1))
	gl.glVertex2d(top(0), top(1))
	gl.glEnd()
end

local terra drawBar(bot: Vec2d, top: Vec2d, width: double, color: Color3d)
	var dir = top - bot
	dir:normalize()
	var w = width / 2.0
	var perp = Vec2d.stackAlloc(-dir(1), dir(0))
	var p0 = bot - w*perp
	var p1 = bot + w*perp
	var p2 = top + w*perp
	var p3 = top - w*perp
	-- Draw filled quad
	gl.glColor3d(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_QUADS())
	gl.glVertex2d(p0(0), p0(1))
	gl.glVertex2d(p1(0), p1(1))
	gl.glVertex2d(p2(0), p2(1))
	gl.glVertex2d(p3(0), p3(1))
	gl.glEnd()
	-- Draw black outline
	gl.glLineWidth(2.0)
	gl.glColor3d(0.0, 0.0, 0.0)
	gl.glBegin(gl.mGL_LINE_LOOP())
	gl.glVertex2d(p0(0), p0(1))
	gl.glVertex2d(p1(0), p1(1))
	gl.glVertex2d(p2(0), p2(1))
	gl.glVertex2d(p3(0), p3(1))
	gl.glEnd()
end

-- A force is:
--    * A vector indicating the direction and magnitude of the force
--    * A position indicating where the force is applied
--    * A number indicating the number of degrees of freedom in the force
--      (i.e. is it constrained to point in a particular direction, or is it free?)
local forceColor = colors.Tableau10.Red
local forceLineWidth = 3.0
local Force = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct ForceT { vec: Vec2, pos: Vec2, dof: uint }
	-- Compute torque about a particular center of rotation
	terra ForceT:torque(centerOfRotation: Vec2)
		var d = self.pos - centerOfRotation
		-- C.printf("f: (%g, %g), d: (%g, %g)\n", 
		-- 	ad.val(self.vec(0)), ad.val(self.vec(1)), ad.val(d(0)), ad.val(d(1)))
		return d(0)*self.vec(1) - d(1)*self.vec(0)
	end
	-- Check for collocation with another force
	terra ForceT:isCollocatedWith(f: &ForceT)
		return self.pos == f.pos
	end
	-- Add another force into this one (in place)
	-- Assumes that f is collocated with this force
	terra ForceT:combineWith(f: &ForceT)
		if self.dof == 1 and
			(f.dof == 2 or (not collinear(self.vec, f.vec))) then
			self.dof = 2
		end
		self.vec = self.vec + f.vec
	end
	-- Attempt to add another force into this one (in place)
	-- Return true if the add was successful (i.e. the two forces
	--    are collocated and can be added), false otherwise.
	terra ForceT:tryCombiningWith(f: &ForceT)
		if self:isCollocatedWith(f) then
			self:combineWith(f)
			return true
		else return false end
	end
	if real == double then
		terra ForceT:draw(scale: double)
			var endpoint = self.pos + scale*self.vec
			drawLine(self.pos, endpoint, forceLineWidth, Color3d.stackAlloc([forceColor]))
		end
	end
	return ForceT
end)


-------------------------------------------------------------------------------
---                              Objects                                    ---
-------------------------------------------------------------------------------

-- RigidObjects can have forces applied to them and know how to calculate
--    their force and torque residuals
local RigidObject = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local ForceT = Force(real)
	local struct RigidObjectT
	{
		forces: Vector(ForceT),
		active: bool,
		affectedByGravity: bool,
		visible: bool
	}
	terra RigidObjectT:__construct(isActive: bool, affectedByGravity: bool, visible: bool) : {}
		self.active = isActive
		self.affectedByGravity = affectedByGravity
		self.visible = visible
		m.init(self.forces)
	end
	terra RigidObjectT:__destruct()
		m.destruct(self.forces)
	end
	terra RigidObjectT:__copy(other: &RigidObjectT)
		self.active = other.active
		self.affectedByGravity = other.affectedByGravity
		self.visible = other.visible
		self.forces = m.copy(other.forces)
	end
	terra RigidObjectT:centerOfMass() : Vec2
		util.fatalError("centerOfMass left unimplemented in RigidObject subclass\n")
	end
	inheritance.virtual(RigidObjectT, "centerOfMass")
	-- Create a heap-allocated dynamic copy
	inheritance.purevirtual(RigidObjectT, "newcopy", {}->{&RigidObjectT})
	-- This method checks to see if f is collocated with any other force, and if so,
	--    it combines them.
	-- This enforces the invariant that all forces are applied at unique locations.
	-- (This is important for getCentersOfRotation to work properly)
	terra RigidObjectT:applyForce(f: ForceT)
		for i=0,self.forces.size do
			var success = self.forces(i):tryCombiningWith(&f)
			if success then return end
		end
		self.forces:push(f)
	end
	-- Retrieve some number of points on the object that are valid centers of
	--    rotation for torque calculations
	-- Default is to return all the points of force application, but subclasses
	--    can override to do different things
	-- IMPORTANT!: This method can return fewer points than requested. Be sure
	--    to check its output.
	terra RigidObjectT:getCentersOfRotation(num: uint, output: &Vector(Vec2)) : bool
		for i=0,min(num,self.forces.size) do
			output:push(self.forces(i).pos)
		end
		-- output:push(self.forces:back().pos)
		return num == output.size
	end
	inheritance.virtual(RigidObjectT, "getCentersOfRotation")
	-- Calculate force and torque residuals
	terra RigidObjectT:calculateResiduals(fresOut: &Vec2, tresOut: &Vector(real))
		-- Inactive objects have zero residual
		if not self.active then
			return real(0.0), real(0.0)
		else
			-- Figure out how many force degrees of freedom we have
			var numDOF = 0
			for i=0,self.forces.size do
				numDOF = numDOF + self.forces(i).dof
			end
			-- Figure out how many force equations we have
			-- If all forces are 1-DOF and act along the same line, then we have 1
			-- Otherwise, we have 2
			var numForceEqns = self.forces(0).dof
			if numForceEqns == 1 then
				for i=1,self.forces.size do
					var f = self.forces:getPointer(i)
					if f.dof > 1 or not collinear(f.vec, self.forces(0).vec) then
						numForceEqns = 2
						break
					end
				end
			end
			-- We need (numDOF - numForceEqns) torque equations to have a fully-determined
			--    system. And we always have at least one torque equation, just to be
			--    sure that we satisfy static equilibrium. This may make some configurations
			--    over-determined. That's fine--the system should (in theory) try to avoid those.
			var numTorqueEqns = max(numDOF - numForceEqns, 1)
			-- Get the centers of rotation we'll use for torque calculations
			var cor = [Vector(Vec2)].stackAlloc()
			if not self:getCentersOfRotation(numTorqueEqns, &cor) then
				util.fatalError("RigidObject - Was not given enough centers of rotation to enforce torque equilibrium\n")
			end
			-- Compute force residual
			var totalf = Vec2.stackAlloc(0.0)
			for i=0,self.forces.size do
				totalf = totalf + self.forces(i).vec
			end
			@fresOut = totalf
			-- Compute torque residual
			var tsumsq = real(0.0)
			for i=0,cor.size do
				var indvidualResidual = real(0.0)
				for j=0,self.forces.size do
					var t = self.forces(j):torque(cor(i))
					indvidualResidual = indvidualResidual + t
				end
				tresOut:push(indvidualResidual)
			end
		end
	end
	terra RigidObjectT:mass() : real
		return 0.0
	end
	inheritance.virtual(RigidObjectT, "mass")
	-- OpenGL drawing
	if real == double then
		inheritance.purevirtual(RigidObjectT, "drawImpl", {}->{})
		terra RigidObjectT:draw()
			if self.visible then
				self:drawImpl()
			end
		end
	end
	m.addConstructors(RigidObjectT)
	return RigidObjectT
end)


-- A Beam is a rigid bar of finite area
local beamColor = colors.Tableau10.Blue
local beamDensity = 0.1
local Beam = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local polar2rect = macro(function(r, theta)
		return `Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end)
	local RigidObjectT = RigidObject(real)
	local struct BeamT
	{
		endpoints: Vec2[2],
		width: real,
		density: real,
		perp: Vec2
	}
	if real == double then
		BeamT.entries:insert({field="color", type=Color3d})
	end
	inheritance.dynamicExtend(RigidObjectT, BeamT)
	terra BeamT:__construct(bot: Vec2, top: Vec2, w: real, active: bool, visible: bool) : {}
		RigidObjectT.__construct(self, active, true, visible)
		self.endpoints[0] = bot
		self.endpoints[1] = top
		self.width = w
		self.density = beamDensity
		[util.optionally(real==double, function() return quote
			self.color = Color3d.stackAlloc([beamColor])
		end end)]
		var axis = top - bot; axis:normalize()
		self.perp = perp(axis)

	end
	-- Beams are active and visible by default
	terra BeamT:__construct(bot: Vec2, top: Vec2, w: real) : {}
		self:__construct(bot, top, w, true, true)
	end
	-- Constructor in terms of alternative parameters
	BeamT.methods.fromCenterLengthAngle = terra(c: Vec2, len: real, ang: real, w: real, active: bool, visible: bool) : &BeamT
		var halflen = 0.5*len
		var axis = polar2rect(halflen, ang)
		return BeamT.heapAlloc(c - axis, c + axis, w, active, visible)
	end
	BeamT.methods.fromCenterLengthAngle:adddefinition((terra(c: Vec2, len: real, ang: real, w: real) : &BeamT
		return BeamT.fromCenterLengthAngle(c, len, ang, w, true, true)
	end):getdefinitions()[1])
	-- (Inherit parent destructor)
	terra BeamT:__copy(other: &BeamT)
		RigidObjectT.__copy(self, other)
		self.endpoints[0] = other.endpoints[0]
		self.endpoints[1] = other.endpoints[1]
		self.width = other.width
		self.density = other.density
		[util.optionally(real==double, function() return quote
			self.color = other.color
		end end)]
	end
	terra BeamT:newcopy() : &RigidObjectT
		var newbeam = m.new(BeamT)
		newbeam:__copy(self)
		return newbeam
	end
	inheritance.virtual(BeamT, "newcopy")
	terra BeamT:centerOfMass() : Vec2
		return 0.5 * (self.endpoints[0] + self.endpoints[1])
	end
	inheritance.virtual(BeamT, "centerOfMass")
	-- TODO: Uncomment and finish this if it seems like we actually need it.
	-- -- Beams can generate arbitrarily many centers of rotation by randomly
	-- --    sampling inside their bounds
	-- terra BeamT:getCentersOfRotation(num: uint, output: &Vector(Vec2)) : {}
	-- 	RigidObjectT.methods.getCentersOfRotation(self, num, output)
	-- 	for i=output.size,num do
	-- 		var dir = top - bot
	-- 		dir:normalize()
	-- 		var w = width / 2.0
	-- 		var perp = Vec2d.stackAlloc(dir(1), dir(0))
	-- 	end
	-- 	return true
	-- end
	-- inheritance.virtual(BeamT, "getCentersOfRotation")
	terra BeamT:mass() : real
		return (self.endpoints[1] - self.endpoints[0]):norm() * self.width * self.density
	end
	inheritance.virtual(BeamT, "mass")
	-- Selecting corners of the beam
	BeamT.corner = templatize(function(self, index)
		if index == 0 then
			return `self.endpoints[0] - 0.5*self.width*self.perp
		elseif index == 1 then
			return `self.endpoints[0] + 0.5*self.width*self.perp
		elseif index == 2 then
			return `self.endpoints[1] + 0.5*self.width*self.perp
		elseif index == 3 then
			return `self.endpoints[1] - 0.5*self.width*self.perp
		else
			error("BeamT.corner: index must be 0, 1, 2, or 3")
		end
	end)
	terra BeamT:corner(index: uint)
		if index == 0 then
			return [BeamT.corner(self, 0)]
		elseif index == 1 then
			return [BeamT.corner(self, 1)]
		elseif index == 2 then
			return [BeamT.corner(self, 2)]
		elseif index == 3 then
			return [BeamT.corner(self, 3)]
		else
			util.fatalError("BeamT:corner: index must be 0, 1, 2, or 3")
		end
	end
	-- OpenGL drawing code
	if real == double then
		terra BeamT:drawImpl() : {}
			drawBar(self.endpoints[0], self.endpoints[1], self.width, self.color)
		end
		inheritance.virtual(BeamT, "drawImpl")
	end
	m.addConstructors(BeamT)
	return BeamT
end)


-- CableProxy is just an inactive Beam
-- (This is a proxy for rendering)
local cableColor = colors.Black
local CableProxy = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local BeamT = Beam(real)
	return terra(bot: Vec2, top: Vec2, w: real)
		var beam = BeamT.heapAlloc(bot, top, w, false, true)
		beam.affectedByGravity = false
		[util.optionally(real==double, function() return quote
			beam.color = Color3d.stackAlloc([cableColor])
		end end)]
		return beam
	end
end)


-- Ground is just an inactive Beam
local groundTallness = `10000.0
local groundColor = colors.Tableau10.Brown
local Ground = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local BeamT = Beam(real)
	return terra(groundHeight: real, left: real, right: real)
		var midx = 0.5*(left + right)
		var top = Vec2.stackAlloc(midx, groundHeight)
		var bot = Vec2.stackAlloc(midx, groundHeight - groundTallness)
		var w = right - left
		var beam = BeamT.heapAlloc(bot, top, w, false, true)
		beam.affectedByGravity = false
		[util.optionally(real==double, function() return quote
			beam.color = Color3d.stackAlloc([groundColor])
		end end)]
		return beam
	end
end)


-- A RigidScene is a collection of RigidObjects
local RigidScene = templatize(function(real)
	local RigidObjectT = RigidObject(real)
	local struct RigidSceneT
	{
		width: real,
		height: real,
		objects: Vector(&RigidObjectT)
	}
	terra RigidSceneT:__construct(w: real, h: real) : {}
		self.width = w
		self.height = h
		m.init(self.objects)
	end
	terra RigidSceneT:__construct() : {}
		self:__construct(0.0, 0.0)
	end
	terra RigidSceneT:__copy(other: &RigidSceneT)
		self:__construct(other.width, other.height)
		for i=0,other.objects.size do
			self.objects:push(other.objects(i):newcopy())
		end
	end
	terra RigidSceneT:__destruct()
		self.objects:clearAndDelete()
		m.destruct(self.objects)
	end
	-- OpenGL drawing code
	if real == double then
		terra RigidSceneT:draw(forceScale: double)
			gl.glMatrixMode(gl.mGL_PROJECTION())
			gl.glLoadIdentity()
			gl.gluOrtho2D(0, self.width, 0, self.height)
			gl.glMatrixMode(gl.mGL_MODELVIEW())
			gl.glLoadIdentity()
			-- Pass 1: Draw objects
			for i=0,self.objects.size do
				self.objects(i):draw()
			end
			-- Pass 2: Draw forces
			if forceScale > 0.0 then
				for i=0,self.objects.size do
					-- Also draw forces
					for j=0,self.objects(i).forces.size do
						self.objects(i).forces(j):draw(forceScale)
					end
				end
			end
		end
	end
	m.addConstructors(RigidSceneT)
	return RigidSceneT
end)


-------------------------------------------------------------------------------
---                            Connections                                  ---
-------------------------------------------------------------------------------

local function Connections()
	local forcePriorMean = 0.0
	-- local forcePriorMean = 100.0
	local forcePriorVariance = 100000000000.0
	-- local forcePriorVariance = 200.0
	-- local forcePriorVariance = 100.0

	local Vec2 = Vec(real, 2)
	local ForceT = Force(real)
	local BeamT = Beam(real)

	-- Generate a 1-DOF force along a particular direction
	local gen1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=0.0, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate an unbounded 1-DOF force along a particular direction
	local genUnbounded1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a completely unconstrained 2-DOF force
	local genUnconstrained2DForce = macro(function(pos)
		return quote
			var x = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0}) 
			var y = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0})
		in
			ForceT { Vec2.stackAlloc(x, y), pos, 2}
		end
	end)

	----------------------------------

	-- RigidConnections transmit forces between one or more RigidObjects
	local struct RigidConnection {}
	-- Have to separate interface and implementation for "applyForces" because
	--    (as of now) pfn's can't be virtual.
	inheritance.purevirtual(RigidConnection, "applyForcesImpl", {}->{})
	terra RigidConnection:applyForces()
		self:applyForcesImpl()
	end
	RigidConnection.methods.applyForces = pmethod(RigidConnection.methods.applyForces)

	-- A Hinge applies a 2-DOF force to a Beam
	-- It can be connected to one or more Beams
	local struct Hinge
	{
		beams: Vector(&BeamT),
		location: Vec2
	}
	inheritance.dynamicExtend(RigidConnection, Hinge)
	terra Hinge:__construct(loc: Vec2)
		m.init(self.beams)
		self.location = loc
	end
	terra Hinge:__destruct()
		m.destruct(self.beams)
	end
	terra Hinge:addBeam(beam: &BeamT) self.beams:push(beam) end
	terra Hinge:applyForcesImpl() : {}
		for i=0,self.beams.size do
			if self.beams(i).active then
				self.beams(i):applyForce(genUnconstrained2DForce(self.location))
			end
		end
	end
	inheritance.virtual(Hinge, "applyForcesImpl")
	m.addConstructors(Hinge)

	-- A Cable connects two Beams, applying symmetric 1-DOF tensile forces to each.
	local struct Cable
	{
		endpoints: Vec2[2],
		beams: (&BeamT)[2],
		width: real
	}
	inheritance.dynamicExtend(RigidConnection, Cable)
	terra Cable:__construct(ep1: Vec2, ep2: Vec2, beam1: &BeamT, beam2: &BeamT, w: real) : {}
		self.endpoints[0] = ep1
		self.endpoints[1] = ep2
		self.beams[0] = beam1
		self.beams[1] = beam2
		self.width = w
	end
	terra Cable:createProxy()
		return [CableProxy(real)](self.endpoints[0], self.endpoints[1], self.width)
	end
	terra Cable:otherEndpoint(endpoint: uint)
		return (endpoint + 1) % 2
	end
	terra Cable:directionTowards(endpoint: uint)
		var vec = self.endpoints[endpoint] - self.endpoints[self:otherEndpoint(endpoint)]
		vec:normalize()
		return vec
	end
	terra Cable:directionAwayFrom(endpoint: uint)
		return -self:directionTowards(endpoint)
	end
	terra Cable:applyForcesImpl() : {}
		-- Verify that at least one of the attached Beams is active
		if self.beams[0].active or self.beams[1].active then
			-- Generate a 1D tensile force pulling away from the first endpoint
			var f = gen1DForce(self.endpoints[0], self:directionAwayFrom(0))
			-- Apply, if this endpoint Beam is active
			if self.beams[0].active then
				self.beams[0]:applyForce(f)
			end
			-- If the other endpoint beam is active, then flip the force
			--    direction and apply it there
			if self.beams[1].active then
				f.vec = -f.vec
				f.pos = self.endpoints[1]
				self.beams[1]:applyForce(f)
			end
		end
	end
	inheritance.virtual(Cable, "applyForcesImpl")
	m.addConstructors(Cable)

	-- A Stacking connects two Beams (one atop the other)
	-- It applies normal forces at the contact vertices
	-- Assumes the Beams are axis-aligned and that endpoints[0]
	--    is below endpoints[1]
	local struct Stacking
	{
		bottom: &BeamT,
		top: &BeamT
	}
	inheritance.dynamicExtend(RigidConnection, Stacking)
	terra Stacking:__construct(bot: &BeamT, top: &BeamT)
		self.bottom = bot
		self.top = top
	end
	terra Stacking:applyForcesImpl() : {}
		var contactY = self.bottom.endpoints[1](1)
		var botMinX = self.bottom.endpoints[1](0) - self.bottom.width/2.0
		var botMaxX = self.bottom.endpoints[1](0) + self.bottom.width/2.0
		var topMinX = self.top.endpoints[0](0) - self.top.width/2.0
		var topMaxX = self.top.endpoints[0](0) + self.top.width/2.0
		var contactX1 = ad.math.fmax(botMinX, topMinX)
		var contactX2 = ad.math.fmin(botMaxX, topMaxX)
		var up = Vec2.stackAlloc(0.0, 1.0)
		if self.bottom.active then
			self.bottom:applyForce(gen1DForce(Vec2.stackAlloc(contactX1, contactY), -up))
			self.bottom:applyForce(gen1DForce(Vec2.stackAlloc(contactX2, contactY), -up))
		end
		if self.top.active then
			self.top:applyForce(gen1DForce(Vec2.stackAlloc(contactX1, contactY), up))
			self.top:applyForce(gen1DForce(Vec2.stackAlloc(contactX2, contactY), up))
		end
	end
	inheritance.virtual(Stacking, "applyForcesImpl")
	m.addConstructors(Stacking)


	-- A Weld connects two Beams.
	-- One Beam is 'attached' to the other one, which is the 'base'
	-- The connection happens at one edge of the 'attached'
	-- It applies a force (that can be either positive or negative) normal
	--    to the contact surface at both of the edge contact vertices.
	local struct Weld
	{
		base: &BeamT,
		attached: &BeamT,
		contactPoints: Vec2[2]
	}
	inheritance.dynamicExtend(RigidConnection, Weld)
	terra Weld:__construct(base: &BeamT, attached: &BeamT, cp1: uint, cp2: uint)
		self.base = base
		self.attached = attached
		self.contactPoints[0] = attached:corner(cp1)
		self.contactPoints[1] = attached:corner(cp2)
	end
	terra Weld:applyForcesImpl() : {}
		-- Compute the normal vector to the contact edge
		var contactEdge = self.contactPoints[1] - self.contactPoints[0]
		contactEdge:normalize()
		var normal = perp(contactEdge)

		-- Apply contact forces
		if self.base.active then
			self.base:applyForce(genUnbounded1DForce(self.contactPoints[0], normal))
			self.base:applyForce(genUnbounded1DForce(self.contactPoints[1], normal))
		end
		if self.attached.active then
			self.attached:applyForce(genUnbounded1DForce(self.contactPoints[0], normal))
			self.attached:applyForce(genUnbounded1DForce(self.contactPoints[1], normal))
		end
	end
	inheritance.virtual(Weld, "applyForcesImpl")
	m.addConstructors(Weld)


	return 
	{
		RigidConnection = RigidConnection,
		Hinge = Hinge,
		Cable = Cable,
		Stacking = Stacking,
		Weld = Weld
	}

end



return
{
	Force = Force,
	RigidObject = RigidObject,
	Beam = Beam,
	Ground = Ground,
	RigidScene = RigidScene,
	Connections = Connections,
	perp = perp
}



