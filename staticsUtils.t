
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

local terra drawQuad(p0: Vec2d, p1: Vec2d, p2: Vec2d, p3: Vec2d, color: Color3d)
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

	terra RigidObjectT:disable()
		self.active = false
		self.affectedByGravity = false
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
			var normconst = 1.0/cor.size
			var tsumsq = real(0.0)
			for i=0,cor.size do
				var indvidualResidual = real(0.0)
				for j=0,self.forces.size do
					var t = self.forces(j):torque(cor(i))
					indvidualResidual = indvidualResidual + t
				end
				tresOut:push(normconst*indvidualResidual)
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
local beamDefaultColor = colors.Tableau10.Blue
local beamDefaultDensity = 0.1
local Beam = templatize(function(real)

	local Vec2 = Vec(real, 2)
	local polar2rect = macro(function(r, theta)
		return `Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end)
	local RigidObjectT = RigidObject(real)

	local struct BeamT
	{
		bot1: Vec2,
		bot2: Vec2,
		top1: Vec2,
		top2: Vec2,
		density: real
	}
	if real == double then
		BeamT.entries:insert({field="color", type=Color3d})
	end
	inheritance.dynamicExtend(RigidObjectT, BeamT)

	-- Construct from four points
	terra BeamT:__construct(b1: Vec2, b2: Vec2, t1: Vec2, t2: Vec2) : {}
		-- Default to active, affected by gravity, and visible
		RigidObjectT.__construct(self, true, true, true)
		self.bot1 = b1
		self.bot2 = b2
		self.top1 = t1
		self.top2 = t2
		self.density = beamDefaultDensity
		[util.optionally(real == double, function() return quote
			self.color = Color3d.stackAlloc([beamDefaultColor])
		end end)]
	end

	-- Construct from two endpoints and a width (rectangular only)
	terra BeamT:__construct(b: Vec2, t: Vec2, w: real) : {}
		var axis = t - b; axis:normalize()
		var whalf = 0.5*w
		var perpvec = whalf*perp(axis)
		self:__construct(b - perpvec, b + perpvec, t - perpvec, t + perpvec)
	end

	-- Construct from a center, length, width, and angle (rectangular only)
	terra BeamT:__construct(c: Vec2, l: real, w: real, a: real) : {}
		var lenhalf = 0.5*l
		var axis = polar2rect(lenhalf, a)
		self:__construct(c - axis, c + axis, w)
	end

	-- (Inherit parent destructor)

	terra BeamT:__copy(other: &BeamT)
		RigidObjectT.__copy(self, other)
		self.bot1 = other.bot1
		self.bot2 = other.bot2
		self.top1 = other.top1
		self.top2 = other.top2
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

	-- Generate a bunch of points uniformly space along the length of this beam
	terra BeamT:getCentersOfRotation(num: uint, output: &Vector(Vec2)) : bool
		var e1 = self:endpoint(0)
		var e2 = self:endpoint(1)
		for i=0,num do
			var t = (i+0.5)/num
			var cor = (1.0-t)*e1 + t*e2
			output:push(cor)
		end
		return true
	end
	inheritance.virtual(BeamT, "getCentersOfRotation")

	terra BeamT:centerOfMass() : Vec2
		return 0.25 * (self.bot1 + self.bot2 + self.top1 + self.top2)
	end
	inheritance.virtual(BeamT, "centerOfMass")

	terra BeamT:mass() : real
		-- 1/2 * || AC x BD ||
		var ac = self.top2 - self.bot1
		var bd = self.top1 - self.bot2
		var cross = ac(0)*bd(1) - ac(1)*bd(0)
		return 0.5 * ad.math.sqrt(cross*cross) * self.density
	end
	inheritance.virtual(BeamT, "mass")

	-- Selecting endpoints of the beam
	BeamT.endpoint = templatize(function(self, index)
		if index == 0 then
			return `0.5*(self.bot1 + self.bot2)
		elseif index == 1 then
			return `0.5*(self.top1 + self.top2)
		else
			error("BeamT.endpoint: index must be 0, 1, 2, or 3")
		end
	end)
	terra BeamT:endpoint(index: uint)
		if index == 0 then
			return [BeamT.endpoint(self, 0)]
		elseif index == 1 then
			return [BeamT.endpoint(self, 1)]
		else
			util.fatalError("BeamT.endpoint: index must be 0, 1, 2, or 3")
		end
	end

	-- Selecting corners of the beam
	BeamT.corner = templatize(function(self, index)
		if index == 0 then
			return `self.bot1
		elseif index == 1 then
			return `self.bot2
		elseif index == 2 then
			return `self.top2
		elseif index == 3 then
			return `self.top1
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
			drawQuad(self.bot1, self.bot2, self.top2, self.top1, self.color)
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
		var beam = BeamT.heapAlloc(bot, top, w)
		beam:disable()
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
		var beam = BeamT.heapAlloc(bot, top, w)
		beam:disable()
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
	local forcePriorVariance = 100000000000.0
	-- local forcePriorVariance = 100.0

	local Vec2 = Vec(real, 2)
	local ForceT = Force(real)
	local RigidObjectT = RigidObject(real)
	local BeamT = Beam(real)

	-- Generate a 1-DOF force along a particular direction
	local gen1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a nonnegative 1-DOF force along a particular direction
	local genNonNegative1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=0.0, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a bounded 1-DOF force along a particular direction
	local genBounded1DForce = macro(function(pos, dir, boundMag)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=-boundMag, upperBound=boundMag, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a bounded, nonnegative 1-DOF force along a particular direction
	local genBoundedNonNegative1DForce = macro(function(pos, dir, boundMag)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=0.0, upperBound=boundMag, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a 2-DOF force
	local gen2DForce = macro(function(pos)
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


	-- A Hinge applies a 2-DOF force to an object
	-- It can be connected to one or more objects
	-- NOTE: Deprecated, not sure if this is even physically meaningful...
	local struct Hinge
	{
		objs: Vector(&RigidObjectT),
		location: Vec2
	}
	inheritance.dynamicExtend(RigidConnection, Hinge)

	terra Hinge:__construct(loc: Vec2)
		m.init(self.objs)
		self.location = loc
	end

	terra Hinge:__destruct()
		m.destruct(self.objs)
	end

	terra Hinge:addObj(obj: &RigidObjectT) self.objs:push(obj) end

	terra Hinge:applyForcesImpl() : {}
		for i=0,self.objs.size do
			if self.objs(i).active then
				self.objs(i):applyForce(gen2DForce(self.location))
			end
		end
	end
	inheritance.virtual(Hinge, "applyForcesImpl")

	m.addConstructors(Hinge)


	-- A Cable connects two objects, applying symmetric 1-DOF tensile forces to each.
	local struct Cable
	{
		endpoints: Vec2[2],
		objs: (&RigidObjectT)[2],
		width: real
	}
	inheritance.dynamicExtend(RigidConnection, Cable)

	terra Cable:__construct(ep1: Vec2, ep2: Vec2, obj1: &RigidObjectT, obj2: &RigidObjectT, w: real) : {}
		self.endpoints[0] = ep1
		self.endpoints[1] = ep2
		self.objs[0] = obj1
		self.objs[1] = obj2
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
		-- Verify that at least one of the attached objects is active
		if self.objs[0].active or self.objs[1].active then
			-- Generate a 1D tensile force pulling away from the first endpoint
			var f = genNonNegative1DForce(self.endpoints[0], self:directionAwayFrom(0))
			-- Apply, if this endpoint object is active
			if self.objs[0].active then
				self.objs[0]:applyForce(f)
			end
			-- If the other endpoint object is active, then flip the force
			--    direction and apply it there
			if self.objs[1].active then
				f.vec = -f.vec
				f.pos = self.endpoints[1]
				self.objs[1]:applyForce(f)
			end
		end
	end
	inheritance.virtual(Cable, "applyForcesImpl")

	m.addConstructors(Cable)


	-- A Stacking connects two Beams (one atop the other)
	-- It applies normal forces at the contact vertices
	-- Assumes the Beams are axis-aligned and that endpoints[0]
	--    is below endpoints[1]
	-- NOTE: Deprecated, use FrictionalContact instead
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
		var contactY = [BeamT.endpoint(`self.bottom, 1)](1)
		var botMinX = ad.math.fmin([BeamT.corner(`self.bottom, 0)](0), [BeamT.corner(`self.bottom, 1)](0))
		var botMaxX = ad.math.fmax([BeamT.corner(`self.bottom, 0)](0), [BeamT.corner(`self.bottom, 1)](0))
		var topMinX = ad.math.fmin([BeamT.corner(`self.top, 0)](0), [BeamT.corner(`self.top, 1)](0))
		var topMaxX = ad.math.fmax([BeamT.corner(`self.top, 0)](0), [BeamT.corner(`self.top, 1)](0))
		var contactX1 = ad.math.fmax(botMinX, topMinX)
		var contactX2 = ad.math.fmin(botMaxX, topMaxX)
		var up = Vec2.stackAlloc(0.0, 1.0)
		if self.bottom.active then
			self.bottom:applyForce(genNonNegative1DForce(Vec2.stackAlloc(contactX1, contactY), -up))
			self.bottom:applyForce(genNonNegative1DForce(Vec2.stackAlloc(contactX2, contactY), -up))
		end
		if self.top.active then
			self.top:applyForce(genNonNegative1DForce(Vec2.stackAlloc(contactX1, contactY), up))
			self.top:applyForce(genNonNegative1DForce(Vec2.stackAlloc(contactX2, contactY), up))
		end
	end
	inheritance.virtual(Stacking, "applyForcesImpl")

	m.addConstructors(Stacking)


	-- A Weld connects two Beams.
	-- One Beam is 'attached' to the other one, which is the 'base'
	-- The connection happens at one edge of the 'attached'
	-- It applies a force (that can be either positive or negative) normal
	--    to the contact surface at both of the edge contact vertices.
	-- NOTE: Deprecated, use NailJoint instead
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
			self.base:applyForce(gen1DForce(self.contactPoints[0], normal))
			self.base:applyForce(gen1DForce(self.contactPoints[1], normal))
		end
		if self.attached.active then
			self.attached:applyForce(gen1DForce(self.contactPoints[0], normal))
			self.attached:applyForce(gen1DForce(self.contactPoints[1], normal))
		end
	end
	inheritance.virtual(Weld, "applyForcesImpl")

	m.addConstructors(Weld)


	-- A FrictionalContact connects two objects at some point with some contact normal
	-- It applies an unbounded compressive force along the normal direction
	-- It applies a friction-bounded force along the tangent direction
	-- The normal is assumed to be the normal for object 1; the negative will be used for object 2
	local defaultFrictionCoeff = 0.5
	local struct FrictionalContact
	{
		objs: (&RigidObjectT)[2],
		contactPoint: Vec2,
		contactNormal: Vec2,
		contactTangent: Vec2,
		frictionCoeff: real
	}
	inheritance.dynamicExtend(RigidConnection, FrictionalContact)

	terra FrictionalContact:__construct(o1: &RigidObjectT, o2: &RigidObjectT, cp: Vec2, cn: Vec2) : {}
		self.objs[0] = o1
		self.objs[1] = o2
		self.contactPoint = cp
		self.contactNormal = cn
		self.contactTangent = perp(cn)
		self.frictionCoeff = defaultFrictionCoeff
	end

	-- (Convenience method)
	-- Connect two Beams under frictional contact
	-- Connect beam1 to beam2 along the edge between corners ci1 anc ci2 of beam1
	-- This will generate two FrictionalContact objects
	FrictionalContact.methods.makeBeamContacts = terra(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint)
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0 then
			normal = -normal
		end
		var fc1 = FrictionalContact.heapAlloc(beam1, beam2, cp1, normal)
		var fc2 = FrictionalContact.heapAlloc(beam1, beam2, cp2, normal)
		return fc1, fc2
	end

	terra FrictionalContact:applyForcesImpl() : {}
		var normalForce : ForceT
		var tangentForce : ForceT
		if self.objs[0].active or self.objs[1].active then
			normalForce = genNonNegative1DForce(self.contactPoint, self.contactNormal)
			tangentForce = genBounded1DForce(self.contactPoint, self.contactTangent, self.frictionCoeff*normalForce.vec:norm())
		end
		if self.objs[0].active then
			self.objs[0]:applyForce(normalForce)
			self.objs[0]:applyForce(tangentForce)
		end
		if self.objs[1].active then
			normalForce.vec = -normalForce.vec
			tangentForce.vec = -tangentForce.vec
			self.objs[1]:applyForce(normalForce)
			self.objs[1]:applyForce(tangentForce)
		end
	end
	inheritance.virtual(FrictionalContact, "applyForcesImpl")

	m.addConstructors(FrictionalContact)


	-- A NailJoint connects two objects with a (virtual) nail.
	-- Works a lot like frictional contact, in that it has forces acting in a normal and tangent direction.
	-- Normal force is also bounded by maximum pulling force threshold.
	-- Can simulate the effect of using multiple nails to make the joint stronger.
	local defaultMaxPullForce = `100.0
	local defaultMaxShearForce = `100.0
	-- TODO: Actually put in reasonable values for the maximum pull and shear forces
	local struct NailJoint
	{
		objs: (&RigidObjectT)[2],
		contactPoint: Vec2,
		contactNormal: Vec2,
		contactTangent: Vec2,
		maxPullForce: real,
		maxShearForce: real,
		numNails: uint
	}
	inheritance.dynamicExtend(RigidConnection, NailJoint)

	terra NailJoint:__construct(o1: &RigidObjectT, o2: &RigidObjectT, cp: Vec2, cn: Vec2, nn: uint) : {}
		self.objs[0] = o1
		self.objs[1] = o2
		self.contactPoint = cp
		self.contactNormal = cn
		self.contactTangent = perp(cn)
		self.maxPullForce = defaultMaxPullForce
		self.maxShearForce = defaultMaxShearForce
		self.numNails = nn
	end

	-- Nail beam1 to beam2 at the center of the edge between corners ci1 and ci2 of beam1
	terra NailJoint:__construct(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint, nn: uint) : {}
		var p1 = beam1:corner(ci1)
		var p2 = beam1:corner(ci2)
		var cp = 0.5*(p1+p2)
		var edge = p2 - p1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (toward the center of mass of beam2)
		if (beam2:centerOfMass() - p1):dot(normal) < 0 then
			normal = -normal
		end
		self:__construct(beam1, beam2, cp, normal, nn)
	end

	terra NailJoint:applyForcesImpl() : {}
		if self.objs[0].active then
			var normalForce = genBoundedNonNegative1DForce(self.contactPoint, self.contactNormal, self.maxPullForce)
			var tangentForce = genBounded1DForce(self.contactPoint, self.contactTangent, self.maxShearForce)
			self.objs[0]:applyForce(normalForce)
			self.objs[0]:applyForce(tangentForce)
		end
		if self.objs[1].active then
			var normalForce = genBoundedNonNegative1DForce(self.contactPoint, -self.contactNormal, self.maxPullForce)
			var tangentForce = genBounded1DForce(self.contactPoint, self.contactTangent, self.maxShearForce)
			self.objs[1]:applyForce(normalForce)
			self.objs[1]:applyForce(tangentForce)
		end
	end
	inheritance.virtual(NailJoint, "applyForcesImpl")

	m.addConstructors(NailJoint)


	return 
	{
		RigidConnection = RigidConnection,
		Hinge = Hinge,
		Cable = Cable,
		Stacking = Stacking,
		Weld = Weld,
		FrictionalContact = FrictionalContact,
		NailJoint = NailJoint
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



