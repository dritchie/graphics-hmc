
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
--    * (Optionally) a vector indicating the direction of the force, if it is 1 DOF
local forceColor = colors.Tableau10.Red
local forceLineWidth = 3.0
local Force = templatize(function(real)

	local Vec2 = Vec(real, 2)

	local struct ForceT { vec: Vec2, pos: Vec2, dof: uint, dir: Vec2 }

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
		self.vec = self.vec + f.vec
		if self.dof == 1 and (f.dof == 2 or (not collinear(self.dir, f.dir))) then
			self.dof = 2
		end
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
	-- (This is important for the default getCentersOfRotation to work properly)
	terra RigidObjectT:applyForce(f: ForceT)
		for i=0,self.forces.size do
			if self.forces(i):tryCombiningWith(&f) then return end
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
					if f.dof > 1 or not collinear(f.dir, self.forces(0).dir) then
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
			-- C.printf("--------------\n")
			for i=0,cor.size do
				var indvidualResidual = real(0.0)
				for j=0,self.forces.size do
					var t = self.forces(j):torque(cor(i))
					-- C.printf("%g\n", ad.val(t))
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
local beamDefaultDensity = `700.0 	-- kg/m^3
local beamDefaultSpecificGravity = `0.6
local beamDefaultLateralLoadCoefficient = `94.52	-- See http://www.fpl.fs.fed.us/documnts/fplgtr/fpl_gtr190.pdf, Chapter 8
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
		depth: real,	-- for volume (and mass) calculation
		density: real,
		specificGravity: real,
		lateralLoadCoefficient: real
	}
	if real == double then
		BeamT.entries:insert({field="color", type=Color3d})
	end
	inheritance.dynamicExtend(RigidObjectT, BeamT)

	-- Construct from four points
	terra BeamT:__construct(b1: Vec2, b2: Vec2, t1: Vec2, t2: Vec2, depth: real) : {}
		-- Default to active, affected by gravity, and visible
		RigidObjectT.__construct(self, true, true, true)
		self.bot1 = b1
		self.bot2 = b2
		self.top1 = t1
		self.top2 = t2
		self.depth = depth
		self.density = beamDefaultDensity
		self.specificGravity = beamDefaultSpecificGravity
		self.lateralLoadCoefficient = beamDefaultLateralLoadCoefficient
		[util.optionally(real == double, function() return quote
			self.color = Color3d.stackAlloc([beamDefaultColor])
		end end)]
	end

	-- Construct from two endpoints and a width (rectangular only)
	terra BeamT:__construct(b: Vec2, t: Vec2, w: real, depth: real) : {}
		var axis = t - b; axis:normalize()
		var whalf = 0.5*w
		var perpvec = whalf*perp(axis)
		self:__construct(b - perpvec, b + perpvec, t - perpvec, t + perpvec, depth)
	end

	-- Construct from a center, length, width, and angle (rectangular only)
	terra BeamT:__construct(c: Vec2, l: real, w: real, a: real, depth: real) : {}
		var lenhalf = 0.5*l
		var axis = polar2rect(lenhalf, a)
		self:__construct(c - axis, c + axis, w, depth)
	end

	-- (Inherit parent destructor)

	terra BeamT:__copy(other: &BeamT)
		RigidObjectT.__copy(self, other)
		self.bot1 = other.bot1
		self.bot2 = other.bot2
		self.top1 = other.top1
		self.top2 = other.top2
		self.depth = other.depth
		self.density = other.density
		self.specificGravity = other.specificGravity
		self.lateralLoadCoefficient = other.lateralLoadCoefficient
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
		-- Area is: 1/2 * || AC x BD ||
		var ac = self.top2 - self.bot1
		var bd = self.top1 - self.bot2
		var cross = ac(0)*bd(1) - ac(1)*bd(0)
		return 0.5 * ad.math.sqrt(cross*cross) * self.depth * self.density
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
			util.fatalError("BeamT.endpoint: index must be 0, 1, 2, or 3\n")
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
			util.fatalError("BeamT:corner: index must be 0, 1, 2, or 3\n")
		end
	end

	-- Selecting opposing edges
	terra BeamT:opposingEdge(i1: uint, i2: uint)
		if (i1 == 0 and i2 == 1) or (i1 ==1 and i2 == 0) then
			return 2,3
		elseif (i1 == 1 and i2 == 2) or (i1 == 2 and i2 == 1) then
			return 0,3
		elseif (i1 == 2 and i2 == 3) or (i1 == 3 and i2 == 2) then
			return 0,1
		elseif (i1 == 0 and i2 == 3) or (i1 == 3 and i2 == 0) then
			return 1,2
		else
			util.fatalError("BeamT:opposingEdge: no such edge\n")
		end
	end

	-- Find closest edge to a point
	local function genEdgeCheck(self, p, i1, i2, besti1, besti2, bestdot)
		return quote
			var a = self:corner(i1)
			var b = self:corner(i2)
			var ab = b - a; ab:normalize()
			var ap = p - a; ap:normalize()
			var dot = ab:dot(ap)
			if dot > bestdot then
				besti1 = i1
				besti2 = i2
				bestdot = dot
			end
		end
	end
	terra BeamT:closestEdge(p: Vec2)
		-- Look for edge with largest dot product
		var besti1 = 0
		var besti2 = 1
		var bestdot = real(-1.0)
		[genEdgeCheck(self, p, 0, 1, besti1, besti2, bestdot)]
		[genEdgeCheck(self, p, 1, 2, besti1, besti2, bestdot)]
		[genEdgeCheck(self, p, 2, 3, besti1, besti2, bestdot)]
		[genEdgeCheck(self, p, 3, 0, besti1, besti2, bestdot)]
		return besti1, besti2
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
		var beam = BeamT.heapAlloc(bot, top, w, 1.0)
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
		var beam = BeamT.heapAlloc(bot, top, w, 1.0)
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
			ForceT { mag*dir, pos, 1, dir}
		end
	end)

	-- Generate a nonnegative 1-DOF force along a particular direction
	local genNonNegative1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=0.0, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1, dir}
		end
	end)

	-- Generate a bounded 1-DOF force along a particular direction
	local genBounded1DForce = macro(function(pos, dir, boundMag)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=-boundMag, upperBound=boundMag, initialVal=0.0})
			-- var mag = boundMag*gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=-1.0, upperBound=1.0, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1, dir}
		end
	end)

	-- Generate an lower-bounded 1-DOF force along a particular direction
	local genLowerBounded1DForce = macro(function(pos, dir, boundMag)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=boundMag, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1, dir}
		end
	end)

	-- Generate a bounded, nonnegative 1-DOF force along a particular direction
	local genBoundedNonNegative1DForce = macro(function(pos, dir, boundMag)
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=0.0, upperBound=boundMag, initialVal=0.0})
		in
			ForceT { mag*dir, pos, 1, dir}
		end
	end)

	-- Generate a 2-DOF force
	local gen2DForce = macro(function(pos)
		return quote
			var x = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0}) 
			var y = gaussian(forcePriorMean, forcePriorVariance, {structural=false, initialVal=0.0})
			var f : ForceT
			f.vec = Vec2.stackAlloc(x, y)
			f.pos = pos
			f.dof = 2
		in
			f
		end
	end)

	----------------------------------

	-- Returns t1, t2
	local terra rayIntersect(p1: Vec2, d1: Vec2, p2: Vec2, d2: Vec2)
		-- dx = bs.x - as.x
		-- dy = bs.y - as.y
		-- det = bd.x * ad.y - bd.y * ad.x
		-- u = (dy * bd.x - dx * bd.y) / det
		-- v = (dy * ad.x - dx * ad.y) / det
		var dx = p2(0) - p1(0)
		var dy = p2(1) - p1(1)
		var det = d2(0)*d1(1) - d2(1)*d1(0)
		var t1 = (dy*d2(0) - dx*d2(1)) / det
		var t2 = (dy*d1(0) - dx*d1(1)) / det

		return t1, t2
	end

	-- local terra testRayIntersect()
	-- 	var p1 = Vec2.stackAlloc(0.0, 1.0)
	-- 	var d1 = Vec2.stackAlloc(1.0, 0.0)
	-- 	var p2 = Vec2.stackAlloc(1.0, 0.0)
	-- 	var d2 = Vec2.stackAlloc(0.0, 1.0)
	-- 	var t1, t2 = rayIntersect(p1, d1, p2, d2)
	-- 	util.assert(t1 == 1.0 and t2 == 1.0, "t1: %g, t2: %g\n", ad.val(t1), ad.val(t2))
	-- end
	-- testRayIntersect()

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
			-- Normal force needs to have nonzero magnitude so that tangentForce can have an interval of
			--    nonzero size. We use 1e-14 b/c erp uses 1e-15 as the fudge factor for bounds.
			normalForce = genLowerBounded1DForce(self.contactPoint, self.contactNormal, 1e-14)
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


	-- A NailJoint connects two beams with a (virtual) nail.
	-- Assumption is that the nail is through beam 2 into beam 1
	-- It exerts a normal force that is bounded by a maximum pulling force threshold on the negative side.
	-- The normal force can also go infinitely positive, indicating compression. In this way, the NailJoint
	--    also doubles as a FrictionalContact.
	-- It also exerts a tangent force that is bounded by a maximum shear force threshold.
	-- This is like friction, but much stronger.
	local struct NailJoint
	{
		objs: (&BeamT)[2],
		contactPoint: Vec2,
		contactNormal: Vec2,
		contactTangent: Vec2,
		maxPullForce: real,
		maxShearForce: real
	}
	inheritance.dynamicExtend(RigidConnection, NailJoint)

	-- meters to millimeters
	local toMM = macro(function(x) return `1000.0*x end)

	terra NailJoint:__construct(o1: &BeamT, o2: &BeamT, cp: Vec2, cn: Vec2, nailDiameter: real, penetrationDepth: real) : {}
		self.objs[0] = o1
		self.objs[1] = o2
		self.contactPoint = cp
		self.contactNormal = cn
		self.contactTangent = perp(cn)
		util.assert(penetrationDepth > 10.0*nailDiameter, "Nail penetration depth too small for given nail diameter\n")
		self.maxPullForce = 54.12 * ad.math.pow(o1.specificGravity, 5.0/2.0) * toMM(nailDiameter) * toMM(penetrationDepth)	-- maximum short-term load
		self.maxPullForce = (1.0/6.0)*self.maxPullForce		-- maximum long-term load
		self.maxPullForce = 1.1*self.maxPullForce	-- maximum normal load
		self.maxShearForce = o1.lateralLoadCoefficient * ad.math.pow(toMM(nailDiameter), 3.0/2.0)

		-- C.printf("maxPullForce: %g, maxShearForce: %g\n", ad.val(self.maxPullForce), ad.val(self.maxShearForce))
	end

	local terra determinePenetrationDepth(beam2: &BeamT, cp: Vec2, normal: Vec2, nailLength: real)
		-- Determine the penetration depth by 'raycasting' along the contact normal direction
		var corner1, corner2 = beam2:closestEdge(cp)
		-- C.printf("corner1: %d, corner2: %d\n", corner1, corner2)
		var norm_start = cp
		var norm_dir = -normal
		norm_start = norm_start - 0.001*norm_dir
		var b2edge1_start = beam2:corner(corner1)
		var b2edge1_dir = beam2:corner(corner2) - b2edge1_start
		var oppEdge_i1, oppEdge_i2 = beam2:opposingEdge(corner1, corner2)
		-- C.printf("oppcorner1: %d, oppcorner2: %d\n", oppEdge_i1, oppEdge_i2)
		var b2edge2_start = beam2:corner(oppEdge_i1)
		var b2edge2_dir = beam2:corner(oppEdge_i2) - b2edge2_start
		var t11, t21 = rayIntersect(norm_start, norm_dir, b2edge1_start, b2edge1_dir)
		var t12, t22 = rayIntersect(norm_start, norm_dir, b2edge2_start, b2edge2_dir)
		-- C.printf("t21: %g, t22: %g\n", ad.val(t21), ad.val(t22))
		var isect1 = b2edge1_start + t21*b2edge1_dir
		var isect2 = b2edge2_start + t22*b2edge2_dir
		var thickness = (isect2 - isect1):norm()
		var penetrationDepth = nailLength - thickness
		-- Thickness should be about half the penetration depth
		util.assert(thickness < 0.7*penetrationDepth,
			"Nail is too short. Penetration depth should be about 2x thickness of side member for our model to be accurate.\nthickness: %g\n", ad.val(thickness))
		util.assert(thickness > 0.3*penetrationDepth,
			"Nail is too long. Penetration depth should be about 2x thickness of side member for our model to be accurate.\nthickness: %g\n", ad.val(thickness))

		-- C.printf("penetrationDepth: %g\n", ad.val(penetrationDepth))
		return penetrationDepth
	end

	-- Nail two beams together using one nail at the center of the edge between corner ci1 and ci2 of beam1.
	terra NailJoint:__construct(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint, nailDiameter: real, nailLength: real) : {}
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0 then
			normal = -normal
		end
		var cp = 0.5*(cp1 + cp2)		
		var penetrationDepth = determinePenetrationDepth(beam2, cp, normal, nailLength)
		self:__construct(beam1, beam2, cp, normal, nailDiameter, penetrationDepth)
	end

	-- (Convenience method)
	-- Nail two beams together.
	-- Connect beam1 to beam2 along the edge between corners ci1 and ci2 of beam1
	-- The nail penetrates beam2 along the edge ci3 to ci4
	-- This will generate two NailJoint objects
	NailJoint.methods.makeBeamContacts = terra(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint, nailDiameter: real, nailLength: real)
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0 then
			normal = -normal
		end
		var penDepth1 = determinePenetrationDepth(beam2, cp1, normal, nailLength)
		var penDepth2 = determinePenetrationDepth(beam2, cp2, normal, nailLength)
		var nj1 = NailJoint.heapAlloc(beam1, beam2, cp1, normal, nailDiameter, penDepth1)
		var nj2 = NailJoint.heapAlloc(beam1, beam2, cp2, normal, nailDiameter, penDepth2)
		return nj1, nj2
	end

	local terra logistic(x: real, coeff: real)
		return 1.0 / (1.0 + ad.math.exp(-coeff*x))
	end
	local terra lerp(a: real, b: real, t: real)
		return (1.0-t)*a + t*b
	end

	terra NailJoint:applyForcesImpl() : {}
		var normalForce : ForceT
		var tangentForce : ForceT
		if self.objs[0].active or self.objs[1].active then
			var nCompress = genNonNegative1DForce(self.contactPoint, self.contactNormal)
			var nTension = genBoundedNonNegative1DForce(self.contactPoint, -self.contactNormal, self.maxPullForce)
			normalForce = nCompress; normalForce:combineWith(&nTension)
			-- tangentForce = genBounded1DForce(self.contactPoint, self.contactTangent, self.maxShearForce)
			-- normalForce = gen1DForce(self.contactPoint, self.contactNormal)
			-- tangentForce = gen1DForce(self.contactPoint, self.contactTangent)
			-- C.printf("tangentForceMag: %g, maxShearForce: %g\n", ad.val(tangentForce.vec):norm(), ad.val(self.maxShearForce))
			-- var logisticShapeParam = ad.math.exp(gaussian([math.log(0.1)], 1000.0, {structural=false, initialVal=[math.log(0.1)], upperBound=0.0}))
			-- C.printf("logisticShapeParam: %g\n", ad.val(logisticShapeParam))
			-- var logisticShapeParam = 0.1
			var logisticShapeParam = 0.01
			-- var logisticShapeParam = 1.0
			var forceMag = lerp(-self.maxShearForce, self.maxShearForce, logistic(gaussian(0.0, 10000000.0, {structural=false, initialVal=0.0}), logisticShapeParam))
			tangentForce.vec =  forceMag * self.contactTangent
			tangentForce.pos = self.contactPoint
			tangentForce.dir = self.contactTangent
			tangentForce.dof = 1
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



