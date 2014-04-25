terralib.require("prob")

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

local lerp = macro(function(a, b, t)
	return `(1.0-t)*a + t*b
end)

local perp = macro(function(vec)
	local VecT = vec:gettype()
	return `VecT.stackAlloc(-vec(1), vec(0))
end)

local rot = macro(function(vec, ang)
	local VecT = vec:gettype()
	return quote
		var sinang = ad.math.sin(ang)
		var cosang = ad.math.cos(ang)
	in
		VecT.stackAlloc(cosang*vec(0) - sinang*vec(1), sinang*vec(0) + cosang*vec(1))
	end
end)

-- Magnitude of cross product
local cross = macro(function(v1, v2)
	return `v1(0)*v2(1) - v1(1)*v2(0)
end)

-- Check whether two vectors are on the same line
-- (Uses the z-component of the cross product)
local collinearThresh = 1e-16
local collinear = macro(function(v1, v2)
	return `ad.math.fabs(cross(v1, v2)) < collinearThresh
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

-- Returns t1, t2
local rayIntersect = templatize(function(real)
	local Vec2 = Vec(real, 2)
	return terra(p1: Vec2, d1: Vec2, p2: Vec2, d2: Vec2)
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
end)


-- A force is:
--    * A vector indicating the direction and magnitude of the force
--    * A position indicating where the force is applied
--    * A number indicating the number of degrees of freedom in the force
--      (i.e. is it constrained to point in a particular direction, or is it free?)
--    * (Optionally) a vector indicating the direction of the force, if it is 1 DOF
local forceColor = colors.Tableau10.Gray
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

	terra ForceT:draw(scale: double)
		var endpoint = self.pos + scale*self.vec
		drawLine(ad.val(self.pos), ad.val(endpoint), forceLineWidth, Color3d.stackAlloc([forceColor]))
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

	terra RigidObjectT:applyExternalLoad(gravConst: real, mass: real, point: Vec2)
		var down = Vec2.stackAlloc(0.0, -1.0)
		self:applyForce(ForceT{gravConst * mass * down, point, 0, down})
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
	-- Returns the average force and torque magnitude
	terra RigidObjectT:calculateResiduals(fresOut: &Vec2, tresOut: &Vector(real))
		tresOut:clear()
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
			var avgFmag = real(0.0)
			for i=0,self.forces.size do
				totalf = totalf + self.forces(i).vec
				avgFmag = avgFmag + self.forces(i).vec:norm()
			end
			avgFmag = avgFmag / self.forces.size
			@fresOut = totalf
			-- Compute torque residuals
			-- var normconst = 1.0/cor.size
			var avgTmag = real(0.0)
			for i=0,cor.size do
				var indvidualResidual = real(0.0)
				for j=0,self.forces.size do
					var t = self.forces(j):torque(cor(i))
					avgTmag = avgTmag + ad.math.fabs(t)
					indvidualResidual = indvidualResidual + t
				end
				-- tresOut:push(normconst*indvidualResidual)
				tresOut:push(indvidualResidual)
			end
			avgTmag = avgTmag / (cor.size*self.forces.size)
			return avgFmag, avgTmag
		end
	end

	terra RigidObjectT:mass() : real
		return 0.0
	end
	inheritance.virtual(RigidObjectT, "mass")

	-- OpenGL drawing
	inheritance.purevirtual(RigidObjectT, "drawImpl", {}->{})
	terra RigidObjectT:draw()
		if self.visible then
			self:drawImpl()
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
		lateralLoadCoefficient: real,
		color: Color3d
	}
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
		self.color = Color3d.stackAlloc([beamDefaultColor])
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

	-- Construct from a length, width, and angle, and base (rectangular only)
	terra BeamT:__construct(l: real, w: real, a: real, base: Vec2, depth: real) : {}
		var axis = polar2rect(l, a)
		self:__construct(base, base + axis, w, depth)
	end

	-- Construct axis-aligned beam from a base, width, height, and depth
	terra BeamT:__construct(base: Vec2, w: real, h: real, depth: real) : {}
		var ep2 = base; ep2(1) = ep2(1) + h
		self:__construct(base, ep2, w, depth)
	end

	-- Create a non-rectangular beam from a base point and normal, base angle, top angle, length, width, and depth
	terra BeamT:__construct(basePoint: Vec2, baseNormal: Vec2, baseAng: real, topAng: real, len: real, w: real, depth: real) : {}
		-- C.printf("------------------------------------------------\n")
		-- C.printf("basePoint: (%g, %g), baseNormal: (%g, %g)\n",
		-- 	ad.val(basePoint(0)), ad.val(basePoint(1)), ad.val(baseNormal(0)), ad.val(baseNormal(1)))
		var baseTangent = perp(baseNormal)
		var w2 = 0.5*w
		baseTangent = w2*baseTangent
		var bot1 = basePoint - baseTangent
		var bot2 = basePoint + baseTangent
		-- C.printf("bot1: (%g, %g), bot2: (%g, %g)\n",
		-- 	ad.val(bot1(0)), ad.val(bot1(1)), ad.val(bot2(0)), ad.val(bot2(1)))
		baseNormal = rot(baseNormal, baseAng)
		var topMid = basePoint + len*baseNormal
		var top1 = topMid - baseTangent
		var top2 = topMid + baseTangent
		-- C.printf("top1: (%g, %g), top2: (%g, %g)\n",
		-- 	ad.val(top1(0)), ad.val(top1(1)), ad.val(top2(0)), ad.val(top2(1)))
		-- A bunch of trigonometry to figure out where the top corners should be, given the rotated end cap
		-- NOTE: Am I handling signs of angles here in an OK way?
		var theta1 = ad.math.fabs(topAng)
		var theta2 = ad.math.fabs(baseAng)
		var theta4 = [math.pi/2] - theta2
		var theta3 = [math.pi] - theta4
		var theta5 = [math.pi] - theta3 - theta1
		var displacement = w2*ad.math.sin(topAng)/ad.math.sin(theta5)	-- Use topAng instead of theta1 to get signed value
		top1 = top1 - displacement*baseNormal
		top2 = top2 + displacement*baseNormal
		-- C.printf("[AFTER DISPLACE] top1: (%g, %g), top2: (%g, %g)\n",
		-- 	ad.val(top1(0)), ad.val(top1(1)), ad.val(top2(0)), ad.val(top2(1)))
		self:__construct(bot1, bot2, top1, top2, depth)
	end

	-- (Utilities used by the below creation methods)

	-- Identify the long edge closest to point p along direction d, then
	--    find the point that is t percent along that edge
	terra BeamT:interpolateAlongClosestEdge(p: Vec2, d: Vec2, t: real)
		var c1, c2 = self:closerEdge(p, d, 0, 3, 1, 2)
		return lerp(self:corner(c1), self:corner(c2), t), c1, c2
	end

	-- Compute the dot product (cos of angle of separation) between edge
	--    (c1, c2) of beam and the direction d.
	-- We use whichever orientation of d produces the largest value (closest to 1)
	terra BeamT:dotProdWithEdge(d: Vec2, c1: uint, c2: uint)
		var edgevec = self:corner(c1) - self:corner(c2)
		edgevec:normalize()
		return ad.math.fmax(edgevec:dot(d), edgevec:dot(-d))
	end

	-- Given the dot product between the two connecting edges of two Beams,
	--    find the minimum separation that will keep those Beams in contact
	--    but not interpenetrating.
	-- d is the dot product
	-- w is the width of the connecting Beam.
	local terra minsep(d: real, w: real)
		-- Option 1: w/2d * sqrt(1 - d^2)
		return (w/(2.0*d)) * ad.math.sqrt(1 - d*d)
		-- -- Option 2: w/2 * tan(|acos(d)|)
		-- return 0.5*w*ad.math.tan(ad.math.fabs(ad.math.acos(d)))
	end

	-- Create a Beam that will connect to beam at the point t percent along
	--   its length. This new Beam's other endpoint is at ep,  and it has width w.
	BeamT.methods.createConnectingBeamWithEndpoint = terra(beam: &BeamT, t: real, ep: Vec2, w: real, depth: real)
		var approxEp2 = lerp(beam:endpoint(0), beam:endpoint(1), t)
		var d = approxEp2 - ep; d:normalize()
		var p, c1, c2 = beam:interpolateAlongClosestEdge(ep, d, t)
		d = p - ep; d:normalize()
		var dot = beam:dotProdWithEdge(perp(d), c1, c2)
		var sepdist = minsep(dot, w)
		var ep2 = p - sepdist*d
		return BeamT.heapAlloc(ep, ep2, w, depth)
	end
	-- Same as above, but instead of the other endpoint, we get vector indicating (roughly) how long
	--    (and in what direction) the created beam should be.
	-- Provided direction should point AWAY from the connection point
	BeamT.methods.createConnectingBeamWithLengthVec = terra(beam: &BeamT, t: real, d: Vec2, w: real, depth: real)
		var approxEp2 = lerp(beam:endpoint(0), beam:endpoint(1), t)
		var ep = approxEp2 + d
		var fullD = d
		d = -d; d:normalize()
		var p, c1, c2 = beam:interpolateAlongClosestEdge(ep, d, t)
		var dot = beam:dotProdWithEdge(perp(d), c1, c2)
		var sepdist = minsep(dot, w)
		var ep2 = p - sepdist*d
		return BeamT.heapAlloc(ep2+fullD, ep2, w, depth)
	end

	-- Create a Beam that will connect beam1 to beam2 at the points t1 and t2
	--    percent along their respective lengths. The new Beam will have width w.
	BeamT.methods.createBridgingBeam = terra(beam1: &BeamT, beam2: &BeamT, t1: real, t2: real, w: real, depth: real)
		var approxEp1 = lerp(beam1:endpoint(0), beam1:endpoint(1), t1)
		var approxEp2 = lerp(beam2:endpoint(0), beam2:endpoint(1), t2)
		var d = approxEp1 - approxEp2; d:normalize()
		var ep1, c11, c12 = beam1:interpolateAlongClosestEdge(approxEp2, d, t1)
		var ep2, c21, c22 = beam2:interpolateAlongClosestEdge(approxEp1, -d, t2)
		d = ep1 - ep2; d:normalize()
		var dperp = perp(d)
		var dot1 = beam1:dotProdWithEdge(dperp, c11, c12)
		var dot2 = beam2:dotProdWithEdge(dperp, c21, c22)
		var sepdist1 = minsep(dot1, w)
		var sepdist2 = minsep(dot2, w)
		ep1 = ep1 - sepdist1*d
		ep2 = ep2 + sepdist2*d
		return BeamT.heapAlloc(ep1, ep2, w, depth)
	end

	-- Create a non-rectangular beam that will connect flush to beam at point p
	BeamT.methods.createFlushConnectingBeam = terra(beam: &BeamT, p: Vec2, baseAng: real, topAng: real, l: real, w: real, d: real)
		var c1, c2 = beam:edgeContainingPoint(p)
		var normal = beam:edgeNormal(c1, c2)
		return BeamT.heapAlloc(p, normal, baseAng, topAng, l, w, d)
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
		self.color = other.color
	end

	terra BeamT:newcopy() : &RigidObjectT
		var newbeam = m.new(BeamT)
		newbeam:__copy(self)
		return newbeam
	end
	inheritance.virtual(BeamT, "newcopy")

	-- Generate a bunch of points uniformly spaced along the length of this beam
	terra BeamT:getCentersOfRotation(num: uint, output: &Vector(Vec2)) : bool
		var e1 = self:endpoint(0)
		var e2 = self:endpoint(1)
		for i=0,num do
			var t = (i+0.5)/num
			var cor = (1.0-t)*e1 + t*e2
			output:push(cor)
		end
		-- output:push(self:corner(0))
		-- output:push(self:corner(1))
		-- output:push(self:corner(2))
		-- output:push(self:corner(3))
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

	-- Apply rotation + translation transform
	terra BeamT:transform(displacement: Vec2, ang: real)
		var com = self:centerOfMass()
		var rb1 = self.bot1 - com
		var rb2 = self.bot2 - com
		var rt1 = self.top1 - com
		var rt2 = self.top2 - com
		self.bot1 = rot(rb1, ang) + com + displacement
		self.bot2 = rot(rb2, ang) + com + displacement
		self.top1 = rot(rt1, ang) + com + displacement
		self.top2 = rot(rt2, ang) + com + displacement
	end

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

	-- Get the (outward-facing) normal of edge c1, c2
	terra BeamT:edgeNormal(c1: uint, c2: uint)
		var p1 = self:corner(c1)
		var p2 = self:corner(c2)
		var vec = p2 - p1; vec:normalize(); vec = perp(vec)
		if vec:dot(self:centerOfMass() - p1) > 0.0 then
			vec = -vec
		end
		return vec
	end

	-- Generate the (unnormalized) vector along the edge c1, c2
	terra BeamT:edgeVec(c1: uint, c2: uint)
		var p1 = self:corner(c1)
		var p2 = self:corner(c2)
		return p2 - p1
	end

	-- Find the edge that point p lies on
	-- Assumes that p is on some edge, throws an error if not
	terra BeamT:edgeContainingPoint(p: Vec2)
		-- For an edge (p1, p2) to contain p, it must be collinear
		--    with (p1, p)
		var c0 = self:corner(0)
		var c1 = self:corner(1)
		var c2 = self:corner(2)
		var c3 = self:corner(3)
		if collinear(c1-c0, p-c0) then return 0, 1 end
		if collinear(c2-c1, p-c1) then return 1, 2 end
		if collinear(c3-c2, p-c2) then return 2, 3 end
		if collinear(c0-c3, p-c3) then return 0, 3 end
		util.fatalError("BeamT:edgeContainingPoint - provided point is not on any edge.\n")
	end

	-- Find the distance to a particular edge when starting at some point p
	--    and looking along some direction d
	terra BeamT:lineDistToEdge(p: Vec2, d: Vec2, ci1: uint, ci2: uint)
		-- Do a ray intersection. Report the (p + td) t value.
		-- If the edge t value is not in [0,1], then return -1.0 (no intersection)
		var ep1 = self:corner(ci1)
		var ep2 = self:corner(ci2)
		var ed = ep2 - ep1
		p = p - 1e-15*d  -- Don't want to miss intersections at 0
		var t1, t2 = [rayIntersect(real)](p, d, ep1, ed)
		-- C.printf("p: (%g, %g), d: (%g, %g), t1: %g, t2: %g\n",
		-- 	ad.val(p(0)), ad.val(p(1)), ad.val(d(0)), ad.val(d(1)), ad.val(t1), ad.val(t2))
		if t2 >=0.0 and t2 <= 1.0 then
			return ad.math.fabs(t1)
		else
			return real(-1.0)
		end
	end

	-- Which of two edges is closer?
	terra BeamT:closerEdge(p: Vec2, d: Vec2, e11: uint, e12: uint, e21: uint, e22: uint)
		-- C.printf("-- BEGIN closerEdge\n")
		var dist1 = self:lineDistToEdge(p, d, e11, e12)
		-- util.assert(dist1 == self:lineDistToEdge(p, d, e12, e11))
		var dist2 = self:lineDistToEdge(p, d, e21, e22)
		-- util.assert(dist2 == self:lineDistToEdge(p, d, e22, e21))
		-- C.printf("dist1: %g, dist2: %g\n", ad.val(dist1), ad.val(dist2))
		-- C.printf("-- END closerEdge\n")
		var c1: uint
		var c2: uint
		if dist1 < dist2 then
			c1 = e11
			c2 = e12
		else
			c1 = e21
			c2 = e22
		end
		-- Sort lowest-to-highest, for consistency
		if c1 > c2 then
			var tmp = c1
			c1 = c2
			c2 = tmp
		end
		return c1, c2
	end

	-- This notion of width is only meaningful for rectangular beams; use wisely
	terra BeamT:width()
		return (self.top2 - self.top1):norm()
	end

	-- Find the thickness of the beam from point p looking along direction d
	terra BeamT:thickness(p: Vec2, d: Vec2)
		-- Find line distance to all four edges
		var d01 = self:lineDistToEdge(p, d, 0, 1)
		var d12 = self:lineDistToEdge(p, d, 1, 2)
		var d23 = self:lineDistToEdge(p, d, 2, 3)
		var d30 = self:lineDistToEdge(p, d, 3, 0)
		-- The ray (p + td) can only pass through two of them, so find the two non-negative
		--    distances and return the bigger minus the smaller
		if d01 >= 0.0 and d12 >= 0.0 then
			return ad.math.fmax(d01 - d12, d12 - d01)
		elseif d01 >= 0.0 and d23 >= 0.0 then
			return ad.math.fmax(d01 - d23, d23 - d01)
		elseif d01 >= 0.0 and d30 >= 0.0 then
			return ad.math.fmax(d01 - d30, d30 - d01)
		elseif d12 >= 0.0 and d23 >= 0.0 then
			return ad.math.fmax(d12 - d23, d23 - d12)
		elseif d12 >= 0.0 and d30 >= 0.0 then
			return ad.math.fmax(d01 - d30, d30 - d01)
		elseif d23 >= 0.0 and d30 >= 0.0 then
			return ad.math.fmax(d23 - d30, d30 - d23)
		-- If none of the above cases pass, then the ray (p + td) never intersects the beam at all
		-- Return -1.0 in this case
		else
			return real(-1.0)
		end
	end

	-- Find the 'apparent' thickness of the beam from point p looking along direction d
	-- This is the distance a ray travels from p before it exits the back side of the beam
	terra BeamT:apparentThickness(p: Vec2, d: Vec2)
		-- Find the line distance to all four edges, report the maximum
		var d01 = self:lineDistToEdge(p, d, 0, 1)
		var d12 = self:lineDistToEdge(p, d, 1, 2)
		var d23 = self:lineDistToEdge(p, d, 2, 3)
		var d30 = self:lineDistToEdge(p, d, 3, 0)
		-- C.printf("d01, d12, d23, d30: %g, %g, %g, %g\n",
		-- 	ad.val(d01), ad.val(d12), ad.val(d23), ad.val(d30))
		return ad.math.fmax(d01, ad.math.fmax(d12, ad.math.fmax(d23, d30)))
	end

	-- OpenGL drawing code
	terra BeamT:drawImpl() : {}
		drawQuad(ad.val(self.bot1), ad.val(self.bot2), ad.val(self.top2), ad.val(self.top1), self.color)
	end
	inheritance.virtual(BeamT, "drawImpl")	

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
	terra RigidSceneT:draw(forceScale: double)
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		gl.gluOrtho2D(0, ad.val(self.width), 0, ad.val(self.height))
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

	m.addConstructors(RigidSceneT)
	return RigidSceneT
end)


-------------------------------------------------------------------------------
---                            Connections                                  ---
-------------------------------------------------------------------------------

local function Connections()
	local forcePriorMean = 0.0
	local forcePriorVariance = 100000000000.0
	-- local forcePriorVariance = 10000000000000000.0
	-- local forcePriorVariance = 1000000.0
	-- local forcePriorVariance = 10000.0

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
	local genBounded1DForce = macro(function(pos, dir, boundMag, optBoundShape)
		optBoundShape = optBoundShape or `1.0
		return quote
			var mag = gaussian(forcePriorMean, forcePriorVariance, {structural=false, lowerBound=-boundMag, upperBound=boundMag, boundShapeParam=optBoundShape, initialVal=0.0})
			-- var mag = uniform(-boundMag, boundMag, {structural=false, lowerBound=-boundMag, upperBound=boundMag, boundShapeParam=optBoundShape, initialVal=0.0})
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
			-- var mag = uniform(0.0, boundMag, {structural=false, lowerBound=0.0, upperBound=boundMag, initialVal=0.0})
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

	-- Connect two Beams under frictional contact
	-- Connect beam1 to beam2 along the edge between corners ci1 anc ci2 of beam1
	-- Connection is at the center of the ci1,ci2 edge
	terra FrictionalContact:__construct(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint) : {}
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0.0 then
			normal = -normal
		end
		var cp = 0.5*(cp1 + cp2)
		self:__construct(beam1, beam2, cp, normal)
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
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0.0 then
			normal = -normal
		end
		var fc1 = FrictionalContact.heapAlloc(beam1, beam2, cp1, normal)
		var fc2 = FrictionalContact.heapAlloc(beam1, beam2, cp2, normal)
		return fc1, fc2
	end

	-- Connect beam1 to beam2 using two frictional contacts
	-- The user asserts that beam1 and beam2 touch along edges (e11, e12) and (e21, e22)
	-- This method figures out where the contacts should be placed along that shared edge
	FrictionalContact.methods.makeBeamContacts:adddefinition((terra(beam1: &BeamT, beam2: &BeamT, e11: uint, e12: uint, e21: uint, e22: uint)
		-- C.printf("----------------------------------    \n")
		-- We know that the four edge points lie on the same line. So we pick one point to be 'zero' (arbitrarily),
		--    compute signed distances from it to all other points, and then place the contacts at the middle two points
		var p11 = beam1:corner(e11)
		var p12 = beam1:corner(e12)
		var p21 = beam2:corner(e21)
		var p22 = beam2:corner(e22)
		-- C.printf("p11: (%g, %g), p12: (%g, %g)\n",
		-- 	ad.val(p11(0)), ad.val(p11(1)), ad.val(p12(0)), ad.val(p12(1)))
		-- C.printf("p21: (%g, %g), p12: (%g, %g)\n",
		-- 	ad.val(p21(0)), ad.val(p21(1)), ad.val(p22(0)), ad.val(p22(1)))
		var origin = p11	-- Arbitrary choice
		var axis = p12 - p11
		-- Relabel the points so that we can correspond the 'lo' and 'hi'
		--    ends of each edge
		var lo1 = p11
		var hi1 = p12
		-- C.printf("lo1: (%g, %g), hi1: (%g, %g)\n",
		-- 	ad.val(lo1(0)), ad.val(lo1(1)), ad.val(hi1(0)), ad.val(hi1(1)))
		var axis2 = p22 - p21
		var lo2 = p21
		var hi2 = p22
		-- C.printf("lo2: (%g, %g), hi2: (%g, %g)\n",
		-- 	ad.val(lo2(0)), ad.val(lo2(1)), ad.val(hi2(0)), ad.val(hi2(1)))
		if axis:dot(axis2) < 0.0 then
			lo2 = p22
			hi2 = p21
			-- C.printf("[FLIPPED!] lo2: (%g, %g), hi2: (%g, %g)\n",
			-- 	ad.val(lo2(0)), ad.val(lo2(1)), ad.val(hi2(0)), ad.val(hi2(1)))
		end
		-- Compute distances along the arbitrarily-chosen number line
		var dlo1 = 0.0
		var dhi1 = axis:norm()
		axis = axis / dhi1	-- Normalize axis
		var dlo2 = (lo2 - origin):dot(axis)
		var dhi2 = (hi2 - origin):dot(axis)
		-- C.printf("dlo1: %g, dhi1: %g, dlo2: %g, dhi2: %g\n",
		-- 	ad.val(dlo1), ad.val(dhi1), ad.val(dlo2), ad.val(dhi2))
		-- The contact points should be at max(dlo1, dlo2) and min(dhi1, dhi2)
		var cplo = lo1
		if dlo2 > dlo1 then cplo = lo2 end
		var cphi = hi1
		if dhi2 < dhi1 then cphi = hi2 end
		-- if cplo == lo1 then C.printf("Picking cplo = lo1\n") else C.printf("Picking cplo = lo2\n") end
		-- if cphi == hi1 then C.printf("Picking cphi = hi1\n") else C.printf("Picking cphi = hi2\n") end

		-- Derive the contact normal, as in all the other constructors
		var normal = perp(axis)
		if (beam2:centerOfMass() - cplo):dot(normal) >= 0.0 then
			normal = -normal
		end
		-- Create and return contact objects
		var fc1 = FrictionalContact.heapAlloc(beam1, beam2, cplo, normal)
		var fc2 = FrictionalContact.heapAlloc(beam1, beam2, cphi, normal)
		return fc1, fc2
	end):getdefinitions()[1])

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
			-- Don't double-count the DOFs for these forces
			if self.objs[0].active then
				normalForce.dof = 0
				tangentForce.dof = 0
			end
			self.objs[1]:applyForce(normalForce)
			self.objs[1]:applyForce(tangentForce)
		end
	end
	inheritance.virtual(FrictionalContact, "applyForcesImpl")

	m.addConstructors(FrictionalContact)


	-- A NailJoint connects two beams with a (virtual) nail.
	-- Assumption is that the nail is through beam 2 into beam 1
	-- The connection does not have to be perpendicular, but the assumption is that the nail is driven parallel into beam1
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

	terra NailJoint:__construct(o1: &BeamT, o2: &BeamT, cp: Vec2, cn: Vec2, nailDiameter: real, penetrationDepth: real, numNails: uint) : {}
		self.objs[0] = o1
		self.objs[1] = o2
		self.contactPoint = cp
		self.contactNormal = cn
		self.contactTangent = perp(cn)

		-- var constrainedPenetrationDepth = gaussian(penetrationDepth, 10000.0, {structural=false, initialVal=penetrationDepth, lowerBound=10.0*nailDiameter})
		-- factor(softeq(constrainedPenetrationDepth, penetrationDepth, 0.5*nailDiameter))
		-- util.assert(penetrationDepth > 10.0*nailDiameter,
		-- 	"Nail penetration depth too small for given nail diameter\npenetrationDepth: %g, nailDiameter: %g\n", ad.val(penetrationDepth), ad.val(nailDiameter))
		
		self.maxPullForce = 54.12 * ad.math.pow(o1.specificGravity, 5.0/2.0) * toMM(nailDiameter) * toMM(penetrationDepth)	-- maximum short-term load
		self.maxPullForce = (1.0/6.0)*self.maxPullForce		-- maximum long-term load
		self.maxPullForce = 1.1*self.maxPullForce	-- maximum normal load
		self.maxShearForce = o1.lateralLoadCoefficient * ad.math.pow(toMM(nailDiameter), 3.0/2.0)

		-- Boost force thresholds as we use more nails
		self.maxPullForce = double(numNails)*self.maxPullForce
		self.maxShearForce = double(numNails)*self.maxShearForce

		-- C.printf("maxPullForce: %g, maxShearForce: %g\n", ad.val(self.maxPullForce), ad.val(self.maxShearForce))
	end

	local terra determinePenetrationDepth(beam2: &BeamT, cp: Vec2, normal: Vec2, nailLength: real)
		-- Find the apparent thickness from cp looking along the normal
		var thickness = beam2:apparentThickness(cp, -normal)
		-- C.printf("thickness: %g\n", ad.val(thickness))
		var penetrationDepth = nailLength - thickness

		-- Thickness should be about half the penetration depth
		-- factor(softeq(thickness/penetrationDepth, 0.5, 0.1))
		-- util.assert(thickness < 0.7*penetrationDepth,
		-- 	"Nail is too short. Penetration depth should be about 2x thickness of side member for shear model to be accurate.\nthickness: %g, penetrationDepth: %g\n", ad.val(thickness), ad.val(penetrationDepth))
		
		return penetrationDepth
	end

	-- Nail two beams together using one nail at the center of the edge between corner ci1 and ci2 of beam1.
	terra NailJoint:__construct(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint, nailDiameter: real, nailLength: real, numNails: uint) : {}
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0.0 then
			normal = -normal
		end
		var cp = 0.5*(cp1 + cp2)		
		var penetrationDepth = determinePenetrationDepth(beam2, cp, normal, nailLength)
		self:__construct(beam1, beam2, cp, normal, nailDiameter, penetrationDepth, numNails)
	end

	-- (Convenience method)
	-- Nail two beams together.
	-- Connect beam1 to beam2 along the edge between corners ci1 and ci2 of beam1
	-- The nail penetrates beam2 along the edge ci3 to ci4
	-- This will generate two NailJoint objects
	NailJoint.methods.makeBeamContacts = terra(beam1: &BeamT, beam2: &BeamT, ci1: uint, ci2: uint, nailDiameter: real, nailLength: real, numNails: uint)
		var cp1 = beam1:corner(ci1)
		var cp2 = beam1:corner(ci2)
		var edge = cp2 - cp1; edge:normalize()
		var normal = perp(edge)
		-- Ensure that the normal points in the right direction
		-- (away from the center of mass of beam2)
		if (beam2:centerOfMass() - cp1):dot(normal) >= 0.0 then
			normal = -normal
		end
		var penDepth1 = determinePenetrationDepth(beam2, cp1, normal, nailLength)
		var penDepth2 = determinePenetrationDepth(beam2, cp2, normal, nailLength)
		var nj1 = NailJoint.heapAlloc(beam1, beam2, cp1, normal, nailDiameter, penDepth1, numNails)
		var nj2 = NailJoint.heapAlloc(beam1, beam2, cp2, normal, nailDiameter, penDepth2, numNails)
		return nj1, nj2
	end

	terra NailJoint:applyForcesImpl() : {}
		var normalForce : ForceT
		var tangentForce : ForceT
		if self.objs[0].active or self.objs[1].active then
			var nCompress = genNonNegative1DForce(self.contactPoint, self.contactNormal)
			var nTension = genBoundedNonNegative1DForce(self.contactPoint, -self.contactNormal, self.maxPullForce)
			normalForce = nCompress; normalForce:combineWith(&nTension)
			tangentForce = genBounded1DForce(self.contactPoint, self.contactTangent, self.maxShearForce, 0.1) -- was 0.01
			-- normalForce = gen1DForce(self.contactPoint, self.contactNormal)
			-- tangentForce = gen1DForce(self.contactPoint, self.contactTangent)
			-- C.printf("normalForceMag: %g, maxPullForce: %g\n", ad.val(normalForce.vec):norm(), ad.val(self.maxPullForce))
			-- C.printf("tangentForceMag: %g, maxShearForce: %g\n", ad.val(tangentForce.vec):norm(), ad.val(self.maxShearForce))
		end
		if self.objs[0].active then
			self.objs[0]:applyForce(normalForce)
			self.objs[0]:applyForce(tangentForce)
		end
		if self.objs[1].active then
			normalForce.vec = -normalForce.vec
			tangentForce.vec = -tangentForce.vec
			-- Don't double-count the DOFs for these forces
			if self.objs[0].active then
				normalForce.dof = 0
				tangentForce.dof = 0
			end
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
	cross = cross,
	perp = perp
}



