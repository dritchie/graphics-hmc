
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
	terra ForceT:torque(centerOfRotation: Vec2)
		var d = self.pos - centerOfRotation
		-- C.printf("f: (%g, %g), d: (%g, %g)\n", 
		-- 	ad.val(self.vec(0)), ad.val(self.vec(1)), ad.val(d(0)), ad.val(d(1)))
		return d(0)*self.vec(1) - d(1)*self.vec(0)
	end
	if real==double then
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
	-- IMPORTANT!: apply only one force at any given location,
	--    i.e. do all the aggregation of forces and then pass the aggregated
	--    force to this method.
	terra RigidObjectT:applyForce(f: ForceT)
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
			------------------------------------------------------------------------------
			-- TODO: Not sure if computing residuals this way is the best way to do it...
			------------------------------------------------------------------------------
			-- Compute force residual
			var totalf = Vec2.stackAlloc(0.0)
			for i=0,self.forces.size do
				totalf = totalf + self.forces(i).vec
			end
			-- var fres = totalf:norm()
			-- var fres = ad.math.fabs(totalf(0)) + ad.math.fabs(totalf(1))
			@fresOut = totalf
			-- Compute torque residual
			var tsumsq = real(0.0)
			for i=0,cor.size do
				var indvidualResidual = real(0.0)
				for j=0,self.forces.size do
					var t = self.forces(j):torque(cor(i))
					indvidualResidual = indvidualResidual + t
				end
				-- tsumsq = tsumsq + indvidualResidual*indvidualResidual
				-- tsumsq = tsumsq + ad.math.fabs(indvidualResidual)
				tresOut:push(indvidualResidual)
			end
			-- var tres = ad.math.sqrt(tsumsq/cor.size)
			-- var tres = tsumsq/cor.size
			-- Return
			-- return fres, tres
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
	local RigidObjectT = RigidObject(real)
	local struct BeamT
	{
		endpoints: Vec2[2],
		width: real,
		density: real
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
	end
	-- Beams are active and visible by default
	terra BeamT:__construct(bot: Vec2, top: Vec2, w: real) : {}
		self:__construct(bot, top, w, true, true)
	end
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


-- A Cable is a rigid bar of zero area (and thus zero mass, so it is not affected by gravity)
-- The width member is only for rendering
-- Cables can only be under tension
local cableColor = colors.Black
local Cable = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local RigidObjectT = RigidObject(real)
	local struct CableT
	{
		endpoints: Vec2[2],
		width: real
	}
	inheritance.dynamicExtend(RigidObjectT, CableT)
	terra CableT:__construct(bot: Vec2, top: Vec2, w: real, active: bool, visible: bool) : {}
		-- Cables aren't affected by gravity
		RigidObjectT.__construct(self, active, false, visible)
		self.endpoints[0] = bot
		self.endpoints[1] = top
		self.width = w
	end
	-- Cables are inactive and visible by default
	terra CableT:__construct(bot: Vec2, top: Vec2, w: real) : {}
		self:__construct(bot, top, w, false, true)
	end
	-- (Inherit parent destructor)
	terra CableT:__copy(other: &CableT)
		RigidObjectT.__copy(self, other)
		self.endpoints[0] = other.endpoints[0]
		self.endpoints[1] = other.endpoints[1]
		self.width = other.width
	end
	terra CableT:newcopy() : &RigidObjectT
		var newcable = m.new(CableT)
		newcable:__copy(self)
		return newcable
	end
	inheritance.virtual(CableT, "newcopy")
	terra CableT:otherEndpoint(endpoint: uint)
		return (endpoint + 1) % 2
	end
	terra CableT:directionTowards(endpoint: uint)
		var vec = self.endpoints[endpoint] - self.endpoints[self:otherEndpoint(endpoint)]
		vec:normalize()
		return vec
	end
	terra CableT:directionAwayFrom(endpoint: uint)
		return -self:directionTowards(endpoint)
	end
	-- OpenGL drawing code
	if real == double then
		terra CableT:drawImpl() : {}
			drawBar(self.endpoints[0], self.endpoints[1], self.width, Color3d.stackAlloc([cableColor]))
		end
		inheritance.virtual(CableT, "drawImpl")
	end
	m.addConstructors(CableT)
	return CableT
end)


-- Ground is just an inactive, invisible Beam
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
	local forcePriorVariance = 100000000000.0

	local Vec2 = Vec(real, 2)
	local ForceT = Force(real)
	local BeamT = Beam(real)
	local CableT = Cable(real)

	-- Generate a 1-DOF force along a particular direction
	local gen1DForce = macro(function(pos, dir)
		return quote
			var mag = gaussian(0.0, forcePriorVariance, {structural=false, lowerBound=0.0, initialVal=0.0}) 
		in
			ForceT { mag*dir, pos, 1}
		end
	end)

	-- Generate a completely unconstrained 2-DOF force
	local genUnconstrained2DForce = macro(function(pos)
		return quote
			var x = gaussian(0.0, forcePriorVariance, {structural=false, initialVal=0.0}) 
			var y = gaussian(0.0, forcePriorVariance, {structural=false, initialVal=0.0}) 
		in
			ForceT { Vec2.stackAlloc(x, y), pos, 2}
		end
	end)

	-- Combine a bunch of forces together into one resultant force.
	-- Forces must all be applied at the same point.
	-- Keep track of DOF correctly.
	local terra combineForces(forces: &Vector(ForceT))
		util.assert(forces.size > 0, "Can't combine zero forces\n")
		var totalForce = forces(0).vec
		var pos = forces(0).pos
		var totalDOF = forces(0).dof
		for i=1,forces.size do
			util.assert(forces(i).pos == pos, "Can't combine forces applied at different points\n")
			var f = forces(i).vec
			if totalDOF == 1 and (not collinear(f, totalForce)) then
				totalDOF = 2
			end
			totalForce = totalForce + forces(i).vec
		end
		return ForceT{totalForce, pos, totalDOF}
	end

	----------------------------------

	-- RigidConnections transmit forces between one or more RigidObjects
	local struct RigidConnectionT {}
	-- Have to separate interface and implementation for "applyForces" because
	--    (as of now) pfn's can't be virtual.
	inheritance.purevirtual(RigidConnectionT, "applyForcesImpl", {}->{})
	terra RigidConnectionT:applyForces()
		self:applyForcesImpl()
	end
	RigidConnectionT.methods.applyForces = pmethod(RigidConnectionT.methods.applyForces)

	-- A Hinge applies a 2-DOF force to a Beam
	-- It can be connected to one or more Beams
	local struct HingeT
	{
		beams: Vector(&BeamT),
		location: Vec2
	}
	inheritance.dynamicExtend(RigidConnectionT, HingeT)
	terra HingeT:__construct(loc: Vec2)
		m.init(self.beams)
		self.location = loc
	end
	terra HingeT:__destruct()
		m.destruct(self.beams)
	end
	terra HingeT:addBeam(beam: &BeamT) self.beams:push(beam) end
	terra HingeT:applyForcesImpl() : {}
		for i=0,self.beams.size do
			if self.beams(i).active then
				self.beams(i):applyForce(genUnconstrained2DForce(self.location))
			end
		end
	end
	inheritance.virtual(HingeT, "applyForcesImpl")
	m.addConstructors(HingeT)

	-- A CablePin connects Cables to a Beam
	-- It applies 1-DOF (tensile) forces
	local struct CablePinT
	{
		beam: &BeamT,
		cables: Vector(&CableT),
		cableEndpoints: Vector(uint),
		location: Vec2
	}
	inheritance.dynamicExtend(RigidConnectionT, CablePinT)
	terra CablePinT:__construct(loc: Vec2, beam: &BeamT)
		m.init(self.cables)
		m.init(self.cableEndpoints)
		self.beam = beam
		self.location = loc
	end
	terra CablePinT:__destruct()
		m.destruct(self.cables)
		m.destruct(self.cableEndpoints)
	end
	-- Assumption is that the endpoint is colocated with the pin location
	terra CablePinT:addCable(cable: &CableT, endpoint: uint)
		self.cables:push(cable)
		self.cableEndpoints:push(endpoint)
	end
	terra CablePinT:applyForcesImpl() : {}
		-- Apply force from cables to beam
		-- Aggregate all independent forces into one total force
		if self.beam.active then
			var forces = [Vector(ForceT)].stackAlloc()
			for i=0,self.cables.size do
				var cable = self.cables(i)
				var endpoint = self.cableEndpoints(i)
				var dir = cable:directionAwayFrom(endpoint)
				forces:push(gen1DForce(self.location, dir))
			end
			self.beam:applyForce(combineForces(&forces))
			m.destruct(forces)
		end
		-- Apply force from beam to cables
		for i=0,self.cables.size do
			var cable = self.cables(i)
			if cable.active then
				var endpoint = self.cableEndpoints(i)
				var dir = cable:directionTowards(endpoint)
				cable:applyForce(gen1DForce(self.location, dir))
			end
		end
	end
	inheritance.virtual(CablePinT, "applyForcesImpl")
	m.addConstructors(CablePinT)

	-- A CableKnot connects Cables together
	-- It applies 1-DOF (tensile) forces
	local struct CableKnotT
	{
		cables: Vector(&CableT),
		cableEndpoints: Vector(uint),
		location: Vec2
	}
	inheritance.dynamicExtend(RigidConnectionT, CableKnotT)
	terra CableKnotT:__construct(loc: Vec2)
		m.init(self.cables)
		m.init(self.cableEndpoints)
		self.location = loc
	end
	terra CableKnotT:__destruct()
		m.destruct(self.cables)
		m.destruct(self.cableEndpoints)
	end
	-- Assumption is that the endpoint is colocated with the knot location
	terra CableKnotT:addCable(cable: &CableT, endpoint: uint)
		self.cables:push(cable)
		self.cableEndpoints:push(endpoint)
	end
	terra CableKnotT:applyForcesImpl() : {}
		var forces = [Vector(ForceT)].stackAlloc()
		-- Apply force to all cables
		for i=0,self.cables.size do
			var cable = self.cables(i)
			if cable.active then
				forces:clear()
				-- Apply forces from all other cables
				for j=0,self.cables.size do
					if i ~= j then
						var fcable = self.cables(j)
						var endpoint = self.cableEndpoints(j)
						var dir = fcable:directionAwayFrom(endpoint)
						forces:push(gen1DForce(self.location, dir))
					end
				end
				cable:applyForce(combineForces(&forces))
			end
		end
		m.destruct(forces)
	end
	inheritance.virtual(CableKnotT, "applyForcesImpl")
	m.addConstructors(CableKnotT)

	-- A Stacking connects two Beams (one atop the other)
	-- It applies normal forces at the contact vertices
	-- Assumes the Beams are axis-aligned and that endpoints[0]
	--    is below endpoints[1]
	local struct StackingT
	{
		bottom: &BeamT,
		top: &BeamT
	}
	inheritance.dynamicExtend(RigidConnectionT, StackingT)
	terra StackingT:__construct(bot: &BeamT, top: &BeamT)
		self.bottom = bot
		self.top = top
	end
	terra StackingT:applyForcesImpl() : {}
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
	inheritance.virtual(StackingT, "applyForcesImpl")
	m.addConstructors(StackingT)

	return 
	{
		RigidConnectionT = RigidConnectionT,
		HingeT = HingeT,
		CablePinT = CablePinT,
		CableKnotT = CableKnotT,
		StackingT = StackingT
	}

end



return
{
	Force = Force,
	RigidObject = RigidObject,
	Beam = Beam,
	Cable = Cable,
	Ground = Ground,
	RigidScene = RigidScene,
	Connections = Connections
}



