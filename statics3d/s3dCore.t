terralib.require("prob")

-- -- Allow us to include stuff from the parent directory
-- package.path = "../?.t;" .. package.path 

local m = terralib.require("mem")
local util = terralib.require("util")
local inheritance = terralib.require("inheritance")
local templatize = terralib.requrie("templatize")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")
local Camera = terralib.require("glutils").Camera

-- Everything is defined in a probmodule, so all of this code
--    is safe to use in inference.
return probmodule(function(pcomp)


local Vec3 = Vec(real, 3)


----- SHAPES

-- A Shape is how we give form to a rigid body. It's responsible
--    only for high-level calculations (volume, centroid, etc.)
local struct Shape {}
inheritance.purevirtual(Shape, "volume", {}->{real})
inheritance.purevirtual(Shape, "centroid", {}->{Vec3})


-- A PrimitiveShape stores the raw geometry to represent a shape.
local struct PrimitiveShape
{
	-- Concrete subtypes should ensure that this points to something
	-- (Typically to a fixed-sized vertex array member)
	vertices: &Vec3
}
inheritance.dynamicExtend(Shape, PrimitiveShape)



-- An AggregateShape combines multiple Shapes together.
-- Provides general-purpose computations of volume and centroid
--    so that concrete subclasses don't have to.
local AggregateShape = templatize(function(numParts)
	local struct AggregateShapeT
	{
		shapes: (&Shape)[numParts]
	}
	inheritance.dynamicExtend(Shape, AggregateShapeT)

	local ctorSyms = {}
	for i=1,numParts do table.insert(ctorSyms, symbol(&Shape)) end
	terra AggregateShapeT:__construct([ctorSyms])
		[(function()
			local stmts = {}
			for i=1,numParts do table.insert(stmts, quote self.shapes[ [i-1] ] = [ctorSyms[i]] end)
			return stmts
		end)()]
	end

	-- Sum volumes of subcomponents.
	terra AggregateShapeT:volume() : real
		return [(function()
			local exp = `real(0.0)
			for i=1,numParts do exp = `[exp] + self.shapes[ [i-1] ]:volume() end
			return exp
		end)()]
	end
	inheritance.virtual(AggregateShapeT, "volume")

	-- Volume-weighted average of subcomponent centroids.
	terra AggregateShapeT:centroid() : Vec3
		return [(function()
			local volvars = {}
			for i=1,numParts do table.insert(volvars, symbol(real)) end
			local volassignments = {}
			for i=1,numParts do table.insert(volassignments, quote [volvars[i]] = self.shapes[ [i-1] ]:volume() end) end
			local centroidexp = `Vec3.stackAlloc(0.0, 0.0, 0.0)
			for i=1,numParts do centroidexp = `[centroidexp] + [volvars[i]] * self.shapes[ [i-1] ]:centroid() end
			local volsumexp = `real(0.0)
			for i=01,numParts do volsumexp = `[volsumexp] + [volvars[i]] end
			return quote
				[volassignments]
			in
				([centroidexp]) / ([volsumexp])
			end
		end)()]
	end
	inheritance.virtual(AggregateShapeT, "centroid")

	m.addConstructors(AggregateShapeT)
	return AggregateShapeT
end)


-- Primitive shapes are made out of Faces.
-- Indices should be wound counterclockwise.
--    (Positive-oriented normal points outside the shape)
-- Resulting polygon should be convex.
local Face = templatize(function(nverts)
	local struct FaceT
	{
		indices: uint[nverts],
		shape: &PrimitiveShape
		-- TODO: Compute and store face normal?
	}

	FaceT.methods.index = macro(function(self, i)
		util.luaAssertWithTrace(i < nverts,
			string.format("Cannot access index %u of a Face with only %u vertices", i, nverts))
		return `self.indices[i]
	end)

	FaceT.methods.vertex = macro(function(self, i)
		util.luaAssertWithTrace(i < nverts,
			string.format("Cannot access vertex %u of a Face with only %u vertices", i, nverts))
		return `self.shape.vertices[self.indices[i]]
	end)

	return FaceT
end)




----- FORCES

-- A Force represents some force vector applied at some point
local struct Force 
{
	force: Vec3,
	appPoint: Vec3,
	isExternal: bool
}

terra Force:torque(measurePoint: Vec3)
	var disp = self.appPoint - measurePoint
	return disp:cross(self.force)
end

-- Check if this force can be combined with another one
-- (i.e. they are collected and both internal/external)
terra Force:canCombineWith(f: &Force)
	return self.appPoint == f.appPoint and 
		   self.isExternal == f.isExternal
end

-- Add another force into this one (in place)
-- (Assumes that f can combine with this)
terra Force:combineWith(f: &Force)
	self.force = self.force + f.force
end

-- Attempt to add another force into this one (in place)
-- Return true if the add was successful (i.e. the two forces
--    could be combined), false otherwise.
terra Force:tryCombiningWith(f: &Force)
	if self:canCombineWith(f) then
		self:combineWith(f)
		return true
	else return false end
end

-- Utilities for generating forces
local randForceSD = `10000000000.0 	-- Effectively uniform
Force.methods.randomForce = pfn(terra(pos: Vec3)
	return Force
	{
		Vec3.stackAlloc(gaussian(0.0, randForceSD, {structural=false, initialVal=0.0}),
						gaussian(0.0, randForceSD, {structural=false, initialVal=0.0}),
						gaussian(0.0, randForceSD, {structural=false, initialVal=0.0})),
		pos,
		false
	}
end)
Force.methods.randomLineForce = pfn(terra(pos: Vec3, dir: Vec3)
	return Force
	{
		gaussian(0.0, randForceSD, {structural=false, initialVal=0.0}) * dir,
		pos,
		false
	}
end)
Force.methods.lowerBoundedRandomLineForce = pfn(terra(pos: Vec3, dir: Vec3, lo: real)
	return Force
	{
		gaussian(0.0, randForceSD, {structural=false, initialVal=0.0, lowerBound=lo}) * dir,
		pos,
		false
	}
end)
Force.methods.boundedRandomLineForce = pfn(terra(pos: Vec3, dir: Vec3, lo: real, hi: real)
	return Force
	{
		uniform(lo, hi, {structural=false, initialVal=0.0, lowerBound=lo, upperBound=hi}) * dir,
		pos,
		false
	}
end)



----- BODIES

-- A Body is a rigid body (duh)
-- It associates a shape with some physical properties
local struct Body
{
	shape: &Shape,
	-- TODO: Maybe have Body record a list of Connections it's involved in?
	forces: Vector(Force),
	active: bool,
	density: real,
	friction: real
}

terra Body:__construct(shape: &Shape, density: real, friction: real)
	self.shape = shape
	m.init(self.forces)
	self.active = true
	self.density = density
	self.friction = friction
end

-- No copy constructor.

terra Body:__destruct()
	m.destruct(self.forces)
end

terra Body:mass()
	return self.density * self.shape:volume()
end

-- We only support bodies of uniform density.
terra Body:centerOfMass()
	return self.shape:centroid()
end

-- Attempt to combine f with all currently-applied forces.
-- If all fail, add a new force
terra Body:applyForce(f: &Force)
	for i=0,self.forces.size do
		if self.forces(i):tryCombiningWith(f) then
			return
		end
	end
	self.forces:push(f)
end

terra Body:applyGravityForce(gravityConst: real, upVector: Vec3)
	var gravForce = Force { -gravityConst*self:mass()*upVector, self:centerOfMass(), true }
	self:applyForce(&gravForce)
end

-- Calculate residual net force and torque
terra Body:residuals()
	var fres = Vec3.stackAlloc(0.0, 0.0, 0.0)
	var tres = Vec3.stackAlloc(0.0, 0.0, 0.0)
	var com = self:centerOfMass()
	for i=0,self.forces.size do
		var f = self.forces:getPointer(i)
		fres = fres + f.force
		tres = tres + f:torque(com)
	end
	return fres, tres
end

m.addConstructors(Body)



----- CONNECTIONS

-- A Connection is a (surprise!) connection between multiple bodies that's responsible
--    for computing some internal forces between those bodies
local struct Connection {}
inheritance.purevirtual(Connection, "applyForcesImpl", {}->{})
-- This indirection is needed because we don't (yet?) have a way to support virtual pmethods.
terra Connection:applyForces()
	self:applyForcesImpl()
end
Connection.methods.applyForces = pmethod(Connection.methods.applyForces)




----- SCENES

-- A Scene should completely describe everything we need to query it for stability
local struct Scene
{
	bodies: Vector(&Body),
	connections: Vector(&Connection),
	gravityConst: real,
	upVector: Vec3,
	view: Camera
}

terra Scene:__construct(gravityConst: real, upVector: Vec3, cam: &Camera)
	m.init(self.bodies)
	m.init(self.connections)
	self.gravityConst = gravityConst
	self.upVector = upVector
	self.camera = @cam
end

terra Scene:__destruct()
	self.bodies:clearAndDelete()
	m.destruct(self.bodies)
	self.connections:clearAndDelete()
	m.destruct(self.connections)
end

-- Used as a normalizing factor for force residuals
terra Scene:avgExtForceMag()
	var avg = real(0.0)
	var numf = 0
	for i=0,self.bodies.size do
		var b = self.bodies(i)
		for j=0,b.forces.size do
			var f = b.forces:getPointer(j)
			if f.isExternal then
				avg = avg + f:norm()
				numf = numf + 1
			end
		end
	end
	return (1.0/numf) * avg
end

-- frelTol and trelTol are gaussian bandwidths, in "% of avg external force" units
local stabilityFactor = factorfn(terra(scene: &Scene, frelTol: real, trelTol: real)
	var f = real(0.0)
	var normConst = scene:avgExtForceMag()
	for i=0,scene.bodies.size do
		var fres, tres = scene.bodies(i):residuals()
		f = f + softeq(fres:norm()/normConst, 0.0, frelTol)
		f = f + softeq(tres:norm()/normConst, 0.0, trelTol)
	end
	return f
end)

terra Scene:encourageStability(frelTol: real, trelTol: real)
	-- Clear forces, apply gravity
	for i=0,scene.bodies.size do
		scene.bodies(i).forces:clear()
		scene.bodies(i):applyGravityForce(self.gravityConst, self.upVector)
	end

	-- Generate latent internal force variables
	for i=0,scene.connections.size do
		scene.connections:applyForces()
	end

	-- Add stability factor
	stabilityFactor(self, frelTol, trelTol)
end

m.addConstructors(Scene)


----- EXPORTS
return 
{
	Vec3 = Vec3,
	Shape = Shape,
	PrimitiveShape = PrimitiveShape,
	AggregateShape = AggregateShape,
	Face = Face,
	Force = Force,
	Body = Body,
	Connection = Connection,
	Scene = Scene
}

end)






