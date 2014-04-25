
local m = terralib.require("mem")
local staticsUtils = terralib.require("staticsUtils")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")

local C = terralib.includecstring [[
#include "math.h"
#include "stdio.h"
]]

-- Include 2D rigid body dynamics code
local cp = terralib.require("chipmunk")


local RigidSceneT = staticsUtils.RigidScene(double)
local BeamT = staticsUtils.Beam(double)
local Vec2d = Vec(double, 2)


-- A simulation of a RigidScene (composed of Beams)
-- Just associates a RigidScene with a cpSpace
local struct RigidSceneSim
{
	scene: &RigidSceneT,
	space: &cp.cpSpace,
	-- Bodies and shapes that we allocated that need destructing
	bodies: Vector(&cp.cpBody),
	shapes: Vector(&cp.cpShape)
}

-- Convert between my Vec2d and chipmunk's cpVect
local vec2dToCpVect = macro(function(v)
	return `cp.cpv(v(0), v(1))
end)
local cpVectToVec2d = macro(function(v)
	return `Vec2d.stackAlloc(v.x, v.y)
end)

-- Have to explicitly provide the cofficient of friction, since my code defines it per-contact,
--    whereas chipmunk expects it per-object.
terra RigidSceneSim:__construct(scene: &RigidSceneT, gravConst: double, friction: double)
	self.scene = scene
	m.init(self.bodies)
	m.init(self.shapes)

	self.space = cp.cpSpaceNew()
	cp.cpSpaceSetGravity(self.space, cp.cpv(0.0, -gravConst))

	-- Per-contact friction is the product of the two per-object frictions, so we need
	--   to do this.
	var frictionPerObj = C.sqrt(friction)

	-- Storage for the vertices of the beam polygons
	var vertsPreArray : Vec2d[4]
	var vertsArray : cp.cpVect[4]

	-- Add bodies w/ collision shapes for all the objects (Beams) in the scene.
	for i=0,scene.objects.size do
		var beam = [&BeamT](self.scene.objects(i))
		-- Transform beam vertices to be local to the beam's center of mass
		var com = beam:centerOfMass()
		vertsPreArray[0] = beam:corner(0) - com
		vertsPreArray[1] = beam:corner(1) - com
		vertsPreArray[2] = beam:corner(2) - com
		vertsPreArray[3] = beam:corner(3) - com
		-- Convert the vertices of the beam polygon into chipmunk's format
		-- (Start with ordering 0,1,2,3, and switch to order 1,0,3,2 if the winding order is wrong)
		vertsArray[0] = vec2dToCpVect(vertsPreArray[0])
		vertsArray[1] = vec2dToCpVect(vertsPreArray[1])
		vertsArray[2] = vec2dToCpVect(vertsPreArray[2])
		vertsArray[3] = vec2dToCpVect(vertsPreArray[3])
		if cp.cpPolyValidate(vertsArray, 4) == 0 then
			vertsArray[0] = vec2dToCpVect(vertsPreArray[1])
			vertsArray[1] = vec2dToCpVect(vertsPreArray[0])
			vertsArray[2] = vec2dToCpVect(vertsPreArray[3])
			vertsArray[3] = vec2dToCpVect(vertsPreArray[2])
		end
		-- If the object is inactive, use a static body
		var body : &cp.cpBody
		-- Otherwise, make a dynamic body with the correct mass / moment
		if beam.active then
			var mass = beam:mass()
			var moment = cp.cpMomentForPoly(mass, 4, vertsArray, cp.cpv(0.0, 0.0))
			body = cp.cpSpaceAddBody(self.space, cp.cpBodyNew(mass, moment))
		else
			body = cp.cpBodyNewStatic()
		end
		cp.cpBodySetPos(body, vec2dToCpVect(com))
		self.bodies:push(body)
		-- Create and attach the collision shape
		var shape = cp.cpSpaceAddShape(self.space, cp.cpPolyShapeNew(body, 4, vertsArray, cp.cpv(0.0, 0.0)))
		self.shapes:push(shape)
		cp.cpShapeSetFriction(shape, frictionPerObj)
	end
end

terra RigidSceneSim:__destruct()
	self.bodies:clearAndDelete()
	self.shapes:clearAndDelete()
	cp.cpSpaceFree(self.space)
end

-- Step the dynamics, then spit out an updated copy
--    of the RigidScene.
terra RigidSceneSim:step(timestep: double)
	cp.cpSpaceStep(self.space, timestep)

	-- Update the RigidScene with new state
	var newScene = m.copy(@self.scene)
	for i=0,newScene.objects.size do
		var obj = newScene.objects(i)
		if obj.active then
			var beam = [&BeamT](obj)
			var com = beam:centerOfMass()
			var pos = cp.cpBodyGetPos(self.bodies(i))
			var displacement = cpVectToVec2d(pos) - com
			var ang = cp.cpBodyGetAngle(self.bodies(i))
			beam:transform(displacement, ang)
		end
	end
	return newScene
end

m.addConstructors(RigidSceneSim)


return
{
	RigidSceneSim = RigidSceneSim
}


