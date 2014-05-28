terralib.require("prob")


local s3dLib = terralib.require("s3dLib")
local util = terralib.require("util")
local ad = terralib.require("ad")
local AutoPtr = terralib.require("autopointer")
local Vec = terralib.require("linalg").Vec
local Vec3d = Vec(double, 3)
local glutils = terralib.require("glutils")
util.importEntries(glutils, "Camera", "Light")


local C = terralib.includecstring [[
#include "stdio.h"
]]


return probcomp(function()
	local s3d = s3dLib()
	util.importAll(s3d)

	local gravityConst = `9.8
	local upVector = global(Vec3d)
	upVector:getpointer():__construct(0.0, 0.0, 1.0)

	local frelTol = 0.005
	local trelTol = 0.005

	local mm = macro(function(x)
		return `0.001*x
	end)
	local radians = macro(function(deg)
		return `deg*[math.pi]/180.0
	end)

	local genRandomBlockShape = pfn(terra(minDim: double, maxDim: double, minAng: double, maxAng: double, index: int)
		var boxShape = QuadHex.heapAlloc(); boxShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0),
			boundedUniform(minDim, maxDim), boundedUniform(minDim, maxDim), boundedUniform(minDim, maxDim))
		if index % 2 == 0 then
			boxShape:shearY(boundedUniform(minAng, maxAng))
			boxShape:topShearX(boundedUniform(minAng, maxAng))
			boxShape:shearX(boundedUniform(minAng, maxAng))
		else
			boxShape:shearX(boundedUniform(minAng, maxAng))
			boxShape:topShearY(boundedUniform(minAng, maxAng))
			boxShape:shearY(boundedUniform(minAng, maxAng))
		end
		-- boxShape:assertFacePlanarity()
		return boxShape
	end)

	local numBlocks = 5
	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		var camdist = mm(450.0)
		camera.eye = Vec3d.stackAlloc(0.75*camdist, -camdist, 0.5*camdist)
		camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(150.0))
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		renderScene:addLight(light)

		-- Parameters
		var minDim = mm(20.0)
		var maxDim = mm(80.0)
		var minAng = -radians(30.0)
		var maxAng = radians(30.0)
		var margin = mm(10.0)
		-- var minDim = mm(39.99)
		-- var maxDim = mm(40.01)
		-- var minAng = -radians(0.01)
		-- var maxAng = radians(0.01)
		-- var margin = mm(10.0)

		-- Set up stuff in the scene --

		-- Ground
		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(1000.0), mm(1000.0), mm(10.0))
		var groundBody = Body.oak(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)

		-- First block
		var boxShape = genRandomBlockShape(minDim, maxDim, minAng, maxAng, 0)
		boxShape:stack(groundShape, 0.5, 0.5, true)
		var boxBody = Body.oak(boxShape)
		renderScene.scene.bodies:push(boxBody)
		renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, groundBody, boxShape:botFace(), groundShape:topFace(), false))

		-- Subsequent blocks
		for i=1,numBlocks do
			var bodyIndex = i+1
			var prevBody = renderScene.scene.bodies(bodyIndex-1)
			var prevShape = [&QuadHex](prevBody.shape)
			var boxShape = genRandomBlockShape(minDim, maxDim, minAng, maxAng, i)
			boxShape:stack(prevShape, 0.5, 0.5, true)
			-- boxShape:stackRandom(prevShape, margin, false)
			boxShape:alignStacked(prevShape)	-- IMPORTANT!
			boxBody = Body.oak(boxShape)
			renderScene.scene.bodies:push(boxBody)
			renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, prevBody, boxShape:botFace(), prevShape:topFace(), false))
			-- renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, prevBody, boxShape:botFace(), prevShape:topFace(), true))

			-- Enforce stability at every intermediate state of construction.
			renderScene.scene:encourageStability(frelTol, trelTol)
		end

		return renderScene
	end
end)



