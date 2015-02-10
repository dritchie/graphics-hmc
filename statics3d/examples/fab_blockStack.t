require("prob")


local s3dLib = require("s3dLib")
local util = require("util")
local ad = require("ad")
local AutoPtr = require("autopointer")
local Vec = require("linalg").Vec
local Vec3d = Vec(double, 3)
local glutils = require("glutils")
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

	local frelTol = 0.01
	local trelTol = 0.01

	local mm = macro(function(x)
		return `0.001*x
	end)
	local radians = macro(function(deg)
		return `deg*[math.pi]/180.0
	end)

	-- Parameters
	local minDim = `mm(20.0)
	local maxDim = `mm(80.0)
	local minHeight = `mm(40.0)
	local maxHeight = `mm(75.0)
	local minWidth = `mm(15.0)
	local maxWidth = `mm(75.0)
	local depth = `mm(38.0)
	local maxAng = `radians(30.0)
	local margin = `mm(10.0)	

	local genRandomBlockShape = pfn(terra()
		var height = boundedUniform(minHeight, maxHeight)
		var width = boundedUniform(minWidth, maxWidth)
		var boxShape = QuadHex.heapAlloc(); boxShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0), width, depth, height)
		boxShape:topShearX(boundedUniform(-maxAng, maxAng))
		boxShape:shearX(boundedUniform(-maxAng, maxAng))
		return boxShape
	end)

	local numBlocks = 6
	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		var camdist = mm(450.0)
		camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
		camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(120.0))
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		renderScene:addLight(light)

		-- Set up stuff in the scene --

		-- Ground
		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(1000.0), mm(1000.0), mm(10.0))
		var groundBody = Body.poplar(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)

		-- First block
		var boxShape = genRandomBlockShape()
		boxShape:stack(groundShape, 0.5, 0.5, true)
		var boxBody = Body.poplar(boxShape)
		renderScene.scene.bodies:push(boxBody)
		renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, groundBody, boxShape:botFace(), groundShape:topFace(), false))

		-- Subsequent blocks
		for i=1,numBlocks do
			var bodyIndex = i+1
			var prevBody = renderScene.scene.bodies(bodyIndex-1)
			var prevShape = [&QuadHex](prevBody.shape)
			var boxShape = genRandomBlockShape()
			boxShape:stackRandomX(prevShape, margin, false)
			boxBody = Body.poplar(boxShape)
			renderScene.scene.bodies:push(boxBody)
			renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, prevBody, boxShape:botFace(), prevShape:topFace(), false))
		end

		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)


		return renderScene
	end
end)



