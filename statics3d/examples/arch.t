terralib.require("prob")


local s3dLib = terralib.require("s3dLib")
local m = terralib.require("mem")
local util = terralib.require("util")
local ad = terralib.require("ad")
local AutoPtr = terralib.require("autopointer")
local Vector = terralib.require("Vector")
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

	local frelTol = 0.01
	local trelTol = 0.01

	local mm = macro(function(x)
		return `0.001*x
	end)
	local Mm = function(x) return 0.001*x end
	local radians = macro(function(deg)
		return `deg*[math.pi]/180.0
	end)
	local Radians = function(deg) return deg*math.pi/180.0 end

	-- Parameters
	local numBlocks = 7
	local baseWidth = `[Mm(200.0)]
	local peakHeight = `[Mm(200.0)]
	local blockWidth = `[Mm(40.0)]
	local friction = math.sqrt(0.35)

	-- Returns the point and tangent at the interpolant value t
	local terra bezier(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: real)
		var t2 = t*t
		var t3 = t*t2
		var oneMinusT = 1.0 - t
		var oneMinusT2 = oneMinusT*oneMinusT
		var oneMinusT3 = oneMinusT*oneMinusT2
		var point = oneMinusT3*p0 + 3.0*oneMinusT2*t*p1 + 3.0*oneMinusT*t2*p2 + t3*p3
		var tangent = 3.0*oneMinusT2*(p1 - p0) + 6.0*oneMinusT*t*(p2 - p1) + 3.0*t2*(p3 - p2)
		return point, tangent
	end

	-- Generates an x-axis aligned perfect arch
	local terra perfectArch(groundShape: &QuadHex, outBlocks: &Vector(&Body))
		var groundHeight = groundShape:centroid()(2)
		var baseHalfWidth = 0.5*baseWidth
		var p0 = Vec3.stackAlloc(-baseHalfWidth, 0.0, groundHeight)
		var p1 = Vec3.stackAlloc(-baseHalfWidth, 0.0, peakHeight)
		var p2 = Vec3.stackAlloc(baseHalfWidth, 0.0, peakHeight)
		var p3 = Vec3.stackAlloc(baseHalfWidth, 0.0, groundHeight)
		var blockHalfWidth = 0.5*blockWidth
		var yaxis = blockHalfWidth*Vec3.stackAlloc(0.0, 1.0, 0.0)
		for i=0,numBlocks do
			var tlo = double(i)/numBlocks
			var thi = double(i+1)/numBlocks
			var plo, dlo = bezier(p0, p1, p2, p3, tlo)
			var phi, dhi = bezier(p0, p1, p2, p3, thi)
			-- y component is zero, so this is a quick-n-dirty way to find a perpendicular
			var nlo = dlo; util.swap(nlo(0), nlo(2)); nlo(0) = -nlo(0); nlo:normalize(); nlo = blockHalfWidth*nlo
			var nhi = dhi; util.swap(nhi(0), nhi(2)); nhi(0) = -nhi(0); nhi:normalize(); nhi = blockHalfWidth*nhi
			-- Build the block
			var fbl = plo - nlo - yaxis
			var fbr = plo - nlo + yaxis
			var bbl = plo + nlo - yaxis
			var bbr = plo + nlo + yaxis
			var ftl = phi - nhi - yaxis
			var ftr = phi - nhi + yaxis
			var btl = phi + nhi - yaxis
			var btr = phi + nhi + yaxis
			var blockShape = QuadHex.heapAlloc(fbl, fbr, ftl, ftr, bbl, bbr, btl, btr)
			var blockBody = Body.oak(blockShape)  
			blockBody.friction = friction
			outBlocks:push(blockBody)
		end
	end

	local perturbArch = pfn(terra(blocks: &Vector(&Body))
		for i=0,blocks.size-1 do
			var blockShape = [&QuadHex](blocks(i).shape)
			if i > 0 then
				var prevBlockShape = [&QuadHex](blocks(i-1).shape)
				-- Slide the bottom face by some amount
				var slideAmt = boundedUniform(-0.25, 1.25, {initialVal=0.5})
				blockShape:botFace():weld(prevBlockShape:topFace(), 0.5, slideAmt, false)
			end
			-- Grow/shrink the block length by some amount
			var axis = blockShape:topFace():centroid() - blockShape:botFace():centroid()
			var maxChange = 0.5
			var change = boundedUniform(-maxChange, maxChange, {initialVal=0.0})
			var mat = Mat4.translate(change*axis)
			blockShape:topFace():transform(&mat)
			-- Tilt the top face by some amount
			var maxTiltAmt = radians(30.0)
			blockShape:topShearX(boundedUniform(-maxTiltAmt, maxTiltAmt, {initialVal=0.0}))
			-- Make sure next block is connected after all of this
			var nextBlockShape = [&QuadHex](blocks(i+1).shape)
			nextBlockShape:botFace():weld(blockShape:topFace(), blockShape:topFace():centroid(), false)
		end
	end)

	local terra addArchToScene(blocks: &Vector(&Body), scene: &Scene)
		for i=0,blocks.size do
			scene.bodies:push(blocks(i))
		end
		-- Don't forget connections!
		var groundBody = scene.bodies(0)
		var groundShape = [&QuadHex](groundBody.shape)
		scene.connections:push(RectRectContact.heapAlloc(groundBody, blocks(0), groundShape:topFace(), [&QuadHex](blocks(0).shape):botFace(), false))
		scene.connections:push(RectRectContact.heapAlloc(groundBody, blocks:back(), groundShape:topFace(), [&QuadHex](blocks:back().shape):topFace(), false))
		for i=0,blocks.size-1 do
			var currBlockBody = blocks(i)
			var currBlockShape = [&QuadHex](currBlockBody.shape)
			var nextBlockBody = blocks(i+1)
			var nextBlockShape = [&QuadHex](nextBlockBody.shape)
			scene.connections:push(RectRectContact.heapAlloc(currBlockBody, nextBlockBody, currBlockShape:topFace(), nextBlockShape:botFace(), false))
		end
	end

	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		var camdist = mm(350.0)
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
		var groundBody = Body.oak(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)

		-- Build perfect arch
		var archBlocks = [Vector(&Body)].stackAlloc()
		perfectArch(groundShape, &archBlocks)

		-- Perturb arch
		perturbArch(&archBlocks)

		-- Add arch (and connections) to the scene
		addArchToScene(&archBlocks, &renderScene.scene)

		m.destruct(archBlocks)

		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)


		return renderScene
	end
end)


-- return probcomp(function()
-- 	local s3d = s3dLib()
-- 	util.importAll(s3d)

-- 	local gravityConst = `9.8
-- 	local upVector = global(Vec3d)
-- 	upVector:getpointer():__construct(0.0, 0.0, 1.0)

-- 	local frelTol = 0.01
-- 	local trelTol = 0.01

-- 	local mm = macro(function(x)
-- 		return `0.001*x
-- 	end)
-- 	local Mm = function(x) return 0.001*x end
-- 	local radians = macro(function(deg)
-- 		return `deg*[math.pi]/180.0
-- 	end)
-- 	local Radians = function(deg) return deg*math.pi/180.0 end

-- 	-- Parameters
-- 	local numBlocksPerHalfArch = 3
-- 	local minDim = Mm(20.0)
-- 	local maxDim = Mm(80.0)
-- 	local minAng = Radians(0.0)
-- 	local maxAng = Radians(30.0)
-- 	local margin = Mm(3.0)
-- 	local baseWidth = Mm(200.0)

-- 	local terra genBlockShape(w: real, h: real, d: real, shearx: real, sheary: real, topshear: real)
-- 		var boxShape = QuadHex.heapAlloc(); boxShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0), w, h, d)
-- 		boxShape:topShearX(topshear)
-- 		boxShape:shearX(shearx)
-- 		boxShape:shearY(sheary)
-- 		return boxShape
-- 	end

-- 	local genRandomBlockShape = pfn(terra(minShearX: double, maxShearX: double, minShearY: double, maxShearY: double, minTopShear: double, maxTopShear: double)
-- 		var w = boundedUniform(minDim, maxDim)
-- 		var h = boundedUniform(minDim, maxDim)
-- 		var d = boundedUniform(minDim, maxDim)
-- 		var shearx = boundedUniform(minShearX, maxShearX)
-- 		var sheary = boundedUniform(minShearY, maxShearY)
-- 		var topshear = boundedUniform(minTopShear, maxTopShear)
-- 		return genBlockShape(w, h, d, shearx, sheary, topshear)
-- 	end)

-- 	local genHalfArch = pfn(terra(scene: &Scene, basePoint: Vec3, isLeftHalf: bool)
-- 		-- Allowable bounds for shear angles change based on which half of the arch this is
-- 		-- Left half: positive shear angles, negative top shear angles
-- 		-- Right half: negative shear angles, positive top shear angles
-- 		var minShearAng : double = minAng
-- 		var maxShearAng : double = maxAng
-- 		var minTopShearAng : double = -maxAng
-- 		var maxTopShearAng : double = -minAng
-- 		if not isLeftHalf then
-- 			minShearAng = -maxAng
-- 			maxShearAng = -minAng
-- 			minTopShearAng = minAng
-- 			maxTopShearAng = maxAng
-- 		end
-- 		var minShearAngY = -maxAng
-- 		var maxShearAngY = maxAng

-- 		-- Make first block, stack it on the ground
-- 		var blocks = [Vector(&Body)].stackAlloc()
-- 		var ground = scene.bodies(0)
-- 		var blockShape = genRandomBlockShape(minShearAng, maxShearAng, minShearAngY, maxShearAngY, minTopShearAng, maxTopShearAng)
-- 		blockShape:stack([&QuadHex](ground.shape), basePoint)
-- 		var blockBody = Body.oak(blockShape)
-- 		blocks:push(blockBody)
-- 		scene.bodies:push(blockBody)
-- 		scene.connections:push(RectRectContact.heapAlloc(blockBody, ground, blockShape:botFace(), [&QuadHex](ground.shape):topFace(), false))

-- 		-- Make other blocks, stack them
-- 		for i=1,numBlocksPerHalfArch do
-- 			var prevBlock = blocks(i-1)
-- 			var prevShape = [&QuadHex](prevBlock.shape)
-- 			var blockShape = genRandomBlockShape(minShearAng, maxShearAng, minShearAngY, maxShearAngY, minTopShearAng, maxTopShearAng)
-- 			blockShape:stackRandom(prevShape, margin, false)
-- 			blockBody = Body.oak(blockShape)
-- 			blocks:push(blockBody)
-- 			scene.bodies:push(blockBody)
-- 			scene.connections:push(RectRectContact.heapAlloc(blockBody, prevBlock, blockShape:botFace(), prevShape:topFace(), false))
-- 		end

-- 		-- Return the last body
-- 		m.destruct(blocks)
-- 		return blockBody
-- 	end)

-- 	return terra()
-- 		-- Set up scene
-- 		var scene = Scene.stackAlloc(gravityConst, upVector)
-- 		var camera = Camera.stackAlloc()
-- 		var camdist = mm(350.0)
-- 		camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
-- 		camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(120.0))
-- 		camera.up = upVector
-- 		camera.znear = 0.01
-- 		camera.zfar = 10.0
-- 		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
-- 		var light = Light.stackAlloc()
-- 		renderScene:addLight(light)

-- 		-- Set up stuff in the scene --

-- 		-- Ground
-- 		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(1000.0), mm(1000.0), mm(10.0))
-- 		var groundBody = Body.oak(groundShape); groundBody.active = false
-- 		renderScene.scene.bodies:push(groundBody)

-- 		-- Make two halves of arch, then stitch them together
-- 		var leftBase = Vec3.stackAlloc(-baseWidth*0.5, 0.0, groundShape:topFace():centroid()(2))
-- 		var leftHalfTopBody = genHalfArch(&renderScene.scene, leftBase, true)
-- 		var rightBase = Vec3.stackAlloc(baseWidth*0.5, 0.0, groundShape:topFace():centroid()(2))
-- 		var rightHalfTopBody = genHalfArch(&renderScene.scene, rightBase, false)
-- 		var leftHalfTopFace = [&QuadHex](leftHalfTopBody.shape):topFace()
-- 		var rightHalfTopFace = [&QuadHex](rightHalfTopBody.shape):topFace()
-- 		rightHalfTopFace:weld(leftHalfTopFace, leftHalfTopFace:centroid(), false)
-- 		-- C.printf("rightHalfTopBody volume: %g\n", ad.val(rightHalfTopBody.shape:volume()))

-- 		-- Stablity
-- 		renderScene.scene:encourageStability(frelTol, trelTol)


-- 		return renderScene
-- 	end
-- end)



