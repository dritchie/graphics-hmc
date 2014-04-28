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

	local frelTol = 0.02
	local trelTol = 0.02

	local mm = macro(function(x)
		return `0.001*x
	end)
	local radians = macro(function(deg)
		return `deg*[math.pi]/180.0
	end)


	-- Parameters
	local numArchBlocks = 7
	local minArchBaseWidth = `mm(100.0)
	local maxArchBaseWidth = `mm(300.0)
	local minArchPeakHeight = `mm(100.0)
	local maxArchPeakHeight = `mm(300.0)
	local minArchBlockWidth = `mm(10.0)
	local maxArchBlockWidth = `mm(60.0)
	local minArchSeparation = `mm(100.0)
	local maxArchSeparation = `mm(500.0)
	local minTowerHeight = `mm(50.0)
	local maxTowerHeight = `mm(300.0)
	local topWeightDim = `mm(60.0)
	local topWeightDensity = `2700.0    -- Stone of some kind?
	local topWeightTargetHeight = `mm(400.0)
	local topWeightTargetSoftness = `mm(10.0)



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

	-- Stores bodies representing an arch
	local struct Arch
	{
		blocks: Vector(&Body)
	}

	-- Constructs an arch with the specified parameters along the x axis, centered at (0,0) and rooted
	--    at z = 0
	terra Arch:__construct(numBlocks: uint, baseWidth: real, peakHeight: real, blockWidth: real, bodyGen: {&Shape}->{&Body})
		m.init(self.blocks)
		util.assert(numBlocks % 2 == 1, "Arch must have odd number of blocks (to have a flat top block)\n")
		var baseHalfWidth = 0.5*baseWidth
		var p0 = Vec3.stackAlloc(-baseHalfWidth, 0.0, 0.0)
		var p1 = Vec3.stackAlloc(-baseHalfWidth, 0.0, peakHeight)
		var p2 = Vec3.stackAlloc(baseHalfWidth, 0.0, peakHeight)
		var p3 = Vec3.stackAlloc(baseHalfWidth, 0.0, 0.0)
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
			var blockBody = bodyGen(blockShape)
			self.blocks:push(blockBody)
		end
	end

	-- Scene will assume ownership of blocks, so don't delete them
	terra Arch:__destruct()
		m.destruct(self.blocks)
	end

	-- Transform all block shapes
	terra Arch:transform(mat: &Mat4)
		for i=0,self.blocks.size do
			self.blocks(i).shape:transform(mat)
		end
	end

	terra Arch:leftFootFace()
		return [&QuadHex](self.blocks(0).shape):botFace()
	end

	terra Arch:rightFootFace()
		return [&QuadHex](self.blocks(self.blocks.size-1).shape):topFace()
	end

	terra Arch:middleBody()
		return self.blocks(self.blocks.size/2)
	end

	terra Arch:middleTopFace()
		return [&QuadHex](self:middleBody().shape):backFace()
	end

	-- Add block bodies (and relevant connections) to the scene
	-- Assumes body 0 in the scene is the ground
	terra Arch:addToScene(scene: &Scene)
		for i=0,self.blocks.size do
			scene.bodies:push(self.blocks(i))
		end
		-- Connections
		var groundBody = scene.bodies(0)
		var groundShape = [&QuadHex](groundBody.shape)
		scene.connections:push(RectRectContact.heapAlloc(groundBody, self.blocks(0), groundShape:topFace(), self:leftFootFace(), false))
		scene.connections:push(RectRectContact.heapAlloc(groundBody, self.blocks:back(), groundShape:topFace(), self:rightFootFace(), false))
		for i=0,self.blocks.size-1 do
			var currBlockBody = self.blocks(i)
			var currBlockShape = [&QuadHex](currBlockBody.shape)
			var nextBlockBody = self.blocks(i+1)
			var nextBlockShape = [&QuadHex](nextBlockBody.shape)
			scene.connections:push(RectRectContact.heapAlloc(currBlockBody, nextBlockBody, currBlockShape:topFace(), nextBlockShape:botFace(), false))
		end
	end

	m.addConstructors(Arch)


	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		var camdist = mm(500.0)
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
		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(2000.0), mm(2000.0), mm(10.0))
		var groundBody = Body.oak(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)

		-- Arches
		var baseWidth = boundedUniform(minArchBaseWidth, maxArchBaseWidth)
		var peakHeight = boundedUniform(minArchPeakHeight, maxArchPeakHeight)
		var blockWidth = boundedUniform(minArchBlockWidth, maxArchBlockWidth)
		var arch1 = Arch.stackAlloc(numArchBlocks, baseWidth, peakHeight, blockWidth, Body.oak)
		var arch2 = Arch.stackAlloc(numArchBlocks, baseWidth, peakHeight, blockWidth, Body.oak)
		----
		var groundDisp = Vec3.stackAlloc(0.0, 0.0, groundShape:topFace():vertex(0)(2))
		var archsep = boundedUniform(minArchSeparation, maxArchSeparation)
		var archDisp = Vec3.stackAlloc(0.0, 0.5*archsep, 0.0)
		var arch1Xform = Mat4.translate(groundDisp - archDisp)
		var arch2Xform = Mat4.translate(groundDisp + archDisp)
		arch1:transform(&arch1Xform)
		arch2:transform(&arch2Xform)
		arch1:addToScene(&renderScene.scene)
		arch2:addToScene(&renderScene.scene)

		-- Cross beam
		var beamThickness = boundedUniform(minArchBlockWidth, maxArchBlockWidth)
		var crossBeamShape = Box.heapAlloc()
		var actualArchHeight = arch1:middleTopFace():vertex(0)(2)
		crossBeamShape:makeBox(Vec3.stackAlloc(0.0, 0.0, actualArchHeight + 0.5*beamThickness), beamThickness, archsep + 0.5*blockWidth, beamThickness)
		var crossBeamBody = Body.oak(crossBeamShape)
		renderScene.scene.bodies:push(crossBeamBody)
		renderScene.scene.connections:push(RectRectContact.heapAlloc(crossBeamBody, arch1:middleBody(), crossBeamShape:botFace(), arch1:middleTopFace(), false))
		renderScene.scene.connections:push(RectRectContact.heapAlloc(crossBeamBody, arch2:middleBody(), crossBeamShape:botFace(), arch2:middleTopFace(), false))

		-- Tower
		var towerThickness = boundedUniform(minArchBlockWidth, maxArchBlockWidth)
		var towerHeight = boundedUniform(minTowerHeight, maxTowerHeight)
		var towerShape = Box.heapAlloc()
		towerShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0), towerThickness, towerThickness, towerHeight)
		towerShape:stack(crossBeamShape, 0.5, 0.5, true)
		var towerBody = Body.oak(towerShape)
		renderScene.scene.bodies:push(towerBody)
		renderScene.scene.connections:push(RectRectContact.heapAlloc(towerBody, crossBeamBody, towerShape:botFace(), crossBeamShape:topFace(), false))

		-- Weight
		var weightShape = Box.heapAlloc(); weightShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0), topWeightDim, topWeightDim, topWeightDim)
		weightShape:stack(towerShape, 0.5, 0.5, true)
		var weightBody = Body.oak(weightShape)
		weightBody.density = topWeightDensity
		renderScene.scene.bodies:push(weightBody)
		renderScene.scene.connections:push(RectRectContact.heapAlloc(weightBody, towerBody, weightShape:botFace(), towerShape:topFace(), false))



		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)


		-- Target height
		var topWeightActualHeight = weightShape:centroid()(2)
		factor(softeq(topWeightActualHeight, topWeightTargetHeight, topWeightTargetSoftness))


		return renderScene
	end
end)








