terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand = terralib.require("prob.random")
local gl = terralib.require("gl")

local GradientAscent = terralib.require("gradientAscent")

local staticsUtils = terralib.require("staticsUtils")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)


----------------------------------

local lerp = macro(function(lo, hi, t)
	return `(1.0-t)*lo + t*hi
end)
local expinterp = macro(function(lo, hi, t)
	return `ad.math.exp( (1.0-t)*ad.math.log(lo) + t*ad.math.log(hi) )
end)

----------------------------------

local staticsModel = probcomp(function()
	local gravityConstant = `9.8

	local Vec2 = Vec(real, 2)
	local ForceT = staticsUtils.Force(real)
	local RigidObjectT = staticsUtils.RigidObject(real)
	local BeamT = staticsUtils.Beam(real)
	local Ground = staticsUtils.Ground(real)
	local RigidSceneT = staticsUtils.RigidScene(real)
	local Connections = staticsUtils.Connections()

	local polar2rect = macro(function(r, theta)
		return `Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end)

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)

	----------------------------------

	-- Enforce static equilibrium of a scene given some connections
	local enforceStability = pfn(terra(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection))
		-- Apply internal forces
		for i=0,connections.size do
			connections(i):applyForces()
		end
		-- Apply gravity to everything affected by it
		var down = Vec2.stackAlloc(0.0, -1.0)
		for i=0,scene.objects.size do
			var obj = scene.objects(i)
			if obj.affectedByGravity then
				var gforce = gravityConstant * obj:mass() * down
				obj:applyForce(ForceT{gforce, obj:centerOfMass(), 0})
			end
		end
		-- Calculate residuals and apply factors
		var fres = Vec2.stackAlloc(0.0, 0.0)
		var tres = [Vector(real)].stackAlloc()
		for i=0,scene.objects.size do
			if scene.objects(i).active then
				tres:clear()
				scene.objects(i):calculateResiduals(&fres, &tres)
				-- factor(softEq(fres(0), 0.0, 1.0))
				manifold(fres(0), 1.0)
				-- factor(softEq(fres(1), 0.0, 1.0))
				manifold(fres(1), 1.0)
				for j=0,tres.size do
					-- factor(softEq(tres(j), 0.0, 1.0))
					manifold(tres(j), 1.0)
				end
			end
		end
		m.destruct(tres)
	end)

	-- Define a simple scene with a beam rotating on a hinge
		--    attached to the ground. There's also a cable connecting
		--    the beam to the ground. The cable is inactive; it applies
		--    forces, but we don't solve for its equilibrium.
	local simpleBeamHingeCableScene = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)

		var hingeY = groundHeight
		var hinge = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth*0.5, hingeY))
		
		var beamBot = Vec2.stackAlloc(hinge.location(0), hingeY)
		var beamLen = 20.0
		-- var beamAngle = [math.pi/4]
		var beamAngle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beamTop = beamBot + polar2rect(beamLen, beamAngle)
		var beamWidth = 3.0
		var beam = BeamT.heapAlloc(beamBot, beamTop, beamWidth)
		hinge:addBeam(beam)

		var groundPinLoc = Vec2.stackAlloc(sceneWidth*0.2, groundHeight)
		var beamPinLoc = beamTop
		var cableWidth = 0.5
		var cable = Connections.Cable.heapAlloc(groundPinLoc, beamPinLoc, ground, beam, cableWidth)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam)
		scene.objects:push(cable:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge)
		connections:push(cable)

		enforceStability(&scene, &connections)

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	-- Slightly more complicated example: two hinged beams connected at the top
	--    by a cable.
	local twoBeamsConnectedByCableScene = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
		var beamLength = 20.0
		var beamWidth = 3.0
		var beam1angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beam2angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		-- var beam1angle = [3*math.pi/4]
		-- var beam2angle = [math.pi/4]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth)
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth)
		hinge1:addBeam(beam1)
		hinge2:addBeam(beam2)

		var cable = Connections.Cable.heapAlloc(beam1.endpoints[1], beam2.endpoints[1], beam1, beam2, 0.5)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(cable:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(cable)

		enforceStability(&scene, &connections)

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	-- Slighty wacky "bridge" with one link
	local singleLinkWackyBridge = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
		var beamLength = 20.0
		var beamWidth = 3.0
		var beam1angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beam2angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		-- var beam1angle = [0.6*math.pi]
		-- var beam2angle = [0.1*math.pi]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth)
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth)
		hinge1:addBeam(beam1)
		hinge2:addBeam(beam2)
		var platformHeight = uniform(10.0, 30.0, {structural=false, lowerBound=10.0, upperBound=30.0})
		-- var platformHeight = 10.0
		var platformWidth = 3.0
		var platformLength = 10.0
		var platform = BeamT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.5 - platformLength*0.5, platformHeight + platformWidth*0.5),
									   Vec2.stackAlloc(sceneWidth*0.5 + platformLength*0.5, platformHeight + platformWidth*0.5),
									   platformWidth)
		var cableWidth = 0.5
		var cable1 = Connections.Cable.heapAlloc(beam1.endpoints[1], platform.endpoints[0], beam1, platform, cableWidth)
		var cable2 = Connections.Cable.heapAlloc(beam2.endpoints[1], platform.endpoints[1], beam2, platform, cableWidth)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(platform)
		scene.objects:push(cable1:createProxy())
		scene.objects:push(cable2:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(cable1)
		connections:push(cable2)

		enforceStability(&scene, &connections)

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	-- Multi-link wacky bridge with rotatable links
	local multiLinkWackyBridge = pfn(terra(numLinks: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Set up end support beams
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(0.2*sceneWidth, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(0.8*sceneWidth, groundHeight))
		var beamLength = 40.0
		var beamWidth = 4.0
		var beam1angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beam2angle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		-- var beam1angle = [math.pi/2]
		-- var beam2angle = [math.pi/2]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth)
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth)
		hinge1:addBeam(beam1)
		hinge2:addBeam(beam2)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		connections:push(hinge1)
		connections:push(hinge2)

		-- Create platforms
		var platforms = [Vector(&BeamT)].stackAlloc()
		var platxmin = beam1.endpoints[0](0)
		var platxmax = beam2.endpoints[0](0)
		var platxrange = platxmax - platxmin
		for i=0,numLinks do
			var t = ((i+1)/double(numLinks+1))
			var centerx = platxmin + t*platxrange
			var centery = uniform(10.0, 40.0, {structural=false, lowerBound=10.0, upperBound=40.0})
			var center = Vec2.stackAlloc(centerx, centery)
			-- var rot = 0.0
			var rot = gaussian(0.0, [math.pi/20], {structural=false})
			-- var rot = uniform([-math.pi/6], [math.pi/6], {structural=false, lowerBound=[-math.pi/6], upperBound=[math.pi/6]})
			var width = 1.5
			var length = 4.0
			var longAxis = polar2rect(length, rot)
			var platform = BeamT.heapAlloc(center - longAxis, center+longAxis, width)
			platforms:push(platform)
			scene.objects:push(platform)
		end

		-- Link everything together with cables
		var startBeams = [Vector(&BeamT)].stackAlloc()
		var startEndpoints = [Vector(uint)].stackAlloc()
		var endBeams = [Vector(&BeamT)].stackAlloc()
		var endEndpoints = [Vector(uint)].stackAlloc()
		startBeams:push(beam1)
		startEndpoints:push(1)
		for i=0,platforms.size do
			endBeams:push(platforms(i))
			endEndpoints:push(0)
			startBeams:push(platforms(i))
			startEndpoints:push(1)
		end
		endBeams:push(beam2)
		endEndpoints:push(1)
		m.destruct(platforms)
		for i=0,startBeams.size do
			var width = 0.4
			var cable = Connections.Cable.heapAlloc(startBeams(i).endpoints[startEndpoints(i)],
										 			endBeams(i).endpoints[endEndpoints(i)],
										 			startBeams(i), endBeams(i), width)
			scene.objects:push(cable:createProxy())
			connections:push(cable)
		end
		m.destruct(startBeams)
		m.destruct(startEndpoints)
		m.destruct(endBeams)
		m.destruct(endEndpoints)

		enforceStability(&scene, &connections)

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	-- Multi-link bridge with rotatable links and sliding support columns
	local multiLinkSlidingBridge = pfn(terra(numLinks: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Set up end support beams
		var beamSepWidth = uniform(0.5*sceneWidth, 0.99*sceneWidth, {structural=false, lowerBound=0.5*sceneWidth, upperBound=0.99*sceneWidth})
		var beam1x = 0.5*sceneWidth - 0.5*beamSepWidth
		var beam2x = 0.5*sceneWidth + 0.5*beamSepWidth
		var beam1bot = Vec2.stackAlloc(beam1x, groundHeight)
		var beam2bot = Vec2.stackAlloc(beam2x, groundHeight)
		var beamLength = 40.0
		var beamWidth = 4.0
		var beam1 = BeamT.heapAlloc(beam1bot, beam1bot + Vec2.stackAlloc(0.0, beamLength), beamWidth, false, true)
		var beam2 = BeamT.heapAlloc(beam2bot, beam2bot + Vec2.stackAlloc(0.0, beamLength), beamWidth, false, true)
		scene.objects:push(beam1)
		scene.objects:push(beam2)

		-- Create platforms
		var platforms = [Vector(&BeamT)].stackAlloc()
		var platxmin = beam1.endpoints[0](0)
		var platxmax = beam2.endpoints[0](0)
		var platxrange = platxmax - platxmin
		for i=0,numLinks do
			var t = ((i+1)/double(numLinks+1))
			var centerx = platxmin + t*platxrange
			var centery = uniform(5.0, 40.0, {structural=false, lowerBound=5.0, upperBound=40.0})
			var center = Vec2.stackAlloc(centerx, centery)
			-- var rot = 0.0
			-- var rot = gaussian(0.0, [math.pi/20], {structural=false})
			var rot = uniform([-math.pi/4], [math.pi/4], {structural=false, lowerBound=[-math.pi/4], upperBound=[math.pi/4]})
			var width = 1.5
			var length = 4.0
			var longAxis = polar2rect(length, rot)
			var platform = BeamT.heapAlloc(center - longAxis, center+longAxis, width)
			platforms:push(platform)
			scene.objects:push(platform)
		end

		-- Link everything together with cables
		var startBeams = [Vector(&BeamT)].stackAlloc()
		var startEndpoints = [Vector(uint)].stackAlloc()
		var endBeams = [Vector(&BeamT)].stackAlloc()
		var endEndpoints = [Vector(uint)].stackAlloc()
		startBeams:push(beam1)
		startEndpoints:push(1)
		for i=0,platforms.size do
			endBeams:push(platforms(i))
			endEndpoints:push(0)
			startBeams:push(platforms(i))
			startEndpoints:push(1)
		end
		endBeams:push(beam2)
		endEndpoints:push(1)
		m.destruct(platforms)
		for i=0,startBeams.size do
			var width = 0.4
			var cable = Connections.Cable.heapAlloc(startBeams(i).endpoints[startEndpoints(i)],
										 			endBeams(i).endpoints[endEndpoints(i)],
										 			startBeams(i), endBeams(i), width)
			scene.objects:push(cable:createProxy())
			connections:push(cable)
		end
		m.destruct(startBeams)
		m.destruct(startEndpoints)
		m.destruct(endBeams)
		m.destruct(endEndpoints)

		enforceStability(&scene, &connections)

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	-- Single-spar cable-stayed bridge
	local cableStayedBridge = pfn(terra(numCablePairs: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Platform
		var platformWidth = uniform(0.5*sceneWidth, 0.99*sceneWidth, {structural=false, lowerBound=0.5*sceneWidth, upperBound=0.99*sceneWidth})
		var platformHeight = uniform(0.1*sceneWidth, 0.3*sceneWidth, {structural=false, lowerBound=0.1*sceneWidth, upperBound=0.3*sceneWidth})
		var platformLeft = Vec2.stackAlloc(0.5*sceneWidth - 0.5*platformWidth, platformHeight)
		var platformRight = Vec2.stackAlloc(0.5*sceneWidth + 0.5*platformWidth, platformHeight)
		var platformCenter = 0.5*(platformLeft + platformRight)
		var platformThickness = 4.0
		var platform = BeamT.heapAlloc(platformLeft, platformRight, platformThickness)
		scene.objects:push(platform)

		-- Support beams
		var beamWidth = 4.0
		var beamBotY = groundHeight
		var beamTopY = platformHeight - 0.5*platformThickness
		var beam1xlo = platformLeft(0) + 0.5*beamWidth
		var beam1xhi = platformCenter(0) - 0.5*beamWidth
		-- var beam1x = uniform(beam1xlo, beam1xhi, {structural=false, lowerBound=beam1xlo, upperBound=beam1xhi})
		var beam1x = beam1xlo
		var beam2xlo = platformCenter(0) + 0.5*beamWidth
		var beam2xhi = platformRight(0) - 0.5*beamWidth
		-- var beam2x = uniform(beam2xlo, beam2xhi, {structural=false, lowerBound=beam2xlo, upperBound=beam2xhi})
		var beam2x = beam2xhi
		var beam1Bot = Vec2.stackAlloc(beam1x, beamBotY)
		var beam1Top = Vec2.stackAlloc(beam1x, beamTopY)
		var beam2Bot = Vec2.stackAlloc(beam2x, beamBotY)
		var beam2Top = Vec2.stackAlloc(beam2x, beamTopY)
		var beam1 = BeamT.heapAlloc(beam1Bot, beam1Top, beamWidth)
		var beam2 = BeamT.heapAlloc(beam2Bot, beam2Top, beamWidth)
		scene.objects:push(beam1)
		scene.objects:push(beam2)

		-- Spar
		var sparWidth = 4.0
		var sparLength = uniform(0.5*platformWidth, platformWidth, {structural=false, lowerBound=0.5*platformWidth, upperBound=platformWidth})
		var sparBotParam = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})
		var sparLowerBound = Vec2.stackAlloc(platformLeft(0) + 0.5*sparWidth, platformLeft(1))
		var sparUpperBound = Vec2.stackAlloc(platformRight(0) - 0.5*sparWidth, platformLeft(1))
		var sparBot = lerp(sparLowerBound, sparUpperBound, sparBotParam)
		var sparBotLeft = Vec2.stackAlloc(sparBot(0)-0.5*sparWidth, sparBot(1))
		var sparBotRight = Vec2.stackAlloc(sparBot(0)+0.5*sparWidth, sparBot(1))
		var sparAngle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var sparTop = sparBot + polar2rect(sparLength, sparAngle)
		var spar = BeamT.heapAlloc(sparBot, sparTop, sparWidth)
		scene.objects:push(spar)

		-- Support beams are stacked on the ground
		connections:push(Connections.Stacking.heapAlloc(ground, beam1))
		connections:push(Connections.Stacking.heapAlloc(ground, beam2))

		-- Support beams are welded to platform
		connections:push(Connections.Weld.heapAlloc(platform, beam1, 2, 3))
		connections:push(Connections.Weld.heapAlloc(platform, beam2, 2, 3))

		-- Spar is connected to platform by a hinge
		var hinge = Connections.Hinge.heapAlloc(sparBot)
		hinge:addBeam(spar)
		connections:push(hinge)

		-- Spar is connected to platform by cables
		var cableWidth = 0.4
		var sparAxis = sparTop - sparBot; sparAxis:normalize()
		var sparPerp = staticsUtils.perp(sparAxis)
		var platformLeftVec = platformLeft - platformCenter; platformLeftVec:normalize()
		var platformRightVec = platformRight - platformCenter; platformRightVec:normalize()
		if sparPerp:dot(platformLeftVec) > sparPerp:dot(platformRightVec) then
			sparPerp = -sparPerp    -- Guarantee perp points toward right edge of scene
		end
		for i=0,numCablePairs do
			var sparLengthParam = uniform(0.1, 1.0, {structural=false, lowerBound=0.1, upperBound=1.0})
			var sparPinCenter = lerp(sparBot, sparTop, sparLengthParam)
			var sparPinLeft = sparPinCenter - 0.5*sparWidth*sparPerp
			var sparPinRight = sparPinCenter + 0.5*sparWidth*sparPerp
			var platPinLeftParam = uniform(0.05, 0.95, {structural=false, lowerBound=0.05, upperBound=0.95})
			var platPinRightParam = uniform(0.05, 0.95, {structural=false, lowerBound=0.05, upperBound=0.95})
			var platPinLeft = lerp(platformLeft, sparBotLeft, platPinLeftParam)
			platPinLeft(1) = platPinLeft(1) + 0.5*platformThickness
			var platPinRight = lerp(sparBot, platformRight, platPinRightParam)
			platPinRight(1) = platPinRight(1) + 0.5*platformThickness
			var cableLeft = Connections.Cable.heapAlloc(platPinLeft, sparPinLeft, platform, spar, cableWidth)
			var cableRight = Connections.Cable.heapAlloc(platPinRight, sparPinRight, platform, spar, cableWidth)
			scene.objects:push(cableLeft:createProxy())
			scene.objects:push(cableRight:createProxy())
			connections:push(cableLeft)
			connections:push(cableRight)
		end

		enforceStability(&scene, &connections)
		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	----------------------------------

	return terra()
		-- return simpleBeamHingeCableScene()
		-- return twoBeamsConnectedByCableScene()
		-- return singleLinkWackyBridge()
		-- return multiLinkWackyBridge(5)
		-- return multiLinkSlidingBridge(5)
		return cableStayedBridge(3)
	end
end)

----------------------------------

local rendering = terralib.require("rendering")

local imageWidth = 500
local function renderInitFn(samples, im)
	return quote
		var argc = 0
		gl.glutInit(&argc, nil)
		var scene0 = &samples(0).value
		var aspectRatio = scene0.height / scene0.width
		var imageHeight = int(aspectRatio*imageWidth)
		gl.glutInitWindowSize(imageWidth, imageWidth)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		im:resize(imageWidth, imageHeight)
	end
end

local forceScale = 0.1
-- local forceScale = 1.0
local function renderDrawFn(sample, im)
	return quote
		var scene = &sample.value
		gl.glClearColor(1.0, 1.0, 1.0, 1.0)
		gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
		scene:draw(forceScale) 
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

----------------------------------

local numsamps = 1000
local verbose = true
local temp = 1.0
local kernel = HMC({numSteps=1000, verbosity=0,
					relaxManifolds=true,
					temperAcceptHamiltonian=false,
					temperGuideHamiltonian=false,
					temperInitialMomentum=false})
-- local kernel = GradientAscent({stepSize=0.0001})
-- local kernel = GaussianDrift({bandwidth = function(temp) return `lerp(0.5, 3.0, temp/100.0) end})
-- local kernel = GaussianDrift({bandwidth = 1.5})
-- local kernel = RandomWalk()
local scheduleFn = macro(function(iter, currTrace)
	return quote
			-- currTrace.temperature = lerp(100.0, 1.0, iter/double(numsamps))
			-- currTrace.temperature = expinterp(100.0, 1.0, iter/double(numsamps))
			-- currTrace.temperature = 100.0
	end
end)
kernel = Schedule(kernel, scheduleFn)
local newton = terralib.require("prob.newtonProj")
local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")
local model = staticsModel
local terra doInference()
	return [mcmc(model, kernel, {numsamps=numsamps, verbose=verbose})]
	-- return [forwardSample(model, numsamps)]

	-- -- Initialization
	-- var samples = [inf.SampleVectorType(model)].stackAlloc()
	-- var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]

	-- -- Burn in (find somewhere on the manifold)
	-- samples:push([inf.SampleType(model)].stackAlloc([inf.extractReturnValue(model)](currTrace), 0.0))
	-- currTrace = [newton.newtonPlusHMCManifoldProjection(staticsModel, {numSteps=1000}, {numsamps=50, verbose=true}, 2500)](currTrace, &samples)
	-- samples:push([inf.SampleType(model)].stackAlloc([inf.extractReturnValue(model)](currTrace), 0.0))
	
	-- -- CHMC
	-- var kernel = [HMC({numSteps=10, constrainToManifold=true, verbosity=0})()]
	-- currTrace = [inf.mcmcSample(model, {numsamps=numsamps, verbose=verbose})](currTrace, kernel, &samples)
	-- m.delete(kernel)

	-- m.delete(currTrace)
	-- return samples
end
local samples = m.gc(doInference())
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, imageWidth)




