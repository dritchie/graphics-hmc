terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand =terralib.require("prob.random")
local gl = terralib.require("gl")
local image = terralib.require("image")

local GradientAscent = terralib.require("gradientAscent")

local staticsUtils = terralib.require("staticsUtils")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)


----------------------------------


local function staticsModel()
	local gravityConstant = `9.8

	local Vec2 = Vec(real, 2)
	local ForceT = staticsUtils.Force(real)
	local RigidObjectT = staticsUtils.RigidObject(real)
	local BeamT = staticsUtils.Beam(real)
	local CableT = staticsUtils.Cable(real)
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
	local enforceStability = pfn(terra(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnectionT))
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
				-- var fres, tres = scene.objects(i):calculateResiduals()
				scene.objects(i):calculateResiduals(&fres, &tres)
				factor(softEq(fres(0), 0.0, 1.0))
				factor(softEq(fres(1), 0.0, 1.0))
				for j=0,tres.size do
					factor(softEq(tres(j), 0.0, 1.0))
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
		var hinge = Connections.HingeT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.5, hingeY))
		
		var beamBot = Vec2.stackAlloc(hinge.location(0), hingeY)
		var beamLen = 20.0
		-- var beamAngle = [math.pi/4]
		var beamAngle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beamTop = beamBot + polar2rect(beamLen, beamAngle)
		var beamWidth = 3.0
		var beam = BeamT.heapAlloc(beamBot, beamTop, beamWidth)
		hinge:addBeam(beam)

		var groundPin = Connections.CablePinT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.2, groundHeight), ground)
		var beamPin = Connections.CablePinT.heapAlloc(beamTop, beam)
		
		var cableWidth = 0.5
		var cable = CableT.heapAlloc(groundPin.location, beamPin.location, cableWidth)
		groundPin:addCable(cable, 0)
		beamPin:addCable(cable, 1)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam)
		scene.objects:push(cable)
		var connections = [Vector(&Connections.RigidConnectionT)].stackAlloc()
		connections:push(hinge)
		connections:push(groundPin)
		connections:push(beamPin)

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
		var hinge1 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
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
		var cable = CableT.heapAlloc(beam1.endpoints[1], beam2.endpoints[1], 0.5)
		var beam1Pin = Connections.CablePinT.heapAlloc(beam1.endpoints[1], beam1)
		var beam2Pin = Connections.CablePinT.heapAlloc(beam2.endpoints[1], beam2)
		beam1Pin:addCable(cable, 0)
		beam2Pin:addCable(cable, 1)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(cable)
		var connections = [Vector(&Connections.RigidConnectionT)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(beam1Pin)
		connections:push(beam2Pin)

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
		var hinge1 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
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
		var cable1 = CableT.heapAlloc(beam1.endpoints[1], platform.endpoints[0], cableWidth)
		var cable2 = CableT.heapAlloc(beam2.endpoints[1], platform.endpoints[1], cableWidth)
		var beam1Pin = Connections.CablePinT.heapAlloc(beam1.endpoints[1], beam1)
		var beam2Pin = Connections.CablePinT.heapAlloc(beam2.endpoints[1], beam2)
		var platformPin1 = Connections.CablePinT.heapAlloc(platform.endpoints[0], platform)
		var platformPin2 = Connections.CablePinT.heapAlloc(platform.endpoints[1], platform)
		beam1Pin:addCable(cable1, 0)
		platformPin1:addCable(cable1, 1)
		beam2Pin:addCable(cable2, 0)
		platformPin2:addCable(cable2, 1)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(platform)
		scene.objects:push(cable1)
		scene.objects:push(cable2)
		var connections = [Vector(&Connections.RigidConnectionT)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(beam1Pin)
		connections:push(beam2Pin)
		connections:push(platformPin1)
		connections:push(platformPin2)

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
		var connections = [Vector(&Connections.RigidConnectionT)].stackAlloc()

		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Set up end support beams
		var hinge1 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(0.2*sceneWidth, groundHeight))
		var hinge2 = Connections.HingeT.heapAlloc(Vec2.stackAlloc(0.8*sceneWidth, groundHeight))
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
			-- var rot = gaussian(0.0, [math.pi/20], {structural=false})
			var rot = uniform([-math.pi/6], [math.pi/6], {structural=false, lowerBound=[-math.pi/6], upperBound=[math.pi/6]})
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
			var cable = CableT.heapAlloc(startBeams(i).endpoints[startEndpoints(i)],
										 endBeams(i).endpoints[endEndpoints(i)],
										 width)
			scene.objects:push(cable)
			var startPin = Connections.CablePinT.heapAlloc(cable.endpoints[0], startBeams(i))
			var endPin = Connections.CablePinT.heapAlloc(cable.endpoints[1], endBeams(i))
			startPin:addCable(cable, 0)
			endPin:addCable(cable, 1)
			connections:push(startPin)
			connections:push(endPin)
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

	----------------------------------

	return terra()
		-- return simpleBeamHingeCableScene()
		-- return twoBeamsConnectedByCableScene()
		-- return singleLinkWackyBridge()
		return multiLinkWackyBridge(5)
	end
end


----------------------------------

-- local forceScale = 0.1
local forceScale = 1.0
local function renderSamples(samples, moviename, imageWidth)
	local moviefilename = string.format("renders/%s.mp4", moviename)
	local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
	local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
	io.write("Rendering video...")
	io.flush()
	local numsamps = samples.size
	local frameSkip = math.ceil(numsamps / 1000.0)
	local terra renderFrames()
		var argc = 0
		gl.glutInit(&argc, nil)
		var scene0 = &samples(0).value
		var aspectRatio = scene0.height / scene0.width
		var imageHeight = int(aspectRatio*imageWidth)
		gl.glutInitWindowSize(imageWidth, imageWidth)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		var im = RGBImage.stackAlloc(imageWidth, imageHeight)
		var framename: int8[1024]
		var framenumber = 0
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			var scene = &samples(i).value
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			scene:draw(forceScale)
			gl.glFlush()
			gl.glReadPixels(0, 0, imageWidth, imageHeight,
				gl.mGL_BGR(), gl.mGL_UNSIGNED_BYTE(), im.data)
			[RGBImage.save()](&im, image.Format.PNG, framename)
		end
	end
	renderFrames()
	util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
	util.wait(string.format("rm -f %s", movieframewildcard))
	print("done.")
end

----------------------------------
local numsamps = 1000
-- local numsamps = 3000
-- local numsamps = 2000000
-- local numsamps = 200000
local verbose = true
local temp = 1.0
local imageWidth = 500
local kernel = HMC({numSteps=1000, verbosity=0})
-- local kernel = GradientAscent({stepSize=0.0001})
-- local kernel = GaussianDrift({bandwidth=0.1})
-- local kernel = RandomWalk()
local scheduleFn = macro(function(iter, currTrace)
	return quote
			-- currTrace.temperature = numsamps/(iter+1)
			currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
	return [mcmc(staticsModel, kernel, {numsamps=numsamps, verbose=verbose})]
	-- return [forwardSample(staticsModel, numsamps)]
end
local samples = m.gc(doInference())
moviename = arg[1] or "movie"
renderSamples(samples, moviename, imageWidth)




