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
	local sceneWidth = `50.0
	local sceneHeight = `50.0
	local groundHeight = `2.0

	local Vec2 = Vec(real, 2)
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

	-- Define a simple scene with a beam rotating on a hinge
		--    attached to the ground. There's also a cable connecting
		--    the beam to the ground. The cable is inactive; it applies
		--    forces, but we don't solve for its equilibrium.
	local simpleScene1 = pfn(terra()
		var ground = Ground(groundHeight, 0.0, sceneWidth)

		var hingeBot = groundHeight + 2.0
		var hingeRad = 2.0
		var hinge = Connections.HingeT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.666, hingeBot+hingeRad))
		
		var beamBot = Vec2.stackAlloc(hinge.location(0), hingeBot)
		var beamLen = 20.0
		var beamAngle = uniform(0.0, [math.pi], {structural=false, lowerBound=0.0, upperBound=[math.pi]})
		var beamTop = beamBot + polar2rect(beamLen, beamAngle)
		var beamWidth = 5.0
		var beam = BeamT.heapAlloc(beamBot, beamTop, beamWidth)
		hinge:addBeam(beam)

		var groundPin = Connections.CablePinT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.333, groundHeight-1.0), beam)
		var beamPin = Connections.CablePinT.heapAlloc(beamTop + 0.05*(beamBot-beamTop), beam)
		
		var cableWidth = 1.0
		var cable = CableT.heapAlloc(groundPin.location, beamPin.location, cableWidth, false, true)
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

		-- TODO: Enforce equilibrium

		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end)

	----------------------------------

	return terra()
		return simpleScene1()
	end
end


----------------------------------

local function renderSamples(samples, moviename, imageWidth, imageHeight)
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
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		var im = RGBImage.stackAlloc(imageWidth, imageHeight)
		var framename: int8[1024]
		var framenumber = 0
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			var scene = &samples(i).value
			scene:draw()
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
local numsamps = 2000
-- local numsamps = 1000000
local verbose = true
local temp = 1.0
local imageWidth = 500
local imageHeight = 500
local kernel = HMC({numSteps=1000, verbosity=0})
-- local kernel = RandomWalk()
local scheduleFn = macro(function(iter, currTrace)
	return quote
		currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
	-- return [mcmc(staticsModel, kernel, {numsamps=numsamps, verbose=verbose})]
	return [forwardSample(staticsModel, numsamps)]
end
local samples = m.gc(doInference())
moviename = arg[1] or "movie"
renderSamples(samples, moviename, imageWidth, imageHeight)




