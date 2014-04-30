terralib.require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = terralib.require("s3dLib")
local m = terralib.require("mem")
local util = terralib.require("util")
local gl = terralib.require("gl")
local rendering = terralib.require("rendering")
local Vec = terralib.require("linalg").Vec
local colors = terralib.require("colors")

local C = terralib.includecstring [[
#include "stdio.h"
]]

-- local testcomp = terralib.require("examples.blockStack")
-- local testcomp = terralib.require("examples.arch")
-- local testcomp = terralib.require("examples.archTower")
-- local testcomp = terralib.require("examples.multiStack")
local testcomp = terralib.require("examples.curveNetwork")


-------------------------------------------------------

local lerp = macro(function(a, b, t)
	return `(1.0-t)*a + t*b
end)

local ensureEven = macro(function(x)
	return quote
		var y = x
		if y % 2 ~= 0 then y = y + 1 end
	in
		y
	end
end)
local imageWidth = 500
local function renderInitFn(samples, im)
	return quote
		var argc = 0
		gl.glutInit(&argc, nil)
		var scene0 = &samples(0).value
		var aspectRatio = scene0.camera.aspect
		var imageHeight = int(aspectRatio*imageWidth)
		imageHeight = ensureEven(imageHeight)
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE() or gl.mGLUT_DEPTH())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		im:resize(imageWidth, imageHeight)
	end
end

local renderForces = false
local RenderSettings = s3dLib(testcomp).RenderSettings
local unstableColor = colors.Tableau10.Red
local function renderDrawFn(sample, im)
	return quote
		var renderSettings = RenderSettings.stackAlloc()
		renderSettings.renderForces = renderForces
		var renderScene = &sample.value
		if not renderScene.scene:isStable() then
			renderSettings.activeBodyColor = [Vec(double, 4)].stackAlloc([unstableColor], 1.0)
		end
		renderScene:render(&renderSettings)
		[rendering.displayLogprob("BottomLeft")](sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------

local numsamps = 200
local numBurnInSamps = 200
local doHMC = true
local numHMCSteps = 1000
local gaussianBandwidth = 0.005

local ssmhKernel = GaussianDrift({bandwidth=gaussianBandwidth})
local hmcKernel = HMC({numSteps=numHMCSteps})
local kernel = nil
if doHMC then
	kernel = hmcKernel
else
	kernel = ssmhKernel
end
local lag = 1
if not doHMC then lag = 2*numHMCSteps end


-- local go = forwardSample(testcomp, 10000000)

local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")
local percentBurnIn = 0.1
local go = terra()
	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(testcomp)]
	var samps = [SampleVectorType(testcomp)].stackAlloc()
	-- Burn in using SSMH
	var burnInKern = [ssmhKernel()]
	currTrace = [inf.mcmcSample(testcomp, {numsamps=numBurnInSamps, verbose=true, lag=2*numHMCSteps})](currTrace, burnInKern, &samps)
	m.delete(burnInKern)
	-- Continue mixing using whatever kernel we asked for
	var mixKern = [kernel()]
	currTrace = [inf.mcmcSample(testcomp, {numsamps=numsamps, verbose=true, lag=lag})](currTrace, mixKern, &samps)
	m.delete(mixKern)
	m.delete(currTrace)
	return samps
end

-- local s3d = s3dLib(testcomp)
-- local AutoPtr = terralib.require("autopointer")
-- local Vector = terralib.require("vector")
-- local struct Foo { x: double }
-- terra Foo:__construct() self.x = 0.0 end
-- terra Foo:__construct(x: double) self.x = x end
-- m.addConstructors(Foo)
-- go = terra()
-- 	var samps = [Vector(AutoPtr(s3d.Scene))].stackAlloc()
-- 	-- var samps = [Vector(AutoPtr(s3d.Body))].stackAlloc()
-- 	-- var samps = [Vector(AutoPtr(Foo))].stackAlloc()
-- 	-- var samps = [Vector(double)].stackAlloc()
-- 	-- var samps = [Vector(Foo)].stackAlloc()
-- 	for s=0,10000 do
-- 		var scene = AutoPtr.wrap([s3d.Scene].heapAlloc(9.8, [s3d.Vec3].stackAlloc(0.0, 0.0, 1.0)))
-- 		for i=0,1000 do
-- 			var shape = [s3d.Box].heapAlloc()
-- 			var body = [s3d.Body].oak(shape)
-- 			scene.bodies:push(body)
-- 			-- var ab = AutoPtr.wrap(body); samps:push(ab); m.destruct(ab)
-- 			-- var af = AutoPtr.wrap(Foo.heapAlloc(0.0)); samps:push(af); m.destruct(af)
-- 			-- samps:push(0.0)
-- 			-- samps:push(Foo.stackAlloc(0.0))
-- 		end
-- 		samps:push(scene)
-- 		m.destruct(scene)
-- 	end
-- 	return samps
-- end

-- local trace = terralib.require("prob.trace")
-- local Vector = terralib.require("vector")
-- local simplecomp = probcomp(function() return terra() return gaussian(0.0, 1.0, {structural=false}) end end)
-- go = terra()
-- 	-- var currTrace : &trace.BaseTrace(double) = [trace.newTrace(simplecomp)]
-- 	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(testcomp)]
-- 	for i=0,10000000 do
-- 		var nextTrace = currTrace:deepcopy()
-- 		m.delete(currTrace)
-- 		currTrace = nextTrace
-- 		[trace.traceUpdate({structureChange=false})](currTrace)
-- 		-- m.delete(currTrace)
-- 		-- currTrace = [trace.newTrace(simplecomp)]
-- 	end
-- 	return [Vector(double)].stackAlloc()
-- end

local samples = go()

-- C.printf("waiting...\n")
-- util.wait("sleep 4s")
-- C.printf("destructing...\n")
-- samples:__destruct()
-- C.printf("done destructing\n")
-- while true do
-- 	--
-- end

moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, "../renders")






