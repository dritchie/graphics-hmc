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
#include "string.h"
]]

-- local testcomp = terralib.require("examples.blockStack")
-- local testcomp = terralib.require("examples.arch")
-- local testcomp = terralib.require("examples.archTower")
-- local testcomp = terralib.require("examples.multiStack")
-- local testcomp = terralib.require("examples.curveNetwork")
-- local testcomp = terralib.require("examples.multiStack_fab")
local testcomp = terralib.require("examples.blockStack_fab")



local lerp = macro(function(a, b, t)
	return `(1.0-t)*a + t*b
end)

-------------------------------------------------------

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
local function renderDrawFn(sample, im, sampleindex)
	return quote
		var renderSettings = RenderSettings.stackAlloc()
		renderSettings.renderForces = renderForces
		var renderScene = &sample.value
		if not renderScene.scene:isStable() then
			renderSettings.activeBodyColor = [Vec(double, 4)].stackAlloc([unstableColor], 1.0)
		end
		renderScene:render(&renderSettings)
		[rendering.displaySampleInfo("BottomLeft")](sampleindex, sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------

local numsamps = 200
local numBurnInSamps = 100
local saveBurnIn = false
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


-- local go = forwardSample(testcomp, 100)

local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")
local percentBurnIn = 0.1
local go = terra()
	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(testcomp)]
	var samps = [SampleVectorType(testcomp)].stackAlloc()
	-- Burn in using SSMH
	var burnInKern = [ssmhKernel()]
	var burnInSamps : &SampleVectorType(testcomp) = [saveBurnIn and (`&samps) or (`nil)]
	currTrace = [inf.mcmcSample(testcomp, {numsamps=numBurnInSamps, verbose=true, lag=2*numHMCSteps})](currTrace, burnInKern, burnInSamps)
	m.delete(burnInKern)
	-- Continue mixing using whatever kernel we asked for
	var mixKern = [kernel()]
	currTrace = [inf.mcmcSample(testcomp, {numsamps=numsamps, verbose=true, lag=lag})](currTrace, mixKern, &samps)
	m.delete(mixKern)
	m.delete(currTrace)
	return samps
end


-- Collect samples
local samples = go()


-- Initialize glut
local terra initGlut()
	var argc = 0
	gl.glutInit(&argc, nil)
end
initGlut()


-- Render sample trace as movie
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, "../renders")


-- Save blueprints for fabrication!
local fab = terralib.require("fabrication")
local ppi = 300
local terra saveBlueprints()
	var buf : int8[1024]
	for i=0,samples.size do
		C.sprintf(buf, "../renders/blueprints/%u", i)
		[fab.saveBlueprints(testcomp)](&samples(i).value.scene, buf, ppi)
	end
end
print("Saving fab schematics...")
saveBlueprints()
print("done")







