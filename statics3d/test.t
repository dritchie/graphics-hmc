terralib.require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = terralib.require("s3dLib")
local m = terralib.require("mem")
local util = terralib.require("util")
local gl = terralib.require("gl")
local rendering = terralib.require("rendering")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local colors = terralib.require("colors")
local fab = terralib.require("fabrication")
local image = terralib.require("image")
local Params = terralib.require("testparams")

local C = terralib.includecstring [[
#include "stdio.h"
#include "string.h"
]]



local lerp = macro(function(a, b, t)
	return `(1.0-t)*a + t*b
end)

-------------------------------------------------------

-- Initialize glut
local terra initGlut()
	var argc = 0
	gl.glutInit(&argc, nil)
end
initGlut()

local ensureEven = macro(function(x)
	return quote
		var y = x
		if y % 2 ~= 0 then y = y + 1 end
	in
		y
	end
end)

local function makeRenderInitFn(imageWidth)
	return function(samples, im)
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
end

local renderForces = false
local unstableColor = colors.Tableau10.Red
local function makeRenderDrawFn(testcomp)
	local RenderSettings = s3dLib(testcomp).RenderSettings
	return function(sample, im, sampleindex)
		return quote
			var renderSettings = RenderSettings.stackAlloc()
			renderSettings.renderForces = renderForces
			var renderScene = &sample.value
			if not renderScene.scene:isStable() then
				renderSettings.activeBodyColor = [Vec(double, 4)].stackAlloc([unstableColor], 1.0)
			end
			renderScene:render(&renderSettings)
			-- [rendering.displaySampleInfo("BottomLeft")](sampleindex, sample.logprob)
			gl.glFlush()
			gl.glReadPixels(0, 0, im.width, im.height,
				gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
		end
	end
end


-------------------------------------------------------


local hasTrueStats = global(bool, false)
local trueMean = global(Vector(double))
local trueVar = global(double)

local function doRun(params)

	local testcomp = terralib.require(params.exampleToRun)

	local ssmhKernel = GaussianDrift({bandwidth=params.gaussianBandwidth})
	local hmcKernel = HMC({numSteps=params.numHMCSteps})
	local kernel = nil
	if params.doHMC then
		kernel = hmcKernel
	else
		kernel = ssmhKernel
	end
	local lag = 1
	if not params.doHMC then lag = 2*params.numHMCSteps end

	local go = nil

	if params.forwardSample then
		go = forwardSample(testcomp, params.numSamps)
	else
		local inf = terralib.require("prob.inference")
		local trace = terralib.require("prob.trace")
		go = terra()
			var currTrace : &trace.BaseTrace(double) = [trace.newTrace(testcomp)]
			var samps = [SampleVectorType(testcomp)].stackAlloc()
			-- Burn in using SSMH
			var burnInKern = [ssmhKernel()]
			var burnInSamps : &SampleVectorType(testcomp) = [params.saveBurnIn and (`&samps) or (`nil)]
			currTrace = [inf.mcmcSample(testcomp, {numsamps=params.numBurnInSamps, verbose=true, lag=2*params.numHMCSteps})](currTrace, burnInKern, burnInSamps)
			m.delete(burnInKern)
			-- Continue mixing using whatever kernel we asked for
			var mixKern = [kernel()]
			currTrace = [inf.mcmcSample(testcomp, {numsamps=params.numSamps, verbose=true, lag=lag})](currTrace, mixKern, &samps)
			m.delete(mixKern)
			m.delete(currTrace)
			return samps
		end
	end


	-- Collect samples
	local samples = go()


	-- Ensure output dir exists
	util.wait(string.format("mkdir %s", params.outputdir))


	-- Spit out autocorrelation
	if params.computeAutocorr then
		local terra autocorr()
			var vals = [Vector(Vector(double))].stackAlloc(samples.size)
			for i=0,samples.size do
				m.destruct(vals(i))
				vals(i) = samples(i).value.scene:toVector()
			end
			if not hasTrueStats then
				if params.doHMC then
					trueMean = expectation(vals)
					trueVar = variance(vals, trueMean)
					hasTrueStats = true
				else
					util.fatalError("non-HMC sample run couldn't find 'ground truth' mean & variance for autocorrelation.\n")
				end
			end
			var ac = autocorrelation(vals, trueMean, trueVar)
			var buf : int8[1024]
			C.sprintf(buf, "%s/%s_autocorr.csv", params.outputdir, params.name)
			saveAutocorrelation(&ac, buf)
			m.destruct(vals)
			m.destruct(ac)
		end
		print("Computing autocorrelation...")
		autocorr()
		print("done.")
	end

	-- Render sample trace as movie
	if params.renderMovie then
		rendering.renderSamples(samples, makeRenderInitFn(params.imgRes), makeRenderDrawFn(testcomp), params.name, params.outputdir, params.deleteImages)
	end

	-- Generate average image
	if params.genAverageImg then

		local terra makeAverageImage()
			--
		end
		print("Making average image...")
		makeAverageImage()
		print("done.")
	end

	-- Save blueprints for fabrication!
	if params.saveBlueprints then
		local ppi = 300
		local terra saveBlueprints()
			util.systemf("mkdir %s/%s_blueprints", params.outputdir, params.name)
			var buf : int8[1024]
			for i=0,samples.size do
				C.sprintf(buf, "%s/%s_blueprints/%u", params.outputdir, params.name, i)
				[fab.saveBlueprints(testcomp)](&samples(i).value.scene, buf, ppi)
			end
		end
		print("Saving fab schematics...")
		saveBlueprints()
		print("done.")
	end
end

local function doHMCvsSVMHcomparison(params)
	local origname = params.name
	params.doHMC = true
	params.name = "hmc"
	doRun(params)
	params.doHMC = false
	params.name = "svmh"
	doRun(params)
end


-------------------------------------------------------


local params = Params.new():loadFile(arg[1] or "config.txt")
params:print()
doHMCvsSVMHcomparison(params)






