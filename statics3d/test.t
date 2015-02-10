require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = require("s3dLib")
local m = require("mem")
local util = require("util")
local gl = require("gl")
local rendering = require("rendering")
local Vector = require("vector")
local Vec = require("linalg").Vec
local colors = require("colors")
local fab = require("fabrication")
local image = require("image")
local Params = require("testparams")

local C = terralib.includecstring [[
#include "stdio.h"
#include "string.h"
]]

local radians = macro(function(deg)
	return `deg*[math.pi]/180.0
end)

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
	local Scene = s3dLib(testcomp).Scene
	local Body = s3dLib(testcomp).Body
	local Connection = s3dLib(testcomp).Connection

	local terra normalStability(scene: &Scene)
		return scene:isStable()
	end

	local terra tiltStability(scene: &Scene, tiltAmt: double)
		var numSteps = 10
		var tiltIncr = 2.0*tiltAmt/numSteps
		scene:tiltX(-tiltAmt)
		var isstable = scene:isStable()
		for i=0,numSteps do
			scene:tiltX(tiltIncr)
			isstable = isstable and scene:isStable()
		end
		scene:tiltX(-tiltAmt)
		return isstable
	end

	local terra constructionStability(scene: &Scene)
		var bodies = [Vector(&Body)].stackAlloc()
		var connections = [Vector(&Connection)].stackAlloc()

		var isstable = true
		var ground = scene.bodies(0)
		for i=1,scene.bodies.size do  -- Skip ground at index 0
			scene.bodies(i):clearConnections()
			bodies:push(scene.bodies(i))
		end
		scene.bodies:clear()
		scene.bodies:push(ground)
		for i=0,scene.connections.size do
			connections:push(scene.connections(i))
		end
		scene.connections:clear()

		for i=0,bodies.size do
			scene.bodies:push(bodies(i))
			scene.connections:push(connections(i))
			connections(i):recalculate()
			-- There's some kind of numerical glitch that pops up occasionally with evaluating
			--    the stability of towers that have a multiple of three number of blocks. It's
			--    really not worth figuring this out right now, I don't think.
			if (i+1)%3 ~= 0 then
				isstable = isstable and scene:isStable()
			end
		end

		m.destruct(bodies)
		m.destruct(connections)
		return isstable
	end

	local terra sceneIsStable(scene: &Scene)
		return normalStability(scene)
		-- return tiltStability(scene, radians(10.0))
		-- return constructionStability(scene)
	end

	return function(sample, im, sampleindex)
		return quote
			var renderSettings = RenderSettings.stackAlloc()
			renderSettings.renderForces = renderForces
			var renderScene = &sample.value
			if not sceneIsStable(&renderScene.scene) then
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

	local testcomp = require(params.exampleToRun)

	local ssmhKernel = GaussianDrift({bandwidth=params.gaussianBandwidth, bandwidthAdapt=true})
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
		local inf = require("prob.inference")
		local trace = require("prob.trace")
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

	-- Save scene descriptions
	if params.saveSceneDescriptions then
		local terra saveScenes()
			var buf : int8[1024]
			for i=0,samples.size do
				C.sprintf(buf, "%s/%s_%06d.txt", params.outputdir, params.name, i)
				samples(i).value.scene:saveToFile(buf)
			end
		end
		print("Saving scene descriptions...")
		saveScenes()
		print("done.")
	end

	-- Render sample trace as movie
	if params.renderMovie then
		rendering.renderSamples(samples, makeRenderInitFn(params.imgRes), makeRenderDrawFn(testcomp), params.name, params.outputdir, params.deleteImages)
	end

	-- Generate average image
	if params.genAverageImg then
		local RGBImage = image.Image(uint8, 3)
		local RGBImageD = image.Image(double, 3)
		local Color3d = Vec(double, 3)
		local numimgs = params.numSamps
		if params.saveBurnIn then numimgs = numimgs + params.numBurnInSamps end
		local terra makeAverageImage()
			var avgImg = RGBImageD.stackAlloc()
			var buf : int8[1024]
			for i=0,numimgs do
				C.sprintf(buf, "%s/%s_%06d.png", params.outputdir, params.name, i)
				-- Individual images are loaded as 8-bit RGB
				var img = RGBImage.load(image.Format.PNG, buf)
				-- Size the output image, if we haven't yet
				if i==0 then avgImg:resize(img.width, img.height) end
				-- Iterate over pixels, convert to double, add in
				for y=0,img.height do
					for x=0,img.width do
						var bytePixel = img(x,y)
						var floatPixel = Color3d.stackAlloc(bytePixel(0)/255.0, bytePixel(1)/255.0, bytePixel(2)/255.0)
						avgImg(x,y) = avgImg(x,y) + floatPixel
					end
				end
				m.destruct(img)
			end
			-- Normalize
			for y=0,avgImg.height do
				for x=0,avgImg.width do
					avgImg(x,y) = avgImg(x,y) / double(numimgs)
				end
			end
			-- Save (quantize in the process)
			C.sprintf(buf, "%s/%s_average.png", params.outputdir, params.name)
			[RGBImageD.save(uint8)](&avgImg, image.Format.PNG, buf)
			m.destruct(avgImg)
		end
		print("Making average image...")
		makeAverageImage()
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

-- Save blueprints for fabrication!
if params.saveBlueprints ~= -1 then
	local testcomp = require(params.exampleToRun)
	local ppi = 300
	local terra saveBlueprints()
		util.systemf("mkdir %s/%s_blueprints", params.outputdir, params.name)
		var buf : int8[1024]
		C.sprintf(buf, "%s/%s_%06d.txt", params.outputdir, params.name, params.saveBlueprints)
		var scene = [s3dLib(testcomp).Scene].loadFromFile(buf)

		C.sprintf(buf, "%s/%s_blueprints/%u", params.outputdir, params.name, params.saveBlueprints)
		[fab.saveBlueprints(testcomp)](&scene, buf, ppi)
		m.destruct(scene)
	end
	print("Saving fab schematics...")
	saveBlueprints()
	print("done.")

-- Otherwise, do some sampling
elseif params.doComparison then
	doHMCvsSVMHcomparison(params)
else
	doRun(params)
end






