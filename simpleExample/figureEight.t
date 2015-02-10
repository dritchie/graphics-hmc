require("prob")

local util = require("util")
local m = require("mem")
local templatize = require("templatize")
local ad = require("ad")
local Vector = require("vector")
local Vec = require("linalg").Vec
local rand = require("prob.random")
local gl = require("gl")
local image = require("image")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)

----------------------------------

local xmin = -1.5
local xmax = 1.5
local ymin = -1.5
local ymax = 1.5
local targetIsoval = 0.25
local function genFigureEightModel(softness)
	return probcomp(function()
		
		local Vec2 = Vec(real, 2)

		return terra()
			-- Sample random dot uniformly
			var x = uniform(xmin, xmax, {structural=false, lowerBound=xmin, upperBound=xmax})
			var y = uniform(ymin, ymax, {structural=false, lowerBound=ymin, upperBound=ymax})

			-- Constrain to be on a figure eight
			var x2 = x * x
			var y2 = y * y
			var y4 = y2 * y2
			var isoval = y4 - y2 + x2
			factor(softeq(isoval, targetIsoval, softness))

			return Vec2.stackAlloc(x, y)
		end
	end)
end


----------------------------------

local terra drawCircle(pos: Vec2d, rad: double, color: Color3d, subdivs: int) : {}
	gl.glPushMatrix()
	gl.glTranslated(pos(0), pos(1), 0.0)
	gl.glColor3d(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_POLYGON())
	for i=0,subdivs do
		var ang = ([2*math.pi]*i)/subdivs
		gl.glVertex2d(rad*ad.math.cos(ang), rad*ad.math.sin(ang))
	end
	gl.glEnd()
	gl.glPopMatrix()
end
terra drawCircle(pos: Vec2d, rad: double, color: Color3d) : {}
	drawCircle(pos, rad, color, 32)
end

local function renderSamples(samples, moviename, imageWidth, imageHeight)
	local moviefilename = string.format("%s.mp4", moviename)
	local movieframebasename = string.format("%s", moviename) .. "_%06d.png"
	local movieframewildcard = string.format("%s", moviename) .. "_*.png"
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
		var dotColor = Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glMatrixMode(gl.mGL_PROJECTION())
			gl.glLoadIdentity()
			gl.gluOrtho2D(xmin, xmax, ymin, ymax)
			gl.glMatrixMode(gl.mGL_MODELVIEW())
			gl.glLoadIdentity()
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			var dotPos = samples(i).value
			drawCircle(dotPos, 0.1, dotColor)
			gl.glFlush()
			gl.glReadPixels(0, 0, imageWidth, imageHeight,
				gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
			[RGBImage.save()](&im, image.Format.PNG, framename)
		end
	end
	renderFrames()
	util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
	util.wait(string.format("rm -f %s", movieframewildcard))
	print("done.")
end

local function writeSamplesToCSV(samples, filename)
	local fname = string.format("%s.csv", filename)
	local numsamps = samples.size
	local terra doWrite()
		var f = C.fopen(fname, "w")
		for i=0,numsamps do
			var p = samples(i).value
			var t = double(i)/numsamps
			C.fprintf(f, "%g,%g,%g\n", p(0), p(1), t)
		end
		C.fclose(f)
	end
	doWrite()
end

----------------------------------

local easy_softness = 0.1
local difficult_softness = 0.005
local easy_bandwidth = 1.2
local difficult_bandwidth = 0.04

local hasTrueStats = global(bool, false)
local trueMean = global(Vec2d)
local trueVar = global(double)

local function nameOfRun(difficulty, doHMC)
	local base = doHMC and "hmc" or "random"
	return string.format("%s_%s", base, difficulty)
end

-- MUST RUN HMC VERSION FIRST to fill in the 'true' stats
local function doInference(difficulty, doHMC)
	local numsamps = 2000
	local numHMCsteps = 100
	local imageWidth = 500
	local imageHeight = 500
	local kernel = nil
	local lag = 1
	if doHMC then
		kernel = HMC({numSteps=numHMCsteps})
	else
		lag = 2*numHMCsteps
		kernel = GaussianDrift({bandwidth=(difficulty == "easy" and easy_bandwidth or difficult_bandwidth), bandwidthAdapt=false})
	end
	local model = genFigureEightModel(difficulty == "easy" and easy_softness or difficult_softness)
	local terra doInference()
		return [mcmc(model, kernel, {numsamps=numsamps, lag=lag, verbose=true})]()
	end
	local samples = m.gc(doInference())
	local basename = nameOfRun(difficulty, doHMC)
	writeSamplesToCSV(samples, basename)
	local autocorrname = string.format("%s_autocorr.csv", basename)
	local terra writeAutocorrToCSV()
		var vals = sampleValues(samples)
		if not hasTrueStats then
			if doHMC then
				trueMean = expectation(vals)
				trueVar = variance(vals, trueMean)
				hasTrueStats = true
			else
				util.fatalError("Attempted to compute autocorrrelation of a non-HMC run without first computing a 'ground truth' mean and variance\n")
			end
		end
		var autocorr = autocorrelation(vals, trueMean, trueVar)
		saveAutocorrelation(&autocorr, autocorrname)
		m.destruct(autocorr)
		m.destruct(vals)
	end
	writeAutocorrToCSV()
	-- renderSamples(samples, basename, imageWidth, imageHeight)
end

-- Actually do the runs
doInference("difficult", true)
doInference("difficult", false)
doInference("easy", false)




