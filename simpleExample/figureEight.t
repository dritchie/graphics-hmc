terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand = terralib.require("prob.random")
local gl = terralib.require("gl")
local image = terralib.require("image")

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
-- local softness = 0.075
local softness = 0.005
local function figureEightModel()
	
	local Vec2 = Vec(real, 2)

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)

	return terra()
		-- Sample random dot uniformly
		var x = uniform(xmin, xmax, {structural=false, lowerBound=xmin, upperBound=xmax})
		var y = uniform(ymin, ymax, {structural=false, lowerBound=ymin, upperBound=ymax})

		-- Constrain to be on a figure eight
		var x2 = x * x
		var y2 = y * y
		var y4 = y2 * y2
		var isoval = y4 - y2 + x2
		factor(softEq(isoval, targetIsoval, softness))

		return Vec2.stackAlloc(x, y)
	end
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
				gl.mGL_BGR(), gl.mGL_UNSIGNED_BYTE(), im.data)
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
-- local numsamps = 2000
local numsamps = 200000
local verbose = true
local temp = 1.0
local imageWidth = 500
local imageHeight = 500
-- local kernel = HMC({numSteps=100})
local kernel = GaussianDrift({bandwidth=0.075})
local scheduleFn = macro(function(iter, currTrace)
	return quote
		currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
	return [mcmc(figureEightModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())
local moviename = arg[1] or "movie"
renderSamples(samples, moviename, imageWidth, imageHeight)
local filename = arg[1] or "samples"
writeSamplesToCSV(samples, filename)



