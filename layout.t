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

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)


local Plate = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct PlateT { pos: Vec2, size: real }
	return PlateT
end)

local Table = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct TableT { pos: Vec2, size: real, plates: Vector(Plate(real)) }
	terra TableT:__construct() : {}
		m.init(self.pos)
		self.size = 0.0
		m.init(self.plates)
	end
	terra TableT:__construct(pos: Vec2, size: real) : {}
		self.pos = pos
		self.size = size
		m.init(self.plates)
	end
	terra TableT:__destruct()
		m.destruct(self.plates)
	end
	terra TableT:__copy(other: &TableT)
		self.pos = other.pos
		self.size = other.size
		self.plates = m.copy(other.plates)
	end
	m.addConstructors(TableT)
	return TableT
end)


local roomWidth = 100
local roomHeight = 100
local function layoutModel()
	local numPlates = 1
	local Vec2 = Vec(real, 2)
	local PlateT = Plate(real)
	local TableT = Table(real)

	--------------------------------------------

	local polar2rect = macro(function(polarVec)
		return quote
			var r = polarVec(0)
			var theta = polarVec(1)
		in
			Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
		end
	end)

	--------------------------------------------

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)
	local upperBound = macro(function(val, bound, softness)
		return quote
			if val > bound then
				factor(softEq(val-bound, 0.0, softness))
			end 
		end
	end)
	local lowerBound = macro(function(val, bound, softness)
		return quote
			if val < bound then
				factor(softEq(bound-val, 0.0, softness))
			end 
		end
	end)
	local bound = macro(function(val, lo, hi, softness)
		return quote
			lowerBound(val, lo, softness)
			upperBound(val, hi, softness)
		end
	end)

	--------------------------------------------

	local ngaussian = macro(function(m, sd)
		return `gaussian(m, sd, {structural=false})
	end)
	local ngammaMS = macro(function(m, s)
		return `gammaMeanShape(m, s, {structural=false})
	end)
	local nuniformNoPrior = macro(function(lo, hi)
		return `uniform(lo, hi, {structural=false, hasPrior=false})
	end)

	--------------------------------------------

	local makePlate = pfn(terra(parent: &TableT)
		var size = ngammaMS(parent.size/5.0, 100.0)
		var maxRadius = parent.size - size
		var polarPos = Vec2.stackAlloc(nuniformNoPrior(0.0, maxRadius),
									   nuniformNoPrior(0.0, [2*math.pi]))
		-- Soft bound: stay on the table!
		bound(polarPos(0), 0.0, maxRadius, 0.1)
		var pos = parent.pos + polar2rect(polarPos)
		return PlateT { pos, size }
	end)

	local makeTable = pfn(terra(numPlates: int, pos: Vec2)
		var size = ngammaMS(10.0, 100.0)
		var t = TableT.stackAlloc(pos, size)
		for i=0,numPlates do
			t.plates:push(makePlate(&t))
		end
		-- Factor: plates don't overlap
		-- Factor: plates fill up most of the table
		return t
	end)

	--------------------------------------------

	return terra()
		var pos = Vec2.stackAlloc(roomWidth/2.0, roomHeight/2.0)
		return makeTable(numPlates, pos)
	end
end

----------------------------------

local terra drawCircle(pos: Vec2d, rad: double, color: Color3d, subdivs: int) : {}
	gl.glPushMatrix()
	gl.glTranslated(pos(0), pos(1), 0.0)
	gl.glColor3f(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_POLYGON())
	for i=0,subdivs do
		var ang = ([2*math.pi]*i)/subdivs
		gl.glVertex2d(rad*ad.math.cos(ang), rad*ad.math.sin(ang))
	end
	gl.glEnd()
	gl.glPopMatrix()
end
terra drawCircle(pos: Vec2d, rad: double, color: Color3d) : {}
	drawCircle(pos, rad, color, 16)
end

local imageWidth = 500
local imageHeight = 500
local function renderSamples(samples, moviename)
	local moviefilename = string.format("renders/%s.mp4", moviename)
	local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
	local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
	io.write("Rendering video...")
	io.flush()
	local numsamps = samples.size
	local frameSkip = math.ceil(numsamps / 1000.0)
	local terra renderFrames()
		-- init opengl context (via glut window; sort of hacky)
		var argc = 0
		gl.glutInit(&argc, nil)
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)

		-- Set up transforms
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		gl.gluOrtho2D(0, roomWidth, 0, roomHeight)
		gl.glMatrixMode(gl.mGL_MODELVIEW())

		-- Render all frames, save to image, write to disk
		var im = RGBImage.stackAlloc(imageWidth, imageHeight)
		var framename: int8[1024]
		var framenumber = 0
		var tableColor = Color3d.stackAlloc(1.0, 0.25, 0.25)
		var plateColor = Color3d.stackAlloc(0.25, 0.25, 1.0)
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			gl.glLoadIdentity()
			var table = &samples(i).value
			drawCircle(table.pos, table.size, tableColor)
			for i=0,table.plates.size do
				var plate = table.plates:getPointer(i)
				drawCircle(plate.pos, plate.size, plateColor)
			end
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

----------------------------------

local numsamps = 1000
local verbose = true
-- local kernel = HMC({numSteps=1})
local kernel = HMC({numSteps=20})
local terra doInference()
	return [mcmc(layoutModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

local moviename = arg[1] or "movie"
renderSamples(samples, moviename)


