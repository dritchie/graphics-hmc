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


local polar2rect = macro(function(polarVec)
	local VecT = polarVec:gettype()
	return quote
		var r = polarVec(0)
		var theta = polarVec(1)
	in
		VecT.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end
end)

local softEq = macro(function(x, target, softness)
	local xtype = x:gettype()
	return `[rand.gaussian_logprob(xtype)](x, target, softness)
end)

local ngaussian = macro(function(m, sd)
	return `gaussian(m, sd, {structural=false})
end)
local ngammaMS = macro(function(m, s)
	return `gammaMeanShape(m, s, {structural=false})
end)
local nuniformNoPrior = macro(function(lo, hi)
	return `uniform(lo, hi, {structural=false, hasPrior=false})
end)


local Plate = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct PlateT
	{
		polarPos: Vec2,
		pos: Vec2,
		size: real
	}
	terra PlateT:__construct()
		m.init(self.polarPos)
		m.init(self.pos)
		self.size = 0.0
	end
	terra PlateT:__construct(parentSize: real)
		self.size = ngammaMS(parentSize/10.0, 10.0)
		var maxRadius = parentSize - self.size
		self.polarPos = Vec2.stackAlloc(nuniformNoPrior(0.0, [2*math.pi]),
									    nuniformNoPrior(0.0, maxRadius))
		self.pos = polar2rect(self.polarPos)
		-- Factor: plate is on the table
		var rdiff = self.polarPos(1) - maxRadius
		if rdiff > 0.0 then
			factor(softEq(rdiff, 0.0, self.size/5.0))
		end
	end
	PlateT.methods.__construct = pfn(PlateT.methods.__construct, {ismethod=true})
	m.addConstructors(PlateT)
	return PlateT
end)

local Table = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local PlateT = Plate(real)
	local struct TableT
	{
		plates: Vector(PlateT),
		pos: Vec2,
		size: real
	}
	terra TableT:__construct() : {}
		m.init(self.plates)
		m.init(self.pos)
		self.size = 0.0
	end
	terra TableT:__construct(numPlates: int, pos: Vec2) : {}
		self.pos = pos
		self.size = ngammaMS(10.0, 10.0)
		m.init(self.plates)
		for i=0,numPlates do
			self.plates:push(PlateT.stackAlloc(self.size))
		end
		-- Factor: plates don't overlap
		-- Factor: plates fill up most of the table
	end
	TableT.methods.__construct = pfn(TableT.methods.__construct, {ismethod=true})
	m.addConstructors(TableT)
	return TableT
end)

local roomWidth = 100
local roomHeight = 100
local function layoutModel()
	local numPlates = 1
	return terra()
		var pos = Vec2d.stackAlloc(roomWidth/2.0, roomHeight/2.0)
		return [Table(real)].stackAlloc(numPlates, pos)
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
local terra drawCircle(pos: Vec2d, rad: double, color: Color3d) : {}
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
	local frameSkip = math.ceil(numsamps / 1000.0)
	local terra renderFrames()
		-- init opengl context (via glut window; sort of hacky)
		var argc = 0
		gl.glutInit(&argc, nil)
		gl.glutInitWindowSize(0, 0)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")

		-- Set up transforms
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		gl.gluOrtho2d(roomWidth, roomHeight)
		gl.glMatrixMode(gl.mGL_MODELVIEW())
		gl.glLoadIdentity()

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
			gl.glClear(mGL_COLOR_BUFFER_BIT())
			var table = &samples(i).value
			drawCircle(table.pos, table.size, tableColor)
			for i=0,table.plates.size do
				var plate = table.plates:getPointer(i)
				drawCircle(plate.pos, plate.size, plateColor)
			end
			gl.glFlush()
			gl.glReadPixels(0, 0, imageWidth, imageHeight,
				gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
			[RGBImage.save()](im, image.Format.PNG, framename)
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
local kernel = HMC({numSteps=20, targetAcceptRate=0.65})
local terra doInference()
	return [mcmc(layoutModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())



