terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
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

local Circle = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct CircleT { pos: Vec2, size: real}
	terra CircleT:area() return [math.pi]*self.size*self.size end
	terra CircleT:intersectArea(other: &CircleT)
		var r = self.size
		var R = other.size
		var d = self.pos:dist(other.pos)
		if d > r+R then
			return real(0.0)
		end
		if R < r then
			r = other.size
			R = self.size
		end
		var d2 = d*d
		var r2 = r*r
		var R2 = R*R
		var x1 = r2*ad.math.acos((d2 + r2 - R2)/(2*d*r))
		var x2 = R2*ad.math.acos((d2 + R2 - r2)/(2*d*R))
		var x3 = 0.5*ad.math.sqrt((-d+r+R)*(d+r-R)*(d-r+R)*(d+r+R))
		return x1 + x2 - x3
	end
	return CircleT
end)

local Plate = templatize(function(real)
	local CircleT = Circle(real)
	local struct PlateT {}
	inheritance.staticExtend(CircleT, PlateT)
	return PlateT
end)

local Table = templatize(function(real)
	local CircleT = Circle(real)
	local PlateT = Plate(real)
	local Vec2 = Vec(real, 2)
	local struct TableT { plates: Vector(PlateT) }
	inheritance.staticExtend(CircleT, TableT)
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

local Pillar = templatize(function(real)
	local CircleT = Circle(real)
	local struct PillarT {}
	inheritance.staticExtend(CircleT, PillarT)
	return PillarT
end)

local Room = templatize(function(real)
	local TableT = Table(real)
	local PillarT = Pillar(real)
	local struct RoomT
	{
		width: real,
		height: real,
		tables: Vector(TableT),
		pillars: Vector(PillarT)
	}
	terra RoomT:__construct(w: real, h: real) : {}
		self.width = w
		self.height = h
		m.init(self.tables)
		m.init(self.pillars)
	end
	terra RoomT:__construct() : {}
		self:__construct(0.0, 0.0)
	end
	terra RoomT:__destruct()
		m.destruct(self.tables)
		m.destruct(self.pillars)
	end
	terra RoomT:__copy(other: &RoomT)
		self.width = other.width
		self.height = other.height
		self.tables = m.copy(other.tables)
		self.pillars = m.copy(other.pillars)
	end
	m.addConstructors(RoomT)
	return RoomT
end)


local function layoutModel()
	local roomWidth = `100.0
	local roomHeight = `100.0
	local numTables = 8
	local numPlates = 4
	local Vec2 = Vec(real, 2)
	local PlateT = Plate(real)
	local TableT = Table(real)
	local PillarT = Pillar(real)
	local RoomT = Room(real)

	--------------------------------------------

	local terra polar2rect(polarVec: Vec2)
		var r = polarVec(0)
		var theta = polarVec(1)
		return Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end

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
	local lowerClamp = macro(function(val, lo)
		return `ad.math.fmax(val, lo)
	end)
	local upperClamp = macro(function(val, hi)
		return `ad.math.fmin(val, hi)
	end)
	local clamp = macro(function(val, lo, hi)
		return `lowerClamp(upperClamp(val, hi), lo)
	end)

	--------------------------------------------

	local ngaussian = macro(function(m, sd)
		return `gaussian(m, sd, {structural=false})
	end)
	local nuniformNoPrior = macro(function(lo, hi)
		return `uniform(lo, hi, {structural=false, hasPrior=false})
	end)

	--------------------------------------------

	local makePlate = pfn(terra(parent: &TableT, sizePrior: real)
		-- Skew size toward the prior, but make sure it's positive
		-- Soft bound to guide gradients, plus hard clamp afterwards to
		--    make sure value is consitent with program semantics
		var size = ngaussian(sizePrior, 0.25)
		lowerBound(size, 1.0, 0.1)
		size = lowerClamp(size, 0.0)
		var maxRadius = parent.size - size
		var polarPos = Vec2.stackAlloc(nuniformNoPrior(0.0, maxRadius),
									   nuniformNoPrior(0.0, [2*math.pi]))
		-- Soft bound: stay on the table!
		bound(polarPos(0), 0.0, maxRadius, 0.2)
		var pos = parent.pos + polar2rect(polarPos)
		return PlateT { pos, size }
	end)

	local makeTable = pfn()
	local terra makeTableImpl(numPlates: int, pos: Vec2) : TableT
		var size = ngaussian(10.0, 1.0)
		lowerBound(size, 5.0, 0.1)
		size = lowerClamp(size, 0.0)
		var t = TableT.stackAlloc(pos, size)
		-- Size of the child plates should all be similar, so we'll
		--   correlate them through this variable.
		var sizePrior = ngaussian(size/5.0, 0.5)
		for i=0,numPlates do
			t.plates:push(makePlate(&t, sizePrior))
		end
		-- Factor: plates don't overlap
		for i=0,numPlates-1 do
			var p1 = t.plates:getPointer(i)
			for j=i+1,numPlates do
				var p2 = t.plates:getPointer(j)
				factor(softEq(p1:intersectArea(p2), 0.0, 0.2))
			end
		end
		-- Factor: plates occupy a large portion of table area
		var areasum = real(0.0)
		for i=0,numPlates do
			areasum = areasum + t.plates(i):area()
		end
		factor(softEq(areasum/t:area(), 0.7, 0.1))
		return t
	end
	terra makeTableImpl(numPlates: int) : TableT
		var pos = Vec2.stackAlloc(nuniformNoPrior(0.0, roomWidth),
								  nuniformNoPrior(0.0, roomHeight))
		var t = makeTable(numPlates, pos)
		bound(pos(0), t.size, roomWidth-t.size, 1.0)
		bound(pos(1), t.size, roomHeight-t.size, 1.0)
		return t
	end
	makeTable:define(makeTableImpl)

	--------------------------------------------

	return terra()

		var room = RoomT.stackAlloc(roomWidth, roomHeight)
		var tables = &room.tables

		-- Spawn pillars
		var pillarSize = 12.0
		var p1pos = Vec2.stackAlloc(0.25*roomWidth, 0.5*roomHeight)
		var p2pos = Vec2.stackAlloc(0.75*roomWidth, 0.5*roomHeight)
		room.pillars:push(PillarT {p1pos, pillarSize})
		room.pillars:push(PillarT {p2pos, pillarSize})

		-- Spawn tables
		for i=0,numTables do
			var t = makeTable(numPlates)
			tables:push(t)
			m.destruct(t)
		end

		-- Non-overlap (table-table)
		for i=0,tables.size-1 do
			for j=i+1,tables.size do
				var a = tables:getPointer(i):intersectArea(tables:getPointer(j))
				factor(softEq(a, 0.0, 1.0))
			end
		end

		-- Minimum distance between tables
		var minDistBetween = 10.0
		for i=0,tables.size-1 do
			var t1 = tables:getPointer(i)
			for j=i+1,tables.size do
				var t2 = tables:getPointer(j)
				var mind = t1.size + t2.size + minDistBetween
				var d = t1.pos:dist(t2.pos)
				if d < mind then
					factor(softEq(d, mind, 2.0))
				end
			end
		end

		-- Non-overlap (table-pillar)
		for i=0,tables.size do
			var t = tables:getPointer(i)
			factor(softEq(t:intersectArea(&room.pillars(0)), 0.0, 1.0))
			factor(softEq(t:intersectArea(&room.pillars(1)), 0.0, 1.0))
		end 

		return room
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

		-- Render all frames, save to image, write to disk
		var im = RGBImage.stackAlloc(imageWidth, imageHeight)
		var framename: int8[1024]
		var framenumber = 0
		var tableColor = Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
		var plateColor = Color3d.stackAlloc(255/255.0, 127/255.0, 14/255.0)
		var pillarColor = Color3d.stackAlloc(44/255.0, 160/255.0, 44/255.0)
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			var room = &samples(i).value
			-- Set up transforms
			gl.glMatrixMode(gl.mGL_PROJECTION())
			gl.glLoadIdentity()
			gl.gluOrtho2D(0, room.width, 0, room.height)
			gl.glMatrixMode(gl.mGL_MODELVIEW())
			gl.glLoadIdentity()
			-- Draw
			var tables = &room.tables
			for i=0,tables.size do
				var table = tables:getPointer(i)
				drawCircle(table.pos, table.size, tableColor)
				for i=0,table.plates.size do
					var plate = table.plates:getPointer(i)
					drawCircle(plate.pos, plate.size, plateColor)
				end
			end
			for i=0,room.pillars.size do
				var pillar = room.pillars:getPointer(i)
				drawCircle(pillar.pos, pillar.size, pillarColor)
			end
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

local numsamps = 10000
local verbose = true
local kernel = HMC({numSteps=20})
-- local kernel = RandomWalk()
local terra doInference()
	return [mcmc(layoutModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

local moviename = arg[1] or "movie"
renderSamples(samples, moviename)


