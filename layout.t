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
local glUtils = terralib.require("glUtils")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)

local Plate = templatize(function(real)
	local CircleT = glUtils.Circle(real)
	local struct PlateT {}
	inheritance.staticExtend(CircleT, PlateT)
	return PlateT
end)

local Table = templatize(function(real)
	local CircleT = glUtils.Circle(real)
	local PlateT = Plate(real)
	local Vec2 = Vec(real, 2)
	local struct TableT { plates: Vector(PlateT) }
	inheritance.staticExtend(CircleT, TableT)
	terra TableT:__construct() : {}
		m.init(self.pos)
		self.size = 0.0
		m.init(self.plates)
	end
	terra TableT:__construct(pos: Vec2, size: real, color: Color3d) : {}
		self.pos = pos
		self.size = size
		self.color = color
		m.init(self.plates)
	end
	terra TableT:__destruct()
		m.destruct(self.plates)
	end
	terra TableT:__copy(other: &TableT)
		self.pos = other.pos
		self.size = other.size
		self.color = other.color
		self.plates = m.copy(other.plates)
	end
	terra TableT:render()
		CircleT.render(self)
		for i=0,self.plates.size do
			var plate = self.plates:getPointer(i)
			plate:render()
		end
	end
	m.addConstructors(TableT)
	return TableT
end)

local Pillar = templatize(function(real)
	local CircleT = glUtils.Circle(real)
	local struct PillarT {}
	inheritance.staticExtend(CircleT, PillarT)
	terra PillarT:__construct() : {}
	end
	m.addConstructors(PillarT)
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
	terra RoomT:render()
		for i=0,self.tables.size do
			self.tables:getPointer(i):render()
		end
		for i=0,self.pillars.size do
			self.pillars:getPointer(i):render()
		end
	end
	m.addConstructors(RoomT)
	return RoomT
end)


local function layoutModel()
	local roomWidth = `100.0
	local roomHeight = `100.0
	local numTables = 4
	local numPlates = 4
	local Vec2 = Vec(real, 2)
	local PlateT = Plate(real)
	local TableT = Table(real)
	local PillarT = Pillar(real)
	local RoomT = Room(real)

	local tableColor = `Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
	local pillarColor = `Color3d.stackAlloc(44/255.0, 160/255.0, 44/255.0)
	local plateColor = `Color3d.stackAlloc(255/255.0, 127/255.0, 14/255.0)

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
		return PlateT { pos, size, plateColor }
	end)

	local makeTable = pfn()
	local terra makeTableImpl(numPlates: int, pos: Vec2) : TableT
		var size = ngaussian(10.0, 1.0)
		lowerBound(size, 5.0, 0.1)
		size = lowerClamp(size, 0.0)
		var t = TableT.stackAlloc(pos, size, tableColor)
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
		-- bound(pos(0), t.size, roomWidth-t.size, 1.0)
		-- bound(pos(1), t.size, roomHeight-t.size, 1.0)
		return t
	end
	makeTable:define(makeTableImpl)

	--------------------------------------------

	return terra()

		var room = RoomT.stackAlloc(roomWidth, roomHeight)
		var tables = &room.tables

		-- -- Spawn pillars
		-- var pillarSize = 12.0
		-- var p1pos = Vec2.stackAlloc(0.25*roomWidth, 0.5*roomHeight)
		-- var p2pos = Vec2.stackAlloc(0.75*roomWidth, 0.5*roomHeight)
		-- room.pillars:push(PillarT {p1pos, pillarSize, pillarColor})
		-- room.pillars:push(PillarT {p2pos, pillarSize, pillarColor})

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

		-- Tables lie on a circle
		var circCenter = Vec2.stackAlloc(nuniformNoPrior(0.0, roomWidth),
								         nuniformNoPrior(0.0, roomHeight))
		bound(circCenter(0), 0.0, room.width, 1.0)
		bound(circCenter(1), 0.0, room.height, 1.0)
		var circRad = 30.0
		for i=0,tables.size do
			var t = tables:getPointer(i)
			factor(softEq(t.pos:dist(circCenter), circRad, 0.1))
		end

		-- Circle center itself lies on a circle
		var metaCircCenter = Vec2.stackAlloc(room.width/2.0, room.height/2.0)
		var metaCircRad = 30.0
		factor(softEq(circCenter:dist(metaCircCenter), metaCircRad, 0.1))

		-- -- Equal spacing around the circle
		-- var prevDist = tables(0).pos:dist(tables(1).pos)
		-- for i=1,tables.size do
		-- 	var t1 = tables:getPointer(i)
		-- 	var t2 = tables:getPointer((i+1)%tables.size)
		-- 	var currDist = t1.pos:dist(t2.pos)
		-- 	factor(softEq(currDist, prevDist, 0.5))
		-- 	prevDist = currDist
		-- end

		-- -- Minimum distance between tables
		-- var minDistBetween = 10.0
		-- for i=0,tables.size-1 do
		-- 	var t1 = tables:getPointer(i)
		-- 	for j=i+1,tables.size do
		-- 		var t2 = tables:getPointer(j)
		-- 		var mind = t1.size + t2.size + minDistBetween
		-- 		var d = t1.pos:dist(t2.pos)
		-- 		if d < mind then
		-- 			factor(softEq(d, mind, 2.0))
		-- 		end
		-- 	end
		-- end

		-- -- Non-overlap (table-pillar)
		-- for i=0,tables.size do
		-- 	var t = tables:getPointer(i)
		-- 	factor(softEq(t:intersectArea(&room.pillars(0)), 0.0, 1.0))
		-- 	factor(softEq(t:intersectArea(&room.pillars(1)), 0.0, 1.0))
		-- end 

		return room
	end
end

----------------------------------

local kernelType = HMC
-- local kernelType = RandomWalk
local hmcNumSteps = 100
local numsamps = 200
local verbose = true

if kernelType == RandomWalk then numsamps = 0.75*hmcNumSteps*numsamps end
local kernel = nil
if kernelType == RandomWalk then
	kernel = RandomWalk()
else
	kernel = HMC({numSteps=hmcNumSteps})
end
local terra doInference()
	return [mcmc(layoutModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

local moviename = arg[1] or "movie"
local imageWidth = 200
local imageHeight = 200
glUtils.renderSamples(samples, moviename, imageWidth, imageHeight)


