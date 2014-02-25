local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local Grid2D = terralib.require("grid").Grid2D
local image = terralib.require("image")

local C = terralib.includecstring [[
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
]]


-- Input images to the system have been decomposed into superpixels
--    but contain enough information to have their full resolution
--    reconstructed using a linear combination of superpixels
local SuperpixelImage
SuperpixelImage = templatize(function(real)
	local Color3 = Vec(real, 3)
	local RGBImage = image.Image(real, 3)
	local struct SuperpixelImageT
	{
		-- Per-superpixel data
		superpixelColors: Vector(Color3),
		superpixelAreas: Vector(uint),
		superpixelNeighbors: Vector(Vector(uint)),
		superpixelNeighborWeights: Vector(Vector(real)),

		-- Per-pixel data
		pixelNeighbors: Grid2D(Vector(uint)),
		pixelNeighborWeights: Grid2D(Vector(real)),
		pixelToSuperpixel: Grid2D(uint)
	}

	terra SuperpixelImageT:__construct() : {}
		m.init(self.superpixelColors)
		m.destruct(self.superpixelAreas)
		m.init(self.superpixelNeighbors)
		m.init(self.superpixelNeighborWeights)
		m.init(self.pixelNeighbors)
		m.init(self.pixelNeighborWeights)
		m.init(self.pixelToSuperpixel)
	end

	local numSuperpixelNeighbors = 30
	local numPixelNeighbors = 10
	SuperpixelImageT.fromFiles = function(dirname)
		local superpixelInfoFilename = string.format("%s/superpixelInfo.txt", dirname)
		local pixelInfoFilename = string.format("%s/pixelInfo.txt", dirname)
		local superpixelAssignmentsFilename = string.format("%s/superpixelAssignments.txt", dirname)
		return terra()
			var spimg = SuperpixelImageT.stackAlloc()
			var buffer : int8[4096]	-- This is actually dependent on the size of the image...
			var bufptr : &int8 = nil

			-- Read info about superpixels
			var spfile = C.fopen(superpixelInfoFilename, "r")
			bufptr = C.fgets(buffer, 1023, spfile)	-- skip header
			while C.feof(spfile) == 0 do
				-- Each line has: id, r, g, b, x, y, neighbors, neighborWeights
				bufptr = C.fgets(buffer, 1023, spfile)
				if bufptr == nil then break end
				var id = C.atoi(C.strtok(buffer, "\t"))
				var r = C.atoi(C.strtok(nil, "\t"))/255.0
				var g = C.atoi(C.strtok(nil, "\t"))/255.0
				var b = C.atoi(C.strtok(nil, "\t"))/255.0
				spimg.superpixelColors:push(Color3.stackAlloc(r,g,b))
				var x = C.atoi(C.strtok(nil, "\t"))
				var y = C.atoi(C.strtok(nil, "\t"))
				spimg.superpixelNeighbors:resize(id+1)
				spimg.superpixelNeighbors:backPointer():resize(numSuperpixelNeighbors)
				for i=0,numSuperpixelNeighbors do
					var ni = C.atoi(C.strtok(nil, "\t"))
					spimg.superpixelNeighbors:backPointer()(i) = ni
				end
				spimg.superpixelNeighborWeights:resize(id+1)
				spimg.superpixelNeighborWeights:backPointer():resize(numSuperpixelNeighbors)
				for i=0,numSuperpixelNeighbors do
					var nw = C.atof(C.strtok(nil, "\t"))
					spimg.superpixelNeighborWeights:backPointer()(i) = nw
				end
			end
			C.fclose(spfile)

			-- Figure out the resolution of the full-res image
			--    and read info about pixel-to-superpixel assignments
			var assfile = C.fopen(superpixelAssignmentsFilename, "r")
			bufptr = C.fgets(buffer, 1023, assfile)
			var height = C.atoi(C.strtok(buffer, "\t"))
			var width = C.atoi(C.strtok(nil, "\t"))
			spimg.pixelToSuperpixel:resize(width, height)
			for y=0,height do
				bufptr = C.fgets(buffer, 4095, assfile)
				var sid = C.atoi(C.strtok(buffer, "\t"))
				spimg.pixelToSuperpixel(0, y) = sid
				for x=1,width do
					sid = C.atoi(C.strtok(nil, "\t"))
					spimg.pixelToSuperpixel(x, y) = sid
				end
			end
			C.fclose(assfile)
			spimg.pixelNeighbors:resize(width, height)
			spimg.pixelNeighborWeights:resize(width, height)

			-- Read info about pixels
			var pfile = C.fopen(pixelInfoFilename, "r")
			bufptr = C.fgets(buffer, 1023, pfile)	-- skip header
			while C.feof(pfile) == 0 do
				-- Each line has: id, r, g, b, x, y, neighbors, neighborWeights
				bufptr = C.fgets(buffer, 1023, pfile)
				if bufptr == nil then break end
				var id = C.atoi(C.strtok(buffer, "\t"))
				var r = C.atoi(C.strtok(nil, "\t"))/255.0
				var g = C.atoi(C.strtok(nil, "\t"))/255.0
				var b = C.atoi(C.strtok(nil, "\t"))/255.0
				var x = C.atoi(C.strtok(nil, "\t"))
				var y = C.atoi(C.strtok(nil, "\t"))
				spimg.pixelNeighbors(x,y):resize(numPixelNeighbors)
				for i=0,numPixelNeighbors do
					var ni = C.atoi(C.strtok(nil, "\t"))
					spimg.pixelNeighbors(x,y)(i) = ni
				end
				spimg.pixelNeighborWeights(x,y):resize(numPixelNeighbors)
				for i=0,numPixelNeighbors do
					var nw = C.atof(C.strtok(nil, "\t"))
					spimg.pixelNeighborWeights(x,y)(i) = nw
				end
			end
			C.fclose(pfile)

			-- Compute superpixel areas
			m.destruct(spimg.superpixelAreas)
			spimg.superpixelAreas = [Vector(uint)].stackAlloc(spimg:numSuperpixels(), 0)
			for y=0,height do
				for x=0,width do
					var sid = spimg.pixelToSuperpixel(x,y)
					spimg.superpixelAreas(sid) = spimg.superpixelAreas(sid) + 1
				end
			end

			return spimg
		end
	end

	SuperpixelImageT.__templatecopy = templatize(function(real2)
		return terra(self: &SuperpixelImageT, other: &SuperpixelImage(real2))
			self.superpixelColors = [m.templatecopy(Vector(Color3))](other.superpixelColors)
			self.superpixelAreas = m.copy(other.superpixelAreas)
			self.superpixelNeighbors = m.copy(other.superpixelNeighbors)
			self.superpixelNeighborWeights = [m.templatecopy(Vector(Vector(real)))](other.superpixelNeighborWeights)
			self.pixelNeighbors = m.copy(other.pixelNeighbors)
			self.pixelNeighborWeights = [m.templatecopy(Grid2D(Vector(real)))](other.pixelNeighborWeights)
			self.pixelToSuperpixel = m.copy(other.pixelToSuperpixel)
		end
	end)

	terra SuperpixelImageT:__destruct()
		m.destruct(self.superpixelColors)
		m.destruct(self.superpixelAreas)
		m.destruct(self.superpixelNeighbors)
		m.destruct(self.superpixelNeighborWeights)
		m.destruct(self.pixelNeighbors)
		m.destruct(self.pixelNeighborWeights)
		m.destruct(self.pixelToSuperpixel)
	end

	SuperpixelImageT.methods.numSuperpixels = macro(function(self)
		return `self.superpixelColors.size
	end)

	SuperpixelImageT.methods.fullResWidth = macro(function(self)
		return `self.pixelNeighbors.rows
	end)

	SuperpixelImageT.methods.fullResHeight = macro(function(self)
		return `self.pixelNeighbors.cols
	end)

	terra SuperpixelImageT:totalArea()
		return self:fullResHeight() * self:fullResWidth()
	end

	terra SuperpixelImageT:avgSuperpixelArea()
		var sum = 0U
		for i=0,self:numSuperpixels() do
			sum = sum + self.superpixelAreas(i)
		end
		return double(sum/self:numSuperpixels())
	end

	terra SuperpixelImageT:reconstructFullRes(img: &RGBImage)
		img:resize(self:fullResWidth(), self:fullResHeight())
		for y=0,img.height do
			for x=0,img.width do
				var color = Color3.stackAlloc(0.0, 0.0, 0.0)
				for n=0,self.pixelNeighbors(x,y).size do
					var ni = self.pixelNeighbors(x,y)(n)
					var nw = self.pixelNeighborWeights(x,y)(n)
					color = color + nw*self.superpixelColors(ni)
				end
				img:setPixel(x, img.height-y-1, &color)
			end
		end
	end

	m.addConstructors(SuperpixelImageT)
	return SuperpixelImageT
end)


return SuperpixelImage