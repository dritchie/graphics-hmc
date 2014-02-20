terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local Grid2D = terralib.require("grid").Grid2D
local image = terralib.require("image")

local Color3d = Vec(double, 3)
local RGBImaged = image.Image(double, 3)

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
		superpixelNeighbors: Vector(Vector(uint)),
		superpixelNeighborWeights: Vector(Vector(real)),

		-- Per-pixel data
		pixelNeighbors: Grid2D(Vector(uint)),
		pixelNeighborWeights: Grid2D(Vector(real))
	}

	terra SuperpixelImageT:__construct() : {}
		m.init(self.superpixelColors)
		m.init(self.superpixelNeighbors)
		m.init(self.superpixelNeighborWeights)
		m.init(self.pixelNeighbors)
		m.init(self.pixelNeighborWeights)
	end

	local numSuperpixelNeighbors = 30
	local numPixelNeighbors = 10
	SuperpixelImageT.fromFiles = function(dirname)
		local superpixelInfoFilename = string.format("%s/superpixelInfo.txt", dirname)
		local pixelInfoFilename = string.format("%s/pixelInfo.txt", dirname)
		local superpixelAssignmentsFilename = string.format("%s/superpixelAssignments.txt", dirname)
		return terra()
			var spimg = SuperpixelImageT.stackAlloc()
			var buffer : int8[1024]
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
			var assfile = C.fopen(superpixelAssignmentsFilename, "r")
			bufptr = C.fgets(buffer, 1023, assfile)
			var height = C.atoi(C.strtok(buffer, "\t"))
			var width = C.atoi(C.strtok(nil, "\t"))
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
			return spimg
		end
	end

	SuperpixelImageT.__templatecopy = templatize(function(real2)
		return terra(self: &SuperpixelImageT, other: &SuperpixelImage(real2))
			self.superpixelColors = [m.templatecopy(Vector(Color3))](other.superpixelColors)
			self.superpixelNeighbors = m.copy(other.superpixelNeighbors)
			self.superpixelNeighborWeights = [m.templatecopy(Vector(Vector(real)))](other.superpixelNeighborWeights)
			self.pixelNeighbors = m.copy(other.pixelNeighbors)
			self.pixelNeighborWeights = [m.templatecopy(Grid2D(Vector(real)))](other.pixelNeighborWeights)
		end
	end)

	terra SuperpixelImageT:__destruct()
		m.destruct(self.superpixelColors)
		m.destruct(self.superpixelNeighbors)
		m.destruct(self.superpixelNeighborWeights)
		m.destruct(self.pixelNeighbors)
		m.destruct(self.pixelNeighborWeights)
	end

	terra SuperpixelImageT:numSuperpixels()
		return self.superpixelColors.size
	end
	util.inline(SuperpixelImageT.methods.numSuperpixels)

	terra SuperpixelImageT:reconstructFullRes()
		var img = RGBImage.stackAlloc(self.pixelNeighbors.rows, self.pixelNeighbors.cols)
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
		return img
	end

	m.addConstructors(SuperpixelImageT)
	return SuperpixelImageT
end)


-- Flags for different layer blending modes
local LayerBlendMode =
{
	Linear = 0,
	Over = 1
}


-- The object inferred by the system is a set of layers
--    (A color for each layer, plus layer weights for 
--     each superpixel).
local Layering = templatize(function(real)
	local Color3 = Vec(real, 3)
	local SuperpixelImageT = SuperpixelImage(real)
	local struct LayeringT
	{
		layerColors: Vector(Color3),
		layerWeights: Vector(Vector(real))
	}

	terra LayeringT:__construct() : {}
		m.init(self.layerColors)
		m.init(self.layerWeights)
	end

	terra LayeringT:__construct(img: &SuperpixelImageT, numLayers: uint) : {}
		self:__construct()
		self.layerColors:resize(numLayers)
		self.layerWeights:resize(img:numSuperpixels())
		for s=0,img:numSuperpixels() do
			self.layerWeights(s):resize(numLayers)
		end
	end

	terra LayeringT:__copy(other: &LayeringT)
		self.layerColors = m.copy(other.layerColors)
		self.layerWeights = m.copy(other.layerWeights)
	end

	terra LayeringT:__destruct() : {}
		m.destruct(self.layerColors)
		m.destruct(self.layerWeights)
	end

	terra LayeringT:numLayers()
		return self.layerColors.size
	end
	util.inline(LayeringT.methods.numLayers)

	-- Apply this layering to a SuperpixelImage, modifying it in place
	LayeringT.apply = templatize(function(blendMode)
		return terra(self: &LayeringT, spImage: &SuperpixelImageT)
			for s=0,spImage:numSuperpixels() do
				var color = Color3.stackAlloc(0.0, 0.0, 0.0)
				for l=0,self:numLayers() do
					-- Linear blending
					[util.optionally(blendMode == LayerBlendMode.Linear, function() return quote
						color = color + self.layerWeights(s)(l) * self.layerColors(l)
					end end)]
					-- Alpha compositing
					[util.optionally(blendMode == LayerBlendMode.Over, function() return quote
						var alpha = self.layerWeights(s)(l)
						color = (1.0 - alpha)*color + alpha*self.layerColors(l)
					end end)]
				end
				spImage.superpixelColors(s) = color
			end
		end
	end)

	m.addConstructors(LayeringT)
	return LayeringT
end)


-- The model is parameterized by:
-- * a SuperpixelImage(double)
-- * a number of desired layers
-- * a LayerBlendMode
-- TODO: Potentially also let the numer of layers be variable?
local function modelGenerator(spImage, numLayers, blendMode)
	local unityStrength = 0.001
	local localLinearityStrength = 0.001
	local reconstructionStrength = 0.01
	return probcomp(function()
		local Color3 = Vec(real, 3)

		-- We have a (global) copy of spImage that we use for
		--    scratch work in evaluating certain factors
		-- local scratchImage = global(SuperpixelImage(real))
		local scratchImage = terralib.new(SuperpixelImage(real))
		local terra copySpImage()
			scratchImage = [m.templatecopy(SuperpixelImage(real))](spImage)
		end
		copySpImage()
		-- m.gc(scratchImage:get())
		m.gc(scratchImage)

		-- ERP shorthand
		local boundedUniform = macro(function(lo, hi)
			return `uniform(lo, hi, {structural=false, lowerBound=lo, upperBound=hi})
		end)

		-- This is the actual computation we do inference on
		return terra()
			var layering = [Layering(real)].stackAlloc(&scratchImage, numLayers)

			-- Sample random layer colors
			for l=0,numLayers do
				layering.layerColors(l) = Color3.stackAlloc(boundedUniform(0.0, 1.0),
															boundedUniform(0.0, 1.0),
															boundedUniform(0.0, 1.0))
			end
			-- Sample random layer weights
			for s=0,scratchImage:numSuperpixels() do
				for l=0,numLayers do
					layering.layerWeights(s)(l) = boundedUniform(0.0, 1.0)
				end
			end

			-- Apply layering to the scratch image
			[Layering(real).apply(blendMode)](&layering, &scratchImage)

			-- Unity constraint
			-- TODO: disable when using alpha blending?
			for s=0,scratchImage:numSuperpixels() do
				var wsum = real(0.0)
				for l=0,numLayers do
					wsum = wsum + layering.layerWeights(s)(l)
				end
				manifold(wsum, unityStrength)
			end

			-- Local linearity constraint
			for s=0,scratchImage:numSuperpixels() do
				var csum = Color3.stackAlloc(0.0, 0.0, 0.0)
				for ni=0,scratchImage.superpixelNeighbors(s).size do
					var n = scratchImage.superpixelNeighbors(s)(ni)
					var w = scratchImage.superpixelNeighborWeights(s)(ni)
					csum = csum + w*scratchImage.superpixelColors(n)
				end
				var err = csum - scratchImage.superpixelColors(s)
				manifold(err(0), localLinearityStrength)
				manifold(err(1), localLinearityStrength)
				manifold(err(2), localLinearityStrength)
			end

			-- Reconstruction constraint
			for s=0,scratchImage:numSuperpixels() do
				var err = scratchImage.superpixelColors(s) - Color3(spImage.superpixelColors(s))
				manifold(err(0), reconstructionStrength)
				manifold(err(1), reconstructionStrength)
				manifold(err(2), reconstructionStrength)
			end

			-- TODO: Additional constraints, such as:
			-- * Layer sparsity
			-- * Layer non-overlap
			-- * Layer color diversity

			return layering
		end
	end)
end




local spimgdir = "superpixelExamples/bird"
local numLayers = 5
local blendMode = LayerBlendMode.Linear
local spimg = SuperpixelImage(double).fromFiles(spimgdir)()
local model = modelGenerator(spimg, numLayers, blendMode)
local terra doInference()
	return [mcmc(model, HMC({numSteps=1, relaxManifolds=true}), {numsamps=1000, verbose=true})]
end
local samps = m.gc(doInference())

-- local terra imgTest()
-- 	var img = spimg:reconstructFullRes()
-- 	[RGBImaged.save(uint8)](&img, image.Format.PNG, "reconstruction.png")
-- end
-- imgTest()








