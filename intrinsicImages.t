terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local image = terralib.require("image")
local ad = terralib.require("ad")
local SuperpixelImage = terralib.require("superpixelImage")

local Color3d = Vec(double, 3)
local RGBImaged = image.Image(double, 3)

local C = terralib.includecstring [[
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
inline void flush() { fflush(stdout); }
]]


local IntrinsicImages = templatize(function(real)
	local SuperpixelImageT = SuperpixelImage(real)
	local Color3 = Vec(real, 3)
	local RGBImage = image.Image(real, 3)

	local struct IntrinsicImagesT
	{
		parentImage: &SuperpixelImageT,

		-- Data per superpixel
		shading: Vector(real),
		reflectance: Vector(Color3)
	}

	terra IntrinsicImagesT:__construct() : {}
		self.parentImage = nil
		m.init(self.shading)
		m.init(self.reflectance)
	end

	terra IntrinsicImagesT:__construct(parent: &SuperpixelImageT) : {}
		self:__construct()
		self.parentImage = parent
		self.shading:resize(parent:numSuperpixels())
		self.reflectance:resize(parent:numSuperpixels())
	end

	terra IntrinsicImagesT:__copy(other: &IntrinsicImagesT)
		self.parentImage = other.parentImage
		self.shading = m.copy(other.shading)
		self.reflectance = m.copy(other.reflectance)
	end

	terra IntrinsicImagesT:__destruct()
		m.destruct(self.shading)
		m.destruct(self.reflectance)
	end

	terra IntrinsicImagesT:visualize(shadImage: &RGBImage, reflImage: &RGBImage, reconstImage: &RGBImage)
		shadImage:resize(self.parentImage:fullResWidth(), self.parentImage:fullResHeight())
		reflImage:resize(shadImage.width, shadImage.height)
		reconstImage:resize(shadImage.width, shadImage.height)
		for y=0,shadImage.height do
			for x=0,shadImage.width do
				var shad = real(0.0)
				var refl = Color3.stackAlloc(0.0, 0.0, 0.0)
				for i=0,self.parentImage.pixelNeighbors(x,y).size do
					var ni = self.parentImage.pixelNeighbors(x,y)(i)
					var w = self.parentImage.pixelNeighborWeights(x,y)(i)
					shad = shad + w*self.shading(ni)
					refl = refl + w*self.reflectance(ni)
				end
				var shadTriple = Color3.stackAlloc(shad, shad, shad)
				shadImage:setPixel(x, shadImage.height-y-1, &shadTriple)
				reflImage:setPixel(x, reflImage.height-y-1, &refl)
				var reconstColor = shad*refl
				reconstImage:setPixel(x, reconstImage.height-y-1, &reconstColor)
			end
		end
	end

	m.addConstructors(IntrinsicImagesT)
	return IntrinsicImagesT
end)


-- The model is parameterized by:
--   * a SuperpixelImage(double)
local function modelGenerator(spImage)

	return probcomp(function()
		local Color3 = Vec(real, 3)

		-- Factor weights
		local reconstructionSoftness = 0.01

		-- We have a copy of spImage that we use for
		--    scratch work in evaluating certain factors
		local scratchImage = terralib.new(SuperpixelImage(real))
		local terra copySpImage()
			scratchImage = [m.templatecopy(SuperpixelImage(real))](spImage)
		end
		copySpImage()
		m.gc(scratchImage)

		-- ERP shorthand
		local boundedUniform = macro(function(lo, hi, opts)
			local valquote = nil
			if opts then
				local OpsType = opts:gettype()
				local struct NewOpsType {}
				for _,e in ipairs(OpsType.entries) do
					table.insert(NewOpsType.entries, {field=e.field, type=e.type})
				end
				table.insert(NewOpsType.entries, {field="lowerBound", type=lo:gettype()})
				table.insert(NewOpsType.entries, {field="upperBound", type=hi:gettype()})
				table.insert(NewOpsType.entries, {field="structural", type=bool})
				valquote = quote
					var newopts = NewOpsType(opts)
					newopts.structural = false
					newopts.lowerBound = lo
					newopts.upperBound = hi
				in
					uniform(lo, hi, newopts)
				end
			else
				valquote = `uniform(lo, hi, {structural=false, lowerBound=lo, upperBound=hi})
			end
			return valquote
		end)

		-- This is the actual computation we do inference on
		return terra()

			-- Generate random intrinsic images, fill with random values
			var intrinsics = [IntrinsicImages(real)].stackAlloc(&scratchImage)
			for i=0,scratchImage:numSuperpixels() do
				intrinsics.shading(i) = boundedUniform(0.0, 1.0)
				intrinsics.reflectance(i) = Color3.stackAlloc(boundedUniform(0.0, 1.0),
															  boundedUniform(0.0, 1.0),
															  boundedUniform(0.0, 1.0))
			end

			-- Reconstruction constraint
			for s=0,scratchImage:numSuperpixels() do
				var color = intrinsics.shading(s) * intrinsics.reflectance(s)
				var err = color - Color3(spImage.superpixelColors(s))
				manifold(err(0), reconstructionSoftness)
				manifold(err(1), reconstructionSoftness)
				manifold(err(2), reconstructionSoftness)
			end

			return intrinsics
		end
	end)
end

-- We visualize the output samples by writing out an HTML file with a bunch of rows, each of
--    which shows:
-- * Reconstructed image thumbnail, shading thumbnail, reflectance thumbnail
local function visualizeSamples(dirname, samples, spImage)
	util.wait(string.format("mkdir %s", dirname))
	local htmlFilename = string.format("%s/samples.html", dirname)
	local imgdir = string.format("%s/img", dirname)
	util.wait(string.format("mkdir %s", imgdir))
	local reconstImgBasename = imgdir .. "/%06d_reconstruction.png"
	local shadImgBasename = imgdir .. "/%06d_shading.png"
	local reflImgBasename = imgdir .. "/%06d_reflectance.png"	
	local reconstImgRelBasename = "img/%06d_reconstruction.png"
	local shadImgRelBasename = "img/%06d_shading.png"
	local reflImgRelBasename = "img/%06d_reflectance.png"	

	local css =
	[[
	<style>
	.thumb
	{
		height: 250px;
	}
	.spacer
	{
		width: 30px;
		height: 250px;
		background-color: transparent;
	}
	</style>
	]]

	local quant = macro(function(colorChannel)
		return `int(255*colorChannel)
	end)

	local terra viz()
		var htmlfile = C.fopen(htmlFilename, "w")

		C.fprintf(htmlfile, css)
		C.fprintf(htmlfile, "<table>\n")

		var shadImage = RGBImaged.stackAlloc()
		var reflImage = RGBImaged.stackAlloc()
		var reconstImg = RGBImaged.stackAlloc()
		var filenamebuf : int8[1024]
		for i=0,samples.size do
			C.printf(" %d/%u\r", i+1, samples.size)
			C.flush()
			var intrinsics = &samples(i).value
			var lp = samples(i).logprob
			C.fprintf(htmlfile, "<tr>\n")

			-- Visualize intrinsics and reconstruction
			intrinsics:visualize(&shadImage, &reflImage, &reconstImg)

			-- Save to disk, write HTML data
			C.sprintf(filenamebuf, reconstImgBasename, i)
			[RGBImaged.save(uint8)](&reconstImg, image.Format.PNG, filenamebuf)
			C.sprintf(filenamebuf, reconstImgRelBasename, i)
			C.fprintf(htmlfile, "<td><img src='%s' class='thumb' title='%g'/></td>\n", filenamebuf, lp)
			C.sprintf(filenamebuf, shadImgBasename, i)
			[RGBImaged.save(uint8)](&shadImage, image.Format.PNG, filenamebuf)
			C.sprintf(filenamebuf, shadImgRelBasename, i)
			C.fprintf(htmlfile, "<td><div class='spacer'/></td>\n")
			C.fprintf(htmlfile, "<td><img src='%s' class='thumb'/></td>\n", filenamebuf, lp)
			C.sprintf(filenamebuf, reflImgBasename, i)
			[RGBImaged.save(uint8)](&reflImage, image.Format.PNG, filenamebuf)
			C.sprintf(filenamebuf, reflImgRelBasename, i)
			C.fprintf(htmlfile, "<td><div class='spacer'/></td>\n")
			C.fprintf(htmlfile, "<td><img src='%s' class='thumb'/></td>\n", filenamebuf, lp)

			C.fprintf(htmlfile, "</tr>")
		end

		m.destruct(reconstImg)
		m.destruct(shadImage)
		m.destruct(reflImage)

		C.fprintf(htmlfile, "</table>\n")
		C.fclose(htmlfile)
		C.printf("\n")
	end
	print("Visualizing samples...")
	viz()
	print("DONE")
end





local spimgdir = "superpixelExamples_400/turtle"
local spimg = SuperpixelImage(double).fromFiles(spimgdir)()
local model = modelGenerator(spimg)
local trace = terralib.require("prob.trace")
local inf = terralib.require("prob.inference")
local terra doInference()
	return [sampleByRepeatedBurnin(model, HMC({numSteps=10, relaxManifolds=true}), {numsamps=800, verbose=true}, 10)]

	-- -- Burn in
	-- var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]
	-- var burnInKernel = [HMC({numSteps=10, relaxManifolds=true})()]
	-- currTrace = [inf.mcmcSample(model, {numsamps=1000, verbose=true})](currTrace, burnInKernel, nil)
	-- m.delete(burnInKernel)

	-- -- Run very long HMC trajectories to (hopefully) generate distant samples
	-- var samps = [inf.SampleVectorType(model)].stackAlloc()
	-- var mixKernel = [HMC({numSteps=1000, relaxManifolds=true})()]
	-- currTrace = [inf.mcmcSample(model, {numsamps=20, lag=20, verbose=true})](currTrace, burnInKernel, &samps)
	-- m.delete(mixKernel)
	-- m.delete(currTrace)
	-- return samps
end
local samps = m.gc(doInference())
local outputName = arg[1] or "output"
visualizeSamples(string.format("renders/intristicImages/%s", outputName), samps, spimg)








