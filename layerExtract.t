terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local Grid2D = terralib.require("grid").Grid2D
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
	local RGBImage = image.Image(real, 3)
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

	LayeringT.methods.numLayers = macro(function(self)
		return `self.layerColors.size
	end)

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

	terra LayeringT:visualizeLayer(layerIndex: uint, spImage: &SuperpixelImageT, layermask: &RGBImage)
		layermask:resize(spImage:fullResWidth(), spImage:fullResHeight())
		for y=0,layermask.height do
			for x=0,layermask.width do
				var weight = real(0.0)
				for i=0,spImage.pixelNeighbors(x,y).size do
					var ni = spImage.pixelNeighbors(x,y)(i)
					weight = weight + spImage.pixelNeighborWeights(x,y)(i)*self.layerWeights(ni)(layerIndex)
				end
				var color = [Vec(real, 3)].stackAlloc(weight, weight, weight)
				layermask:setPixel(x, layermask.height-y-1, &color)
			end
		end
	end

	m.addConstructors(LayeringT)
	return LayeringT
end)


-- The model is parameterized by:
--   * a SuperpixelImage(double)
--   * a number of desired layers
--   * a LayerBlendMode
--   * an optional table containing layer colors. If this is provided, then
--     colors will be fixed to these values.
-- TODO: Potentially also let the numer of layers be variable?
local function modelGenerator(spImage, numLayers, blendMode, optLayerColors)
	-- In the paper, the weights were reported as:
	--   * lambda_manifold = 1
	--   * lambda_reconstruct = 0.5
	--   * lambda_unity = 0.1
	-- So we should have the manifold constraint be twice as tight as the
	--    reconstruction constraint and ten times as tight as the unity constraint(?)
	local masterConstraintSoftness = 0.01
	local localLinearitySoftness = masterConstraintSoftness
	-- local localLinearitySoftness = 1000 * masterConstraintSoftness
	local reconstructionSoftness = 2 * masterConstraintSoftness
	local unitySoftness = 10 * masterConstraintSoftness

	-- Weights for 'softer' factors/constraints
	local masterFactorSoftness = 0.01
	local colorDiversitySoftness = 100 * masterFactorSoftness
	local layerEntropySoftness = 100 * masterFactorSoftness

	-- Misc stuff
	local maxColorDistSq = 3.0
	local maxColorDist = math.sqrt(maxColorDistSq)

	return probcomp(function()
		local Color3 = Vec(real, 3)

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
		local softBoundedUniform = macro(function(lo, hi, softness, opts)
			local valquote = nil
			if opts then
				local OpsType = opts:gettype()
				local struct NewOpsType {}
				for _,e in ipairs(OpsType.entries) do
					table.insert(NewOpsType.entries, {field=e.field, type=e.type})
				end
				table.insert(NewOpsType.entries, {field="structural", type=bool})
				table.insert(NewOpsType.entries, {field="hasPrior", type=bool})
				valquote = quote
					var newopts = NewOpsType(opts)
					newopts.structural = false
					newopts.hasPrior = false
				in
					uniform(lo, hi, newopts)
				end
			else
				valquote = `uniform(lo, hi, {structural=false, hasPrior=false})
			end
			local function penalty(val)
				return `factor((val*val)/(softness*softness))
			end
			return quote
				var x = [valquote]
				if x < lo then
					[penalty(`lo-x)]
				elseif x > hi then
					[penalty(`x-hi)]
				end
			in
				x
			end
		end)

		-- This is the actual computation we do inference on
		return terra()
			var layering = [Layering(real)].stackAlloc(&scratchImage, numLayers)

			-- Use fixed layer colors...
			[util.optionally(optLayerColors, function()
				local stmts = {}
				for i,c in pairs(optLayerColors) do
					table.insert(stmts, quote layering.layerColors([i-1]) = Color3.stackAlloc([c]) end)
				end
				return stmts
			end)]
			-- ...or sample random layer colors
			[util.optionally(not optLayerColors, function() return quote
				for l=0,numLayers do
					layering.layerColors(l) = Color3.stackAlloc(boundedUniform(0.0, 1.0, {mass=1.0}),
																boundedUniform(0.0, 1.0, {mass=1.0}),
																boundedUniform(0.0, 1.0, {mass=1.0}))
				end
			end end)]
			
			-- Sample random layer weights
			for s=0,scratchImage:numSuperpixels() do
				for l=0,numLayers do
					-- layering.layerWeights(s)(l) = boundedUniform(0.0, 1.0)
					-- layering.layerWeights(s)(l) = boundedUniform(0.0, 1.0, {initialVal=1.0/numLayers, mass=1.0})
					layering.layerWeights(s)(l) = gaussian(0.0, 0.2, {structural=false, initialVal=1.0/numLayers})
					-- layering.layerWeights(s)(l) = softBoundedUniform(0.0, 1.0, 1.0, {initialVal=1.0/numLayers})
				end
			end

			-- Apply layering to the scratch image
			[Layering(real).apply(blendMode)](&layering, &scratchImage)

			-- Unity constraint
			-- (Disable when using alpha blending)
			[util.optionally(blendMode ~= LayerBlendMode.Over, function() return quote
				for s=0,scratchImage:numSuperpixels() do
					var wsum = real(0.0)
					for l=0,numLayers do
						wsum = wsum + layering.layerWeights(s)(l)
					end
					var err = 1.0 - wsum
					manifold(err, unitySoftness)
				end
			end end)]

			-- -- Local linearity constraint
			-- for s=0,scratchImage:numSuperpixels() do
			-- 	var csum = Color3.stackAlloc(0.0, 0.0, 0.0)
			-- 	for ni=0,scratchImage.superpixelNeighbors(s).size do
			-- 		var n = scratchImage.superpixelNeighbors(s)(ni)
			-- 		var w = scratchImage.superpixelNeighborWeights(s)(ni)
			-- 		csum = csum + w*scratchImage.superpixelColors(n)
			-- 	end
			-- 	var err = csum - scratchImage.superpixelColors(s)
			-- 	manifold(err(0), localLinearitySoftness)
			-- 	manifold(err(1), localLinearitySoftness)
			-- 	manifold(err(2), localLinearitySoftness)
			-- end

			-- Reconstruction constraint
			-- var totalErr = real(0.0)
			for s=0,scratchImage:numSuperpixels() do
				-- totalErr = totalErr + scratchImage.superpixelColors(s):distSq(Color3(spImage.superpixelColors(s)))
				var err = scratchImage.superpixelColors(s) - Color3(spImage.superpixelColors(s))
				manifold(err(0), reconstructionSoftness)
				manifold(err(1), reconstructionSoftness)
				manifold(err(2), reconstructionSoftness)
			end
			-- totalErr = ad.math.sqrt(totalErr)
			-- manifold(totalErr, reconstructionSoftness)

			-- -- Layer color diversity
			-- for l1=0,numLayers-1 do
			-- 	for l2=l1+1,numLayers do
			-- 		var dist = layering.layerColors(l1):dist(layering.layerColors(l2))
			-- 		var penalty = maxColorDist - dist
			-- 		manifold(penalty, colorDiversitySoftness)
			-- 	end
			-- end

			-- -- Layer entropy
			-- for s=0,scratchImage:numSuperpixels() do
			-- 	var ent = real(0.0)
			-- 	for l=0,layering.layerColors.size do
			-- 		var p = layering.layerWeights(s)(l)
			-- 		if p > 0.0 then
			-- 			ent = ent + p*ad.math.log(p)
			-- 		end
			-- 	end
			-- 	manifold(ent, layerEntropySoftness)
			-- end

			return layering
		end
	end)
end

-- We visualize the output samples by writing out an HTML file with a bunch of rows, each of
--    which shows:
-- * Reconstructed image thumbnail
-- * List of (color swatch, layer mask thumbnail)
local function visualizeSamples(dirname, samples, spImage, blendMode, rowHeight, swatchWidth, spacing)
	rowHeight = rowHeight or 250
	swatchWidth = swatchWidth or 50
	spacing = spacing or 30
	util.wait(string.format("mkdir %s", dirname))
	local htmlFilename = string.format("%s/samples.html", dirname)
	local imgdir = string.format("%s/img", dirname)
	util.wait(string.format("mkdir %s", imgdir))
	local colorImgBasename = imgdir .. "/%06d_reconstruction.png"	
	local layerImgBasename = imgdir .. "/%06d_layer%d.png"
	local colorImgRelBasename = "img/%06d_reconstruction.png"
	local layerImgRelBasename = "img/%06d_layer%d.png"

	local css =
	[[
	<style>
	.thumb
	{
		height: 250px;
	}
	.swatch
	{
		width: 50px;
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
		var scratchImage = m.copy(spImage)
		var htmlfile = C.fopen(htmlFilename, "w")

		C.fprintf(htmlfile, css)
		C.fprintf(htmlfile, "<table>\n")

		var reconstImg = RGBImaged.stackAlloc()
		var layerMask = RGBImaged.stackAlloc()
		var filenamebuf : int8[1024]
		for i=0,samples.size do
			C.printf(" %d/%u\r", i+1, samples.size)
			C.flush()
			var layering = &samples(i).value
			var lp = samples(i).logprob
			C.fprintf(htmlfile, "<tr>\n")

			-- Apply layering to scratchImage, write to disk
			[Layering(double).apply(blendMode)](layering, &scratchImage)
			scratchImage:reconstructFullRes(&reconstImg)
			C.sprintf(filenamebuf, colorImgBasename, i)
			[RGBImaged.save(uint8)](&reconstImg, image.Format.PNG, filenamebuf)
			C.sprintf(filenamebuf, colorImgRelBasename, i)
			C.fprintf(htmlfile, "<td><img src='%s' class='thumb' title='%g'/></td>\n", filenamebuf, lp)

			-- Visualize each layer, write to disk
			for l=0,layering:numLayers() do
				layering:visualizeLayer(l, &scratchImage, &layerMask)
				C.sprintf(filenamebuf, layerImgBasename, i, l)
				[RGBImaged.save(uint8)](&layerMask, image.Format.PNG, filenamebuf)
				C.sprintf(filenamebuf, layerImgRelBasename, i, l)
				C.fprintf(htmlfile, "<td><div class='spacer'/></td>\n")
				C.fprintf(htmlfile, "<td><div class='swatch' style='background-color: rgb(%d, %d, %d)'/></td>\n",
					quant(layering.layerColors(l)(0)),
					quant(layering.layerColors(l)(1)),
					quant(layering.layerColors(l)(2)))
				C.fprintf(htmlfile, "<td><img src='%s' class='thumb'/></td>\n", filenamebuf)
			end
			C.fprintf(htmlfile, "</tr>")
		end

		m.destruct(scratchImage)
		m.destruct(reconstImg)
		m.destruct(layerMask)

		C.fprintf(htmlfile, "</table>\n")
		C.fclose(htmlfile)
		C.printf("\n")
	end
	print("Visualizing samples...")
	viz()
	print("DONE")
end





local layerColors = nil
local spimgdir = "superpixelExamples_400/bird"
local blendMode = LayerBlendMode.Linear
local numLayers = 5
-- local layerColors = {{21/255.0, 28/255.0, 25/255.0},
-- 					 {147/255.0, 70/255.0, 16/255.0},
-- 					 {102/255.0, 97/255.0, 14/255.0},
-- 					 {24/255.0, 160/255.0, 230/255.0},
-- 					 {231/255.0, 159/255.0, 14/255.0}}
-- local spimgdir = "superpixelExamples_400/alphaCircles"
-- local blendMode = LayerBlendMode.Over
-- local numLayers = 4
-- local layerColors = {{`1.0, `0.0, `0.0},
-- 					 {`0.0, `0.0, `1.0},
-- 					 {`0.0, `1.0, `0.0},
-- 					 {`1.0, `1.0, `1.0}}
local spimg = SuperpixelImage(double).fromFiles(spimgdir)()
local model = modelGenerator(spimg, numLayers, blendMode, layerColors)
local numHmcSteps = 10
local numKeptSamps = 5
local numDesiredOverallSteps = 8000
local numDesiredOverallSamps = math.ceil(numDesiredOverallSteps / numHmcSteps)
assert(numDesiredOverallSamps >= numKeptSamps)
local lag = math.floor(numDesiredOverallSamps / numKeptSamps)
local trace = terralib.require("prob.trace")
local inf = terralib.require("prob.inference")
local terra doInference()
	-- return [mcmc(model, HMC({numSteps=numHmcSteps, relaxManifolds=true}), {numsamps=numKeptSamps, lag=lag, verbose=true})]
	-- return [sampleByRepeatedBurnin(model, HMC({numSteps=10, relaxManifolds=true}), {numsamps=800, verbose=true}, 10)]

	-- Burn in
	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]
	var burnInKernel = [HMC({numSteps=10, relaxManifolds=true})()]
	currTrace = [inf.mcmcSample(model, {numsamps=1000, verbose=true})](currTrace, burnInKernel, nil)
	m.delete(burnInKernel)

	-- Run very long HMC trajectories to (hopefully) generate distant samples
	var samps = [inf.SampleVectorType(model)].stackAlloc()
	var mixKernel = [HMC({numSteps=1000, relaxManifolds=true})()]
	currTrace = [inf.mcmcSample(model, {numsamps=20, lag=20, verbose=true})](currTrace, burnInKernel, &samps)
	m.delete(mixKernel)
	m.delete(currTrace)
	return samps
end
local samps = m.gc(doInference())
local outputName = arg[1] or "output"
visualizeSamples(string.format("renders/layerExtract/%s", outputName), samps, spimg, blendMode)

-- local terra imgTest()
-- 	var img = RGBImaged.stackAlloc()
-- 	spimg:reconstructFullRes(&img)
-- 	[RGBImaged.save(uint8)](&img, image.Format.PNG, "reconstruction.png")
-- end
-- imgTest()








