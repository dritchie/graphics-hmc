-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local imSimConstraints = terralib.require("imageSimConstraints")

local Vec2d = Vec(double, 2)
local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

-- Target images
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")
-- local tgtImage = DoubleGrid.methods.load(image.Format.PNG, "targets/cal_50_40.png")

local function perlinModel()

	local size = 100
	local scale = 16
	local temp = 0.001
	local RealGrid = image.Image(real, 1)
	local Vec2Grid = image.Image(real, 2)
	local Vec2 = Vec(real, 2)
	local ConvSimConstraintT = imSimConstraints.ConvolutionalSimConstraint(real)
	local TransformedSimConstraintT = imSimConstraints.TransformedSimConstraint(real)
	local DirectSimConstraintT = imSimConstraints.DirectSimConstraint(real)

	local randGrad = pfn(terra(v: &Vec2)
		var ang = uniform(0.0, [2*math.pi], {structural=false, hasPrior=false})
		v(0) = ad.math.cos(ang)
		v(1) = ad.math.sin(ang)
	end)

	local ease = macro(function(lo, hi, x)
		return quote
			var t = 3*x*x - 2*x*x*x
		in
			(1-t)*lo + t*hi
		end
	end)

	-- -- Convolutional constraint: make the image appear as many places as possible
	-- local tgtImageEnergy = ConvSimConstraintT(tgtImage, 0, size - tgtImage.width, 4,
	-- 													0, size - tgtImage.height, 4)
	-- -- Transformed similarity constraint: make the image appear somewhere
	-- local tgtImageEnergy = TransformedSimConstraintT(tgtImage)
	-- local tgtImageEnergy = DirectSimConstraintT(tgtImage)

	return terra()
		-- Sample random gradients
		var gradients = Vec2Grid.stackAlloc(scale+1, scale+1)
		for j=0,gradients.height do
			for i=0,gradients.width do
				randGrad(&gradients(i,j))
			end
		end
		-- Generate noise image
		var image = RealGrid.stackAlloc(size, size)
		var p = Vec2d.stackAlloc()
		var p00 = Vec2d.stackAlloc()
		var p01 = Vec2d.stackAlloc()
		var p10 = Vec2d.stackAlloc()
		var p11 = Vec2d.stackAlloc()
		for j=0,image.height do
			var y = scale*[double](j)/image.height
			var y_0 = [int](y)
			var y_1 = [int](y+1)
			for i=0,image.width do
				var x = scale*[double](i)/image.width
				p(0) = x; p(1) = y
				var x_0 = [int](x)
				var x_1 = [int](x+1)
				p00(0) = x_0; p00(1) = y_0
				p01(0) = x_0; p01(1) = y_1
				p10(0) = x_1; p10(1) = y_0
				p11(0) = x_1; p11(1) = y_1
				-- Dot product displacement vectors with random gradients
				var d00 = gradients(x_0, y_0):dot(p - p00)
				var d01 = gradients(x_0, y_1):dot(p - p01)
				var d10 = gradients(x_1, y_0):dot(p - p10)
				var d11 = gradients(x_1, y_1):dot(p - p11)
				-- Bicubic interpolation
				var horiz_0 = ease(d00, d10, x - x_0)
				var horiz_1 = ease(d01, d11, x - x_0)
				var final = ease(horiz_0, horiz_1, y - y_0)
				-- Convert the [-1,1]-valued noise function into [0,1] range
				final = 0.5*(final+1)
				
				image(i,j)(0) = final
			end
		end
		
		-- Apply image constraint
		-- factor(-tgtImageEnergy(&image)/temp);
		-- var center = Vec2.stackAlloc(gaussian(0.0, 0.25, {structural=false}),
		-- 							 gaussian(0.0, 0.25, {structural=false}))
		-- var center = Vec2.stackAlloc(0.5, 0.5)
		-- factor(-tgtImageEnergy(&image, center)/temp)

		m.destruct(gradients)
		return image
	end
end

---------------------------------------------------------------------

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=1})
local terra doInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	-- return [mcmc(perlinModel, kernel, {numsamps=numsamps, verbose=verbose})]
	return [forwardSample(perlinModel, numsamps)]
end
-- Garbage collect the returned vector of samples
--    (i.e. free the memory when it is safe to do so)
local samples = m.gc(doInference())

-- Render the set of gathered samples into a movie
local moviename = arg[1] or "movie"
local moviefilename = string.format("renders/%s.mp4", moviename)
local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
io.write("Rendering video...")
io.flush()
-- We render every frame of a 1000 frame sequence. We want to linearly adjust downward
--    for longer sequences
local frameSkip = math.ceil(numsamps / 1000.0)
local terra renderFrames()
	var framename : int8[1024]
	var framenumber = 0
	for i=0,numsamps,frameSkip do
		C.sprintf(framename, movieframebasename, framenumber)
		framenumber = framenumber + 1
		-- Quantize image to 8 bits per channel when saving
		var imagePtr = &samples(i).value
		[DoubleGrid.save(uint8)](imagePtr, image.Format.PNG, framename)
	end
end
renderFrames()
util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
util.wait(string.format("rm -f %s", movieframewildcard))
print("done.")






