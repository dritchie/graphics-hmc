-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local imSimConstraints = terralib.require("imageSimConstraints")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")
-- local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/circle_20.png")


local numsamps = 200


local function fractalSplineModel()

	local size = 100
	-- local splineTemp = 10.0
	local splineTemp = 50000.0
	local imageSimTemp = 0.0001
	-- local imageSimTemp = 0.00000001
	local rigidity = 1.0
	local tension = 0.5
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1)
	local DirectSimConstraint = imSimConstraints.DirectSimConstraint(real)
	local TransformedSimConstraint = imSimConstraints.TransformedSimConstraint(real)
	local ConvolutionalSimConstraint = imSimConstraints.ConvolutionalSimConstraint(real)

	-- local tgtImageEnergyFn = DirectSimConstraint(tgtImage)
	-- local tgtImageEnergyFn = TransformedSimConstraint(tgtImage)
	local tgtImageEnergyFn = ConvolutionalSimConstraint(tgtImage,
														0, size - tgtImage.width, 4,
														0, size - tgtImage.height, 4)

	-- Discrete derivatives
	local Dx = macro(function(f, x, y, h)
		return `(f(x+1,y) - f(x-1,y))/(2*h)
	end)
	local Dy = macro(function(f, x, y, h)
		return `(f(x,y+1) - f(x,y-1))/(2*h)
	end)
	local Dxx = macro(function(f, x, y, h)
		return `(f(x+1,y) - 2.0*f(x,y) + f(x-1,y))/(2*h*h)
	end)
	local Dyy = macro(function(f, x, y, h)
		return `(f(x,y+1) - 2.0*f(x,y) + f(x,y-1))/(2*h*h)
	end)
	local Dxy = macro(function(f, x, y, h)
		return `(f(x+1,y+1) - f(x+1,y) - f(x,y+1) + f(x,y))/(h*h)
	end)

	local iters = global(int, 0)
	return terra()
		iters = iters + 1
		var lattice = RealGrid.stackAlloc(100, 100)
		-- Priors
		for y=0,size do
			for x=0,size do
				lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
			end
		end
		-- Varational spline derivative constraints
		var h = 1.0 / size
		for y=1,size-1 do
			for x=1,size-1 do
				var dx = Dx(lattice, x, y, h)
				var dy = Dy(lattice, x, y, h)
				var dxx = Dxx(lattice, x, y, h)
				var dyy = Dyy(lattice, x, y, h)
				var dxy = Dxy(lattice, x, y, h)
				var energy = 0.5 * rigidity *
					((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
				factor(-energy(0)/splineTemp)
			end
		end
		-- Image target constraint
		factor(-tgtImageEnergyFn(&lattice)/imageSimTemp)
		-- var center = Vec2.stackAlloc(0.5, 0.5)
		-- var mass = 1.0
		-- if iters < numsamps/2 then mass = 100000.0 end
		-- var cx = gaussian(0.5, 0.25, {structural=false, mass=mass})
		-- var cy = gaussian(0.5, 0.25, {structural=false, mass=mass})
		-- var center = Vec2.stackAlloc(cx, cy)
		-- var circCenter = Vec2.stackAlloc(0.5, 0.5)
		-- var circRadius = 0.45
		-- var err = circCenter:dist(center) - circRadius
		-- err = err*err
		-- factor(-err/imageSimTemp)
		-- C.printf("center: (%g, %g)           \n", ad.val(center(0)), ad.val(center(1)))
		-- factor(-tgtImageEnergyFn(&lattice, center)/imageSimTemp)
		return lattice
	end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local verbose = true
-- local kernel = HMC({numSteps=1})	-- Defaults to trajectories of length 1
local kernel = HMC({numSteps=20, targetAcceptRate=0.65})
local terra doInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	return [mcmc(fractalSplineModel, kernel, {numsamps=numsamps, verbose=verbose})]
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







