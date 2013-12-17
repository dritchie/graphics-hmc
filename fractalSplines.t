-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]


local function fractalSplineModel()

	local size = 100
	local temp = 1.0
	local rigidity = 1.0
	local tension = 0.5
	local RealGrid = image.Image(real, 1)

	-- Discrete derivatives
	local Dx = macro(function(f, x, y, h)
		return `(@f(x+1,y) - @f(x-1,y))/(2*h)
	end)
	local Dy = macro(function(f, x, y, h)
		return `(@f(x,y+1) - @f(x,y-1))/(2*h)
	end)
	local Dxx = macro(function(f, x, y, h)
		return `(@f(x+1,y) - 2.0*@f(x,y) + @f(x-1,y))/(2*h*h)
	end)
	local Dyy = macro(function(f, x, y, h)
		return `(@f(x,y+1) - 2.0*@f(x,y) + @f(x,y-1))/(2*h*h)
	end)
	local Dxy = macro(function(f, x, y, h)
		return `(@f(x+1,y+1) - @f(x+1,y) - @f(x,y+1) + @f(x,y))/(h*h)
	end)

	return terra()
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
				factor(-energy(0)/temp)
			end
		end
		return lattice
	end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 2000
local verbose = true
local kernel = HMC({numSteps=1})	-- Defaults to trajectories of length 1
local terra doInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	return [mcmc(fractalSplineModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
-- Garbage collect the returned vector of samples
--    (i.e. free the memory when it is safe to do so)
local samples = m.gc(doInference())

-- Render the set of gathered samples into a movie
io.write("Rendering video...")
io.flush()
-- We render every frame of a 1000 frame sequence. We want to linearly adjust downward
--    for longer sequences
local frameSkip = math.ceil(numsamps / 1000.0)
local DoubleGrid = image.Image(double, 1)
local terra renderFrames()
	var framename : int8[1024]
	var framenumber = 0
	for i=0,numsamps,frameSkip do
		C.sprintf(framename, "renders/movie_%06d.png", framenumber)
		framenumber = framenumber + 1
		-- Quantize image to 8 bits per channel when saving
		var imagePtr = &samples(i).value
		[DoubleGrid.save(uint8)](imagePtr, image.Format.PNG, framename)
	end
end
renderFrames()
util.wait("ffmpeg -threads 0 -y -r 30 -i renders/movie_%06d.png -c:v libx264 -r 30 -pix_fmt yuv420p renders/movie.mp4 2>&1")
util.wait("rm -f renders/movie_*.png")
print("done.")






