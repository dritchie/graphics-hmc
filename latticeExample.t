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


-- This is continuous lattice model that looks kind of like an Ising model.
-- Every variable is uniform in the range [0,1], but there are constraints that
--    encourage neighboring variables to take on the same value
-- The variables are marked as non-structural because their value does not affect
--    the control flow of the program.
-- 'temp' controls how tight the constraints are.
-- 'RealGrid' must be defined inside the latticeModel() function, as the definition
--    of type 'real' can change (it may be double, or it may be a special autodiff number type)
local function latticeModel()
	local size = 100
	local temp = 0.1
	local RealGrid = image.Image(real, 1)
	return terra()
		var lattice = RealGrid.stackAlloc(100, 100)
		-- Priors
		for y=0,size do
			for x=0,size do
				lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				-- lattice(x,y)(0) = gaussian(0.5, 0.2, {structural=false})
			end
		end
		-- Constraints
		for y=0,size do
			for x=0,size do
				var diff = real(0.0)
				if x < size-1 then
					var diff1 = lattice(x,y)(0) - lattice(x+1,y)(0)
					diff = diff + diff1*diff1
				end
				if y < size-1 then
					var diff2 = lattice(x,y)(0) - lattice(x,y+1)(0)
					diff = diff + diff2*diff2
				end
				if x > 0 then
					var diff3 = lattice(x-1,y)(0) - lattice(x,y)(0)
					diff = diff + diff3*diff3
				end
				if y > 0 then
					var diff4 = lattice(x,y-1)(0) - lattice(x,y)(0)
					diff = diff + diff4*diff4
				end
				factor(-diff/temp)
			end
		end
		return lattice
	end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=1})	-- Defaults to trajectories of length 1
local terra doInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	return [mcmc(latticeModel, kernel, {numsamps=numsamps, verbose=verbose})]
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






