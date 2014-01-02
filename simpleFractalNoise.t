-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local rand = terralib.require("prob.random")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local DoubleGrid = image.Image(double, 1)


local size = 40
local function fractalSplineModel()

	local splineTemp = 100.0
	local imageTemp = 0.005
	local rigidity = 1.0
	local tension = 0.5
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1)


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
	local clamp = macro(function(val, lo, hi)
		return `ad.math.fmin(ad.math.fmax(val, lo), hi)
	end)


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

	-- Circle 'image' constraint
	local terra circleConstraint(lattice: &RealGrid, pos: Vec2, rad: real)
		var p = Vec2.stackAlloc(pos(0)*lattice.width, pos(1)*lattice.height)
		var r = rad*lattice.width
		var pminx = [int](ad.val(clamp(p(0) - r, 0.0, double(lattice.width))))
		var pmaxx = [int](ad.val(clamp(p(0) + r, 0.0, double(lattice.width))))
		var pminy = [int](ad.val(clamp(p(1) - r, 0.0, double(lattice.height))))
		var pmaxy = [int](ad.val(clamp(p(1) + r, 0.0, double(lattice.height))))
		-- C.printf("%d, %d, %d, %d     \n", pminx, pmaxx, pminy, pmaxy)
		var r2 = rad*rad
		var totalPenalty = real(0.0)
		var numPoints = 0
		for y=pminy,pmaxy do
			for x=pminx,pmaxx do
				var p_ = Vec2.stackAlloc(double(x)/lattice.width, double(y)/lattice.height)
				if p_:distSq(pos) < r2 then
					numPoints = numPoints + 1
					totalPenalty = totalPenalty + softEq(lattice(x,y)(0), 1.0, imageTemp)
				end
			end
		end
		factor(totalPenalty/double(numPoints))
	end

	local iters = global(int, 0)
	return terra()
		iters = iters + 1
		var lattice = RealGrid.stackAlloc(size, size)
		-- Priors
		for y=0,size do
			for x=0,size do
				-- lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				lattice(x,y)(0) = gaussian(0.25, 0.25, {structural=false, mass=1.0})
				bound(lattice(x,y)(0), 0.0, 1.0, 0.2)
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
		-- Circle constraint
		var mass = 1.0
		var circCenter = Vec2.stackAlloc(uniform(0.0, 1.0, {structural=false, hasPrior=false, mass=mass}),
										 uniform(0.0, 1.0, {structural=false, hasPrior=false, mass=mass}))
		-- bound(circCenter(0), 0.0, 1.0, 0.1)
		-- bound(circCenter(1), 0.0, 1.0, 0.1)
		-- var circCenter = Vec2.stackAlloc(0.5, 0.5)
		var circRad = 0.1
		-- C.printf("\n(%.2f, %.2f)              \n", ad.val(circCenter(0)), ad.val(circCenter(1)))
		circleConstraint(&lattice, circCenter, circRad)
		-- -- Constrain circle to be on another circle
		-- var metaCircCenter = Vec2.stackAlloc(0.5, 0.5)
		-- var metaCircRad = 0.3
		-- factor(softEq(circCenter:dist(metaCircCenter), metaCircRad, 0.05))
		return lattice
	end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=20})
-- local kernel = HMC({numSteps=40, stepSizeAdapt=false, stepSize=0.002})
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







