-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local rand = terralib.require("prob.random")
local imageConstraints = terralib.require("imageSimConstraints")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")


local size = 80
local function fractalSplineModel()

	local splineTemp = 0.25
	local imageTemp = 0.005
	local rigidity = 1.0
	local tension = 0.5
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1)

	local TransformedSimConstraint = imageConstraints.TransformedSimConstraint(real)
	local tgtImagePenaltyFn = TransformedSimConstraint(tgtImage, imageTemp)


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

	local logistic = macro(function(x)
		return `1.0 / (1.0 + ad.math.exp(-x))
	end)

	-- 'x' assumed to be between 0 and 1
	local rescale = macro(function(x, lo, hi)
		return `lo + x*(hi-lo)
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

	-- Generate unconstrained random lattice
	local genLattice = pfn(terra()
		var lattice = RealGrid.stackAlloc(size, size)
		var scale = 0.01
		-- var scale = 0.005
		-- var scale = 1.0
		for y=0,size do
			for x=0,size do
				var pix = gaussian(0.0, scale, {structural=false})
				-- var pix = uniform(-scale, scale, {structural=false, hasPrior=false})
				lattice(x,y)(0) = logistic(pix/scale)
				-- lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				-- lattice(x,y)(0) = gaussian(0.5, 0.25, {structural=false, mass=1.0})
				-- bound(lattice(x,y)(0), 0.0, 1.0, 0.2)
			end
		end
		return lattice
	end)

	-- Separate circle 'image' constraint
	local terra circleConstraint(lattice: &RealGrid, pos: Vec2, rad: real)
		var p = Vec2.stackAlloc(pos(0)*lattice.width, pos(1)*lattice.height)
		var r = rad*lattice.width
		var pminx = [int](ad.val(clamp(p(0) - r, 0.0, double(lattice.width))))
		var pmaxx = [int](ad.val(clamp(p(0) + r, 0.0, double(lattice.width))))
		var pminy = [int](ad.val(clamp(p(1) - r, 0.0, double(lattice.height))))
		var pmaxy = [int](ad.val(clamp(p(1) + r, 0.0, double(lattice.height))))
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
		if numPoints > 0 then
			totalPenalty = totalPenalty/double(numPoints)
		end
		-- C.printf("%g                   \n", ad.val(totalPenalty))
		factor(totalPenalty)
	end

	-- Variational spline smoothness constraint
	local terra splineConstraint(lattice: &RealGrid)
		var h = 1.0 / size
		var totalEnergy = real(0.0)
		for y=1,size-1 do
			for x=1,size-1 do
				var dx = Dx(lattice, x, y, h)
				var dy = Dy(lattice, x, y, h)
				var dxx = Dxx(lattice, x, y, h)
				var dyy = Dyy(lattice, x, y, h)
				var dxy = Dxy(lattice, x, y, h)
				var energy = 0.5 * rigidity *
					((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
				totalEnergy = totalEnergy + energy(0)
			end
		end
		factor(-totalEnergy/(splineTemp*(size-1)*(size-1)))
	end

	return terra()

		-- Generate lattice
		var lattice = genLattice()

		-- Varational spline derivative constraint
		-- splineConstraint(&lattice)

		-- -- Circle constraint
		-- var circRad = 0.1
		-- var scale = 0.1
		-- -- var cx = gaussian(0.0, scale, {structural=false})
		-- -- var cy = gaussian(0.0, scale, {structural=false})
		-- var cx = uniform(-scale, scale, {structural=false, hasPrior=false})
		-- var cy = uniform(-scale, scale, {structural=false, hasPrior=false})
		-- cx = logistic(cx/scale)
		-- cy = logistic(cy/scale)
		-- var circCenter = Vec2.stackAlloc(cx, cy)
		-- -- var circCenter = Vec2.stackAlloc(0.5, 0.5)
		-- circleConstraint(&lattice, circCenter, circRad)

		-- General image constraint
		var scale = 0.05
		var halfw = (tgtImage.width/2.0) / size
		var halfh = (tgtImage.height/2.0) / size
		var cx = gaussian(0.0, scale, {structural=false})
		var cy = gaussian(0.0, scale, {structural=false})
		-- var cx = uniform(-scale, scale, {structural=false, hasPrior=false})
		-- var cy = uniform(-scale, scale, {structural=false, hasPrior=false})
		cx = logistic(cx/scale)
		cy = logistic(cy/scale)
		cx = rescale(cx, halfw, 1.0-halfw)
		cy = rescale(cy, halfh, 1.0-halfh)
		var center = Vec2.stackAlloc(cx, cy)
		-- C.printf("(%g, %g)                \n", ad.val(cx), ad.val(cy))
		factor(tgtImagePenaltyFn(&lattice, ad.val(center)))
		-- factor(tgtImagePenaltyFn(&lattice, center))

		return lattice
	end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local temp = 1000.0
local kernel = HMC({numSteps=20})



local scheduleFn = macro(function(iter, currTrace)
	return quote
		currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)

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







