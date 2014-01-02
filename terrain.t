-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local mem = terralib.require("mem")
local image = terralib.require("image")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local cmath = util.includec_path("math.h")

-- Returns total difference between heights in image region
-- and constant height range [hmin,hmax] over [xmin,xmax] x [ymin,ymax]
local HeightConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	return function(hmin, hmax, xmin, xmax, ymin, ymax)
		return terra(image: &RealGrid)
			var err = real(0.0)
			for y = ymin, ymax do
				for x = xmin, xmax do
					var h = image(x,y)(0)
					var d = h - h  --gets type right but TODO: properly initialize to 0
					if (h > hmax) then
						d = h - hmax
					elseif (h < hmin) then
						d = h - hmin
					end
					err = err + (d * d)
				end
			end
			return err
		end
	end
end)

-- Returns total difference between height in image and tgtImage
-- with a +/- hDelta threshold
local DeltaHeightConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	local DoubleGrid = image.Image(double, 1)
	return function(hDelta, xmin, xmax, ymin, ymax)
		return terra(image: &RealGrid, tgtImage: &DoubleGrid)
			var err = real(0.0)
			for y = ymin, ymax do
				for x = xmin, xmax do
					var tH = tgtImage(x, y)(0)
					var currH = image(x,y)(0)
					var hmax = tH + hDelta
					var hmin = tH - hDelta
					var d = currH - currH --gets type right but TODO: properly initialize to 0
					if (currH > hmax) then
						d = currH - hmax
					elseif (currH < hmin) then
						d = currH - hmin
					end
					err = err + (d * d)
				end
			end
			return err
		end
	end
end)

-- Container struct for terrain map
local TerrainMap = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	local Vec2 = Vec(real, 2)
	local Vec2i = Vec(int, 2)
  
  local struct MapT
  {
		grid : RealGrid
  }

  terra MapT:__construct(w: uint, h: uint)
  	self.grid = RealGrid.stackAlloc(w, h)
  end

  -- Return value at uv coordinates
  terra MapT:getPixelByUV(u: real, v: real)
  	var x = [int](cmath.round(u * self.grid.width))
		var y = [int](cmath.round(v * self.grid.height))
		return self.grid(x, y)
  end
  
  mem.addConstructors(MapT)
  return MapT
end)


local fractalSplineModel = templatize(function(real)
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

  local TerrainMapT = TerrainMap(real)
	local HeightConstraintT = HeightConstraint(real)
	local DeltaHeightConstraintT = DeltaHeightConstraint(real)
	local hE1 = HeightConstraintT(0.1, 0.3, 0, 15, 0, 15)
  local hE2 = HeightConstraintT(0.8, 1.0, 20, 30, 20, 35)
  local dHE = DeltaHeightConstraintT(0.2, 0, 100, 0, 70)

  return function(tgtImage, temp, rigidity, tension, c)
		return terra()
		  var width = tgtImage.width
		  var height = tgtImage.height 
			var terrainMap = TerrainMapT.stackAlloc(width, height)
			var lattice = terrainMap.grid

			-- Priors
			for y=0,height do
				for x=0,width do
					lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				end
			end

			-- Variational spline derivative constraints
			var h = 1.0 / cmath.fmax(width, height)
			for y = 1, height-1 do
				for x = 1, width-1 do
					var dx = Dx(lattice, x, y, h)
					var dy = Dy(lattice, x, y, h)
					var dxx = Dxx(lattice, x, y, h)
					var dyy = Dyy(lattice, x, y, h)
					var dxy = Dxy(lattice, x, y, h)
					var e_p = 0.5 * rigidity *
						((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
					--var diff = lattice(x,y):distSq(tgtImage(x, y))
					--var e_d = 0.5 * c * diff
					var energy = e_p(0) --+ e_d
					factor(-energy / temp)
				end
			end
			-- var hE = hEnergy(&lattice, c, temp)
			-- var hE2 = hEnergy2(&lattice, c, temp)
			-- factor(-hE/temp)
			-- factor(-hE2/temp)
			factor(-0.5 * c * dHE(&lattice, &tgtImage) / temp)
			return lattice
		end
	end
end)

local DoubleGrid = image.Image(double, 1);
local TEST_IMAGE = (terra()
  var t = DoubleGrid.stackAlloc(100, 100)
	for y=0,100 do
		for x=0,100 do
			if (x+y) > 100 then
				t(x, y)(0) = 1.0
			else
				t(x, y)(0) = 0.0
			end
		end
	end
	return t
end)()

-- Render the set of gathered samples into a movie
-- TODO: Factor out to utils
local function renderSamplesToMovie(samples, numsamps, moviename)
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
end

return
{
	TerrainMap = TerrainMap,
	HeightConstraint = HeightConstraint,
	DeltaHeightConstraint = DeltaHeightConstraint,
	fractalSplineModel = fractalSplineModel,
	renderSamplesToMovie = renderSamplesToMovie,
	TEST_IMAGE = TEST_IMAGE
}