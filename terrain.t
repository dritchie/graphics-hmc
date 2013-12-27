-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local mem = terralib.require("mem")
local image = terralib.require("image")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local cmath = util.includec_path("math.h")

-- Returns MSE of height in image region to constant height range [hmin,hmax]
local HeightConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	return function(hmin, hmax, xmin, xmax, xstep, ymin, ymax, ystep)
		return terra(image: &RealGrid)
			var mse = real(0.0)
			var numSamples = 0
			for y = ymin, ymax, ystep do
				for x = xmin, xmax, xstep do
				  numSamples = numSamples + 1
					var h = image(x,y)(0)
					var d = 0.0
					if (h > hmax) then
						d = h - hmax
					elseif (h < hmin) then
						d = h - hmin
					end
					mse = mse + (d * d)
					--C.printf("%d,%d\t=\t%f,%f,%f\n", x, y, h, d, mse)
				end
			end
			mse = mse / numSamples;
			return mse
		end
	end
end)

-- A map object

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


local fractalSplineModel = templatize(function(real)
  local TerrainMapT = TerrainMap(real)

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
					var diff = lattice(x,y):distSq(tgtImage(x, y))
					var e_d = 0.5 * c * diff
					var energy = e_p(0) + e_d
					factor(-energy/temp)
				end
			end

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
	fractalSplineModel = fractalSplineModel,
	renderSamplesToMovie = renderSamplesToMovie,
	TEST_IMAGE = TEST_IMAGE
}