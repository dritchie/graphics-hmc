-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local Vector = terralib.require("vector")
local image = terralib.require("image")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local terrain = terralib.require("terrain")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

-- Get relevant types
local TerrainMapT = terrain.TerrainMap(real)
local HeightConstraintT = terrain.HeightConstraint(real)
local hEnergy = HeightConstraintT(0.4, 0.6,
	                                0, 100, 5,
	                                0, 100, 5)

-- Target images
local DoubleGrid = image.Image(double, 1);
local testTgtImage = (terra()
  var t = DoubleGrid.stackAlloc(100, 100) --methods.load(image.Format.PNG, "targets/stanfordS_34_50.png")
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
local tgtImage = DoubleGrid.methods.load(image.Format.PNG, "targets/stanfordS_34_50.png")
-- local test = terra()
-- 	for y = 0, tgtImage.height do
-- 	 for x = 0, tgtImage.width do
-- 	 	var v = tgtImage(x, y)(0)
-- 	 	C.printf("%f\n", v)
-- 	 end
-- 	end
-- end
--test()

-- Fractal Spline Model
local function fractalSplineModel()
	local size = 100
	local temp = 0.001
	local rigidity = 1.0
	local tension = 0.5
	local c = 1000000
	local FractalSplineModelT = terrain.fractalSplineModel(real)

	return FractalSplineModelT(tgtImage, temp, rigidity, tension, c)
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 4000
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

-- local function testImg()
-- 	local size = 100
-- 	local temp = 1.0

-- 	return terra()
-- 	  var terrainMap = TerrainMapT.stackAlloc(size)
-- 		var lattice = terrainMap.grid
-- 		var xy : Vec2i
-- 		var uv : Vec2
-- 		uv(0) = 0.5; uv(1)  = 0.6
-- 		terrain.uv2xy(&lattice, &uv, &xy)
-- 		C.printf("%d,%d\n", xy(0), xy(1))
-- 		for y=0,size do
-- 			for x=0,size do
-- 				if (y > (size/2)) then
-- 					lattice(x,y)(0) = 0.5
-- 				else
-- 					lattice(x,y)(0) = 0.0
-- 				end
-- 			end
-- 		end

-- 		var h = hEnergy(&lattice)
-- 		C.printf("%f\n", h)
-- 		return lattice
-- 	end
-- end

-- io.write("Rendering...")
-- io.flush()
-- local DoubleGrid = image.Image(double, 1)
-- local terra renderFrames()
-- 	var framename : int8[1024]
-- 	var framenumber = 0
-- 	for i=0,1 do
-- 		C.sprintf(framename, "renders/movie_%06d.png", framenumber)
-- 		framenumber = framenumber + 1
-- 		-- Quantize image to 8 bits per channel when saving
-- 		var img = [testImg()]()
-- 		var imagePtr = &img
-- 		[DoubleGrid.save(uint8)](imagePtr, image.Format.PNG, framename)
-- 	end
-- end
-- renderFrames()
-- print("done.")