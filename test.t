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

-- Fractal Spline Model
local DoubleGrid = image.Image(double, 1)
local tgtImage = DoubleGrid.methods.load(image.Format.PNG, "targets/stanfordS_100_148.png")
local function fractalSplineModel()
	local temp = 0.001
	local rigidity = 0.2
	local tension = 0.5
	local c = 1000000.05
	local FractalSplineModelT = terrain.fractalSplineModel(real)
  local TerrainMapT = terrain.TerrainMap(real)
  local terrainMap = `TerrainMapT.stackAlloc(100, 100)
	return FractalSplineModelT(tgtImage, temp, rigidity, tension, c)
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=1})
local terra doInference()
	return [mcmc(fractalSplineModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

-- Render movie
local moviename = arg[1] or "movie"
terrain.renderSamplesToMovie(samples, numsamps, moviename)