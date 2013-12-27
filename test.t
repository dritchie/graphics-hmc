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

-- Terrain Height tests
local TerrainMapT = terrain.TerrainMap(real)
local HeightConstraintT = terrain.HeightConstraint(real)
local hEnergy = HeightConstraintT(0.4, 0.6,
	                                0, 100, 5,
	                                0, 100, 5)


-- Fractal Spline Model
local DoubleGrid = image.Image(double, 1)
local tgtImage = DoubleGrid.methods.load(image.Format.PNG, "targets/stanfordS_34_50.png")
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
local kernel = HMC({numSteps=1})
local terra doInference()
	return [mcmc(fractalSplineModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

-- Render movie
local moviename = arg[1] or "movie"
terrain.renderSamplesToMovie(samples, numsamps, moviename)