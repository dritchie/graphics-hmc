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
local U = terralib.require("terrainUtils")

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

local function testLineModel()
  local TerrainMapT = terrain.TerrainMap(real)
  local terrainMap = `TerrainMapT.stackAlloc(100, 100)
  local r = 1.0
  local lattice = `terrainMap.grid
  local function checkFun(lat)
    return terra(x: int, y: int)
      --C.printf("%f", lat)
      lat(x,y)(0) = 1.0
      return true
    end
  end
  return terra()
    var scale = 0.01
    for y=0,lattice.height do
      for x=0,lattice.width do
        var pix = gaussian(0.0, scale, {structural=false})
        lattice(x,y)(0) = U.logistic(pix/scale)
      end
    end

    var check = [checkFun(lattice)]
    U.bresenhamCheck(0, 0, 50, 50, check)
    return lattice
  end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=1})
local terra doInference()
	return [mcmc(testLineModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

-- Render movie
local moviename = arg[1] or "movie"
U.renderSamplesToMovie(samples, moviename)