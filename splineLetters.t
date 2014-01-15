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
local logoUtils = terralib.require("logoUtils")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local U = logoUtils()
local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
local imgS = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")
local imgLoop = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/loop_alpha_50_50.png")
local imgEquals = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/equals_50_50.png")
local imgCross2 = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/cross2_alpha_50_50.png")
local imgHojo = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/triforce_alpha_25_25.png")

local function model()
  local U = logoUtils()  -- Need new U here to make sure random var tracking used on functions defined inside
  local Vec2 = Vec(real, 2)

  local size = 100
  local splineTemp = 0.25
  local distTemp = 100.0
  local tgtSoftness = 0.001
  local rigidity = 1.0
  local tension = 0.5

  local TransformedSimConstraint = imageConstraints.TransformedSimConstraint(real)
  local centerHojoPenaltyFn = TransformedSimConstraint(imgHojo, tgtSoftness)
  return terra()
    -- Generate lattice
    var lattice = [U.RandomLattice(real, {width=size, height=size, scale=0.005})]()

    -- Varational spline derivative constraint
    [U.SplineConstraint(real,
      {temp=splineTemp, rigidity=rigidity, tension=tension})](&lattice)

    -- Target image constraints
    var c0 = Vec2.stackAlloc(0.5, 0.5)
    factor(centerHojoPenaltyFn(&lattice, ad.val(c0)))
    var c1 = [imageConstraints.RandomPosTransformedSimConstraint(real,
      {target=imgHojo, softness=tgtSoftness, scale=0.005})](&lattice)
    -- C.printf("c1=(%g, %g)\tc2=(%g, %g)\n", ad.val(c1(0)), ad.val(c1(1)),
    --                                        ad.val(c2(0)), ad.val(c2(1)))
    
    -- Encourage targets to not overlap
    -- factor(c0:distSq(c1)/distTemp)

    -- -- Symmetry constraint (reflection across x=width/2)
    -- [U.SymmetryConstraint(real, {temp=0.0001})](&lattice)

    -- -- -- Take product with Marble texture
    var marble = [U.MarbleLattice(real,
      {width=size, height=size, xPeriod=5.0, yPeriod=10.0, turbPower=1.0, turbSize=2.0})]()
    for y=0,size do
      for x=0,size do
        lattice(x, y)(0) = 0.5 * (marble(x,y)(0) + lattice(x, y)(0))
      end
    end
    
    return lattice
  end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local temp = 5000.0
local kernel = HMC({numSteps=20})--stepSizeAdapt=false,stepSize=0.0001})  --verbosity=1
local scheduleFn = macro(function(iter, currTrace)
  return quote
    currTrace.temperature = temp
  end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
  return [mcmc(model, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

-- Render the set of gathered samples into a movie
local moviename = arg[1] or "movie"
local gridSaver = U.GridSaver(real, U.WeightedRGBColorizer)
U.renderSamplesToMovie(samples, moviename, gridSaver)
