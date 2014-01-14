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
local imgCross = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/cross_alpha_50_50.png")

local function model()
  local U = logoUtils()  -- Need new U here to make sure random var tracking used on functions defined inside

  local size = 80
  local splineTemp = 0.25
  local tgtSoftness = 0.001
  local rigidity = 1.0
  local tension = 0.5

  return terra()
    -- Generate lattice
    var lattice = [U.RandomLattice(real)](size, size)

    -- Varational spline derivative constraint
    [U.SplineConstraint(real,
      {temp=splineTemp, rigidity=rigidity, tension=tension})](&lattice)

    -- Target image constraints
    -- var c1 = [imageConstraints.RandomPosTransformedSimConstraint(real,
    --   {target=imgS, softness=tgtSoftness})](&lattice)
    -- var c2 = [imageConstraints.RandomPosTransformedSimConstraint(real,
    --   {target=imgLoop, softness=tgtSoftness})](&lattice)
    -- var c3 = [imageConstraints.RandomPosTransformedSimConstraint(real,
    --   {target=imgLoop, softness=tgtSoftness})](&lattice)
    var c4 = [imageConstraints.RandomPosTransformedSimConstraint(real,
      {target=imgCross, softness=tgtSoftness})](&lattice)
    -- var c5 = [imageConstraints.RandomPosTransformedSimConstraint(real,
    --   {target=imgCross, softness=tgtSoftness})](&lattice)
    -- C.printf("c1=(%g, %g)\tc2=(%g, %g)\n", ad.val(c1(0)), ad.val(c1(1)),
    --                                        ad.val(c2(0)), ad.val(c2(1)))
    
    -- Push apart target centers
    -- var r = 0.3
    -- factor(U.softEq(c1:distSq(c2), r*r, 0.01))
    -- factor(U.softEq(c2:distSq(c3), r*r, 0.01))
    -- factor(U.softEq(c3:distSq(c1), r*r, 0.01))

    -- Symmetry constraint (reflection across x=width/2)
    -- [U.SymmetryConstraint(real, {temp=0.00001})](&lattice)

    return lattice
  end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local temp = 10000.0
local kernel = HMC({numSteps=20,stepSizeAdapt=false,stepSize=0.0001})  --verbosity=1
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
local gridSaver = U.GridSaver(real, U.TrivialColorizer)
U.renderSamplesToMovie(samples, moviename, gridSaver)
