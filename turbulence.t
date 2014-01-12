-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local templatize = terralib.require("templatize")
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
local RGBImage = image.Image(double, 3)
local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
-- local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/cal_50_40.png")
-- local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/loop_alpha_100_100.png")
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")

-- local params = {size=512, xPeriod = 5.0, yPeriod = 10.0, xyPeriod = 12.0, turbPower = 0.1, turbSize = 32.0}
-- U.testTurbulence(params, "renders/test.png")

local function cloudModel()
  local U = logoUtils()  -- Need new U here to make sure random var tracking used on functions defined inside
  
  local imageTemp = 0.001
  local Vec2 = Vec(real, 2)
  local DeltaSimConstraint = imageConstraints.DirectSimDeltaConstraint(real)
  local deltaSim = DeltaSimConstraint(0.2)
  local TransformedSimConstraint = imageConstraints.TransformedSimConstraint(real)
  local tgtImagePenaltyFn = TransformedSimConstraint(tgtImage, imageTemp)
  local rescale = macro(function(x, lo, hi)
    return `lo + x*(hi-lo)
  end)

  local size = 128
  local params = {width=size, height=size, xPeriod=5.0, yPeriod=10.0, turbPower=5.0, turbSize=64.0}
  return terra()
    -- var lattice = [U.TurbulenceLattice(real)](params.width, params.height, maxSubdiv)
    
    var maxSubdiv = 5 -- 2^6 = 64
    var lattice = [U.TurbulenceBySubdivLattice(real)](params.width, params.height, maxSubdiv)
    
    var halfw = (tgtImage.width/2.0) / params.width
    var halfh = (tgtImage.height/2.0) / params.height
    var cx = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})
    var cy = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})
    cx = rescale(cx, halfw, 1.0-halfw)
    cy = rescale(cy, halfh, 1.0-halfh)
    var center = Vec2.stackAlloc(cx, cy)
    C.printf("(%g, %g)\n", ad.val(cx), ad.val(cy))
    factor(tgtImagePenaltyFn(&lattice, ad.val(center)))
    
    -- var err = deltaSim(&lattice, &tgtImage)
    -- var c = 100
    -- factor(- c * err)

    return lattice
  end
end

-- Do HMC inference on the model
local numsamps = 100
local verbose = true
local temp = 10000.0
local kernel = HMC({numSteps=20})
local scheduleFn = macro(function(iter, currTrace)
  return quote
    currTrace.temperature = temp
  end
end)
kernel = Schedule(kernel, scheduleFn)

local terra doInference()
  return [mcmc(cloudModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())
local moviename = arg[1] or "movie"
local gridSaver = U.GridSaver(real, U.LightnessColorizer)
U.renderSamplesToMovie(samples, moviename, gridSaver)
