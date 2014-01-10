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
local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")

-- local params = {size=512, xPeriod = 5.0, yPeriod = 10.0, xyPeriod = 12.0, turbPower = 0.1, turbSize = 32.0}
-- U.testTurbulence(params, "renders/test.png")

local function cloudModel()
  local size = 100
  local turbSize = 50 
  local U = logoUtils()  -- Need new U here to make sure random var tracking used on functions defined inside
  
  local imageTemp = 0.05
  local Vec2 = Vec(real, 2)
  local TransformedSimConstraint = imageConstraints.TransformedSimConstraint(real)
  local tgtImagePenaltyFn = TransformedSimConstraint(tgtImage, imageTemp)
  
  local logistic = macro(function(x)
    return `1.0 / (1.0 + ad.math.exp(-x))
  end)

  -- 'x' assumed to be between 0 and 1
  local rescale = macro(function(x, lo, hi)
    return `lo + x*(hi-lo)
  end)
  
  return terra()
    var lattice = [U.TurbulenceLattice(real)](size, size, turbSize)

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

    return lattice
  end
end

-- Do HMC inference on the model
local numsamps = 100
local verbose = true
local temp = 10.0
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
U.renderSamplesToMovie(samples, moviename, DoubleGrid)
