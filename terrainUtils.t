terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand =terralib.require("prob.random")
local image = terralib.require("image")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local U = {}

-- TODO: Relies on properly defined real at this point
-- This utility collection should be wrapped up in a templatize()
local Vec2 = Vec(real, 2)
local Vec2d = Vec(double, 2)

local RealGrid = image.Image(real, 1)
local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)

U.RealGrid = RealGrid
U.Vec2 = Vec2

U.polar2rect = terra(polarVec: Vec2)
  var r = polarVec(0)
  var theta = polarVec(1)
  return Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
end

-- Discrete derivatives
U.Dx = macro(function(f, x, y, h)
  return `(f(x+1,y) - f(x-1,y))/(2*h)
end)
U.Dy = macro(function(f, x, y, h)
  return `(f(x,y+1) - f(x,y-1))/(2*h)
end)
U.Dxx = macro(function(f, x, y, h)
  return `(f(x+1,y) - 2.0*f(x,y) + f(x-1,y))/(2*h*h)
end)
U.Dyy = macro(function(f, x, y, h)
  return `(f(x,y+1) - 2.0*f(x,y) + f(x,y-1))/(2*h*h)
end)
U.Dxy = macro(function(f, x, y, h)
  return `(f(x+1,y+1) - f(x+1,y) - f(x,y+1) + f(x,y))/(h*h)
end)

--------------------------------------------

U.softEq = macro(function(x, target, softness)
  return `[rand.gaussian_logprob(real)](x, target, softness)
end)

U.upperBound = macro(function(val, bound, softness)
  return quote
    if val > bound then
      factor(softEq(val-bound, 0.0, softness))
    end 
  end
end)

U.lowerBound = macro(function(val, bound, softness)
  return quote
    if val < bound then
      factor(softEq(bound-val, 0.0, softness))
    end 
  end
end)

U.bound = macro(function(val, lo, hi, softness)
  return quote
    lowerBound(val, lo, softness)
    upperBound(val, hi, softness)
  end
end)

U.lowerClamp = macro(function(val, lo)
  return `ad.math.fmax(val, lo)
end)

U.upperClamp = macro(function(val, hi)
  return `ad.math.fmin(val, hi)
end)

U.clamp = macro(function(val, lo, hi)
  return `lowerClamp(upperClamp(val, hi), lo)
end)

--------------------------------------------

U.ngaussian = macro(function(m, sd)
  return `gaussian(m, sd, {structural=false})
end)
U.nuniformNoPrior = macro(function(lo, hi)
  return `uniform(lo, hi, {structural=false, hasPrior=false})
end)

U.logistic = macro(function(x)
  return `1.0 / (1.0 + ad.math.exp(-x))
end)

U.lerp = macro(function(lo, hi, t)
  return `(1.0-t)*lo + t*hi
end)

U.ease = macro(function(lo, hi, x)
  return quote
    var t = 3*x*x - 2*x*x*x
  in
    (1-t)*lo + t*hi
  end
end)

-- x and y may not be ints, and may be dual nums
U.bilerp = macro(function(image, x, y, Vec2)
  return quote
    var x0 = [int](ad.math.floor(ad.val(x)))
    var x1 = [int](ad.math.ceil(ad.val(x)))
    var y0 = [int](ad.math.floor(ad.val(y)))
    var y1 = [int](ad.math.ceil(ad.val(y)))
    var tx = x - [double](x0)
    var ty = y - [double](y0)
    var v00 = Vec2(image(x0, y0))
    var v01 = Vec2(image(x0, y1))
    var v10 = Vec2(image(x1, y0))
    var v11 = Vec2(image(x1, y1))
    -- var v0 = lerp(v00, v01, ty)
    -- var v1 = lerp(v10, v11, ty)
    var v0 = ease(v00, v01, ty)
    var v1 = ease(v10, v11, ty)
  in
    -- lerp(v0, v1, tx)
    ease(v0, v1, tx)
  end
end)

local softmaxTightness = 40.0
U.softmax = macro(function(x, y)
  local t = softmaxTightness
  local invt = 1.0/t
  return `ad.math.pow(ad.math.pow(x, t) + ad.math.pow(y, t), invt)
end)

U.bresenhamAccumulate = function(x0, y0, x1, y1, accumFun)
  return terra()
    var sx = 1
    if x0 > x1 then sx = -1 end
    var sy = 1
    if y0 > y1 then sy = -1 end
    var dx = x1 - x0
    if dx < 0 then dx = -dx end
    var dy = y1 - y0
    if dy < 0 then dy = -dy end

    var err = dx - dy
    var err2 = dx - dx
    var accum = accumFun(x0, y0)
    
    while not(x0 == x1 and y0 == y1) do
      err2 = err + err
      if err2 > -dy then
        err = err - dy
        x0  = x0 + sx
      end
      if err2 < dx then
        err = err + dx
        y0  = y0 + sy
      end
      accum = accum + accumFun(x0, y0)
    end
    return accum
  end
end

U.bresenhamLineAccumulate = function(x0, y0, x1, y1, latt)
  local points = {}
  local count = 0
  local err = 0.0
  local result = U.bresenhamAccumulate(x0, y0, x1, y1, function(x,y)
    local herr = `ad.math.fabs(latt(x, y)(0) - 1.0)
    count = count + 1
    points[count] = {x,y}
    return herr
  end)
  return points, result
end

U.TEST_IMAGE = (terra()
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
U.renderSamplesToMovie = function(samples, moviename)
  local moviefilename = string.format("renders/%s.mp4", moviename)
  local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
  local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
  io.write("Rendering video...")
  io.flush()
  local numsamps = samples.size
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

return U