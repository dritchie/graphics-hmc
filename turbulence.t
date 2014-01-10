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

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(double, 3)

local DoubleGrid = image.Image(double, 1)
local DoubleAlphaGrid = image.Image(double, 2)
local tgtImage = DoubleAlphaGrid.methods.load(image.Format.PNG, "targets/stanfordS_alpha_34_50.png")

-- Interpolation
local lerp = macro(function(lo, hi, t)
  return `(1.0-t)*lo + t*hi
end)
local ease = macro(function(lo, hi, x)
  return quote
    var t = 3*x*x - 2*x*x*x
  in
    (1-t)*lo + t*hi
  end
end)
-- x and y may not be ints, and may be dual nums
local bilerp = macro(function(image, x, y)
  return quote
    var x0 = [int](ad.math.floor(ad.val(x)))
    var x1 = [int](ad.math.ceil(ad.val(x)))
    var y0 = [int](ad.math.floor(ad.val(y)))
    var y1 = [int](ad.math.ceil(ad.val(y)))
    var tx = x - [double](x0)
    var ty = y - [double](y0)
    var v00 = image(x0, y0)
    var v01 = image(x0, y1)
    var v10 = image(x1, y0)
    var v11 = image(x1, y1)
    -- var v0 = lerp(v00, v01, ty)
    -- var v1 = lerp(v10, v11, ty)
    var v0 = ease(v00, v01, ty)
    var v1 = ease(v10, v11, ty)
  in
    -- lerp(v0, v1, tx)
    ease(v0, v1, tx)
  end
end)

-- Turbulence
local turbulence = macro(function(randLattice, x, y, maxZoom)
  return quote
    var val = randLattice(x, y) - randLattice(x, y) -- Get zero of image element type
    var zoom = maxZoom
    while zoom >= 1.0 do
      var pix = bilerp(randLattice, x / zoom, y / zoom)
      val = val + pix * zoom
      zoom = zoom / 2.0
    end
    val = 0.5 * val / maxZoom
  in
    val
  end
end)

-- Color conversion
local hsl2rgb = templatize(function(real)
  local Color = Vec(real, 3)
  
  local terra hue2rgb(p: real, q: real, t: real)
    if t < 0.0 then t = t + 1.0 end
    if t > 1.0 then t = t - 1.0 end
    if t < (1.0 / 6.0) then return p + (q - p) * 6.0 * t end
    if t < (1.0 / 2.0) then return q end
    if t < (2.0 / 3.0) then return p + (q - p) * (2.0 / 3.0 - t) * 6.0 end
    return p
  end
  
  return terra(h: real, s: real, l: real)
    var rgb = Color.stackAlloc(l, l, l)  -- default for achromatic case (s == 0)
    if s ~= 0.0 then
      var q = real(0.0)
      if l < 0.5 then q = l * (1 + s) else q = l + s - l * s end
      var p = 2.0 * l - q
      rgb(0) = hue2rgb(p, q, h + (1.0 / 3.0))
      rgb(1) = hue2rgb(p, q, h)
      rgb(2) = hue2rgb(p, q, h - (1.0 / 3.0))
    end
    return rgb
  end
end)

local terra testTurbulence()
  var size = 256
  var testImgFilename = "renders/test.png"
  
  -- Generate random lattice
  var lattice = DoubleGrid.stackAlloc(size, size)
  for y=0,size do
    for x=0,size do
      lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false, lowerBound=0.0, upperBound=1.0})
    end
  end

  -- Turbulence pass
  var lattTurb = DoubleGrid.stackAlloc(size, size)
  for x = 0, size do
    for y = 0, size do
      var t = turbulence(lattice, x, y, 128.0)
      lattTurb(x, y) = t
    end
  end

  -- Create RGB image
  var im = RGBImage.stackAlloc(size, size)
  var h2r = [hsl2rgb(double)]
  for x = 0,size do
    for y = 0,size do
      var t = lattTurb(x, y)(0)
      var L = 0.75 + t / 4.0
      var c = h2r(0.663, 1.0, L)
      
      -- Swap into BGR order
      var blue = c(2)
      c(2) = c(0)
      c(0) = blue
      
      -- C.printf("%f,%f,%f\n", c(0), c(1), c(2))
      im(x, y) = c
    end
  end
 
  -- Output frame
  [RGBImage.save(uint8)](&im, image.Format.PNG, testImgFilename)
  -- [DoubleGrid.save(uint8)](imagePtr, image.Format.PNG, framename)
end
-- testTurbulence()

-- Marble-like patterns by passing "sine" repetitions over domain through turbulence
local marble = macro(function(randLattice, x, y, xPeriod, yPeriod, turbPower, turbSize)
  return quote
    var height = randLattice.height
    var width = randLattice.width
    var xyValue = x * xPeriod / width + y * yPeriod / height + turbPower * turbulence(randLattice, x, y, turbSize)(0)
    var sineValue = ad.math.fabs(ad.math.sin(xyValue * 3.14159))
  in
    sineValue
  end
end)

-- Wood-like pattern passing circular "sines" over turbulence
local wood = macro(function(randLattice, x, y, xyPeriod, turbPower, turbSize)
  return quote
    var height = randLattice.height
    var width = randLattice.width
    var xVal = (x - width / 2.0) / width
    var yVal = (y - height / 2.0) / height
    var distVal = ad.math.sqrt(xVal * xVal + yVal * yVal) + turbPower * turbulence(randLattice, x, y, turbSize)(0)
    var sineVal = 0.5 * ad.math.fabs(ad.math.sin(2.0 * xyPeriod * distVal * 3.14159))
  in
    sineVal
  end
end)

local terra testMarble()
  var size = 256
  var testImgFilename = "renders/test.png"

  -- -- Marble
  -- var xPeriod = 5.0
  -- var yPeriod = 10.0
  -- var turbPower = 2.0
  -- var turbSize = 32.0

  -- Wood
  var xyPeriod = 12.0
  var turbPower = 0.1
  var turbSize = 32.0
  
  -- Generate random lattice
  var lattice = DoubleGrid.stackAlloc(size, size)
  for y=0,size do
    for x=0,size do
      lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false, lowerBound=0.0, upperBound=1.0})
    end
  end

  -- Create RGB image
  var im = RGBImage.stackAlloc(size, size)
  var h2r = [hsl2rgb(double)]
  for x = 0,size do
    for y = 0,size do
      --var t = marble(lattice, x, y, xPeriod, yPeriod, turbPower, turbSize)
      var t = wood(lattice, x, y, xyPeriod, turbPower, turbSize)
      im(x,y)(2) = 0.1 + t
      im(x,y)(1) = 0.03 + t
      im(x,y)(0) = 0.88 * t
    end
  end
  [RGBImage.save(uint8)](&im, image.Format.PNG, testImgFilename)
end
testMarble()


-- util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
-- util.wait(string.format("rm -f %s", movieframewildcard))
print("done.")
