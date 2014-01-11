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

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

return function()

-- Interpolation functions
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

-- Generate unconstrained random lattice
local RandomLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  return pfn(terra(width: int, height: int)
    var lattice = RealGrid.stackAlloc(width, height)
    for y=0,height do
      for x=0,width do
        lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0, mass=1.0})
      end
    end
    return lattice
  end)
end)

-- Turbulence lattice (weighted sum of zoomed in windows on random lattice)
local TurbulenceLattice = templatize(function(real)
  local RealGrid = image.Image(real, 1)
  return terra(width: int, height: int, maxZoom: double)
    var R = [RandomLattice(real)](width, height)
    var Rturb = RealGrid.stackAlloc(width, height)
    var zero = R(0, 0) - R(0, 0) -- Get zero of lattice element type
    for x=0,width do
      for y=0,height do
        var val = zero
        var zoom = maxZoom
        while zoom >= 1.0 do
          var pix = bilerp(R, x / zoom, y / zoom)
          val = val + zoom * pix
          zoom = zoom / 2.0
        end
        val = 0.5 * val / maxZoom
        Rturb(x, y) = val
      end
    end
    return Rturb
  end
end)

-- Turbulence lattice by subsampling and weighted summing of random lattice
local TurbulenceSubSampledLattice = templatize(function(real)
  local RealGrid = image.Image(real, 1)
  local p = params
  return terra(width: int, height: int, maxSubsampleStep: double)
    var R = [RandomLattice(real)](width, height)
    var Rturb = RealGrid.stackAlloc(width, height)
    var zero = R(0, 0) - R(0, 0) -- Get zero of lattice element type
    var step = maxSubsampleStep
    while step >= 1.0 do
      for x=0,width,step do
        for y=0,height,step do
          var val = zero
          var pix = R(x, y) --bilerp(R, x / zoom, y / zoom)
          val = val + step * pix
          val = 0.5 * val / maxSubsampleStep
          Rturb(x, y) = val
        end
      end
      step = step / 2.0
    end
    return Rturb
  end
end)

-- Marble-like patterns by summing "sine" repetitions over domain with turbulence
local MarbleLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local p = params
  return terra()
    var T = [TurbulenceLattice(real)](p.width, p.height, p.turbSize)
    var marble = RealGrid.stackAlloc(p.width, p.height)
    var zero = T(0, 0) - T(0, 0) -- Get zero of lattice element type
    for x=0,p.width do
      for y=0,p.height do
        var xyValue = (x * p.xPeriod / p.width) + (y * p.yPeriod / p.height) + (p.turbPower * T(x, y)(0))
        marble(x, y)(0) = ad.math.fabs(ad.math.sin(xyValue * 3.14159))
      end
    end
    return marble
  end
end)

-- Wood-like patterns by summing circular "sine" repetitions over domain with turbulence
local WoodLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local p = params
  return terra()
    var width = p.width
    var height = p.height
    var turbSize = p.turbSize
    var turbPower = p.turbPower
    var T = [TurbulenceLattice(real)](width, height, turbSize)
    var wood = RealGrid.stackAlloc(width, height)
    var zero = T(0, 0) - T(0, 0) -- Get zero of lattice element type
    for x=0,width do
      for y=0,height do
        var xVal = (x - width / 2.0) / width
        var yVal = (y - height / 2.0) / height
        var dVal = ad.math.sqrt(xVal * xVal + yVal * yVal) + (turbPower * T(x, y)(0))
        wood(x, y)(0) = 0.5 * ad.math.fabs(ad.math.sin(2.0 * xyPeriod * dVal * 3.14159))
      end
    end
    return wood
  end
end)

-- Color conversion
local HSLtoRGB = templatize(function(real)
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
    var zero = real(0.0)
    if s ~= zero then
      var q = zero
      if l < 0.5 then q = l * (1 + s) else q = l + s - l * s end
      var p = 2.0 * l - q
      rgb(0) = hue2rgb(p, q, h + (1.0 / 3.0))
      rgb(1) = hue2rgb(p, q, h)
      rgb(2) = hue2rgb(p, q, h - (1.0 / 3.0))
    end
    return rgb
  end
end)

-- Swap RGB to BGR order
local rgb2bgr = macro(function(c)
  return quote
    var blue = c(2)
    c(2) = c(0)
    c(0) = blue
  in
    c
  end
end)

-- Map real to gray color of that lightness
local TrivialColorizer = templatize(function(real)
  local Color = Vec(real, 3)
  return terra(t: real)
    return Color.stackAlloc(t, t, t)
  end
end)

-- Map real to HSL lightness color ramp
local LightnessColorizer = templatize(function(real)
  local h2r = HSLtoRGB(real)
  return terra(t: real)
    var L = 0.75 + t / 4.0
    var c = h2r(0.663, 1.0, L)
    return rgb2bgr(c)
  end
end)

-- Map real to weighted RGB color
local WeightedRGBColorizer = templatize(function(real)
  local h2r = HSLtoRGB(real)
  local Color = Vec(real, 3)
  return terra(t: real)
    var t2 = 0.88 * t
    var rgb = Color.stackAlloc(t, t, t)
    rgb(2) = t2 + 0.1
    rgb(1) = t2 + 0.03
    rgb(0) = t2
    return rgb
  end
end)

local RealGridToRGBImage = templatize(function(real)
  local RealGrid = image.Image(real, 1)
  local RGBImage = image.Image(double, 3)
  local Color = Vec(double, 3)
  return terra(grid: &RealGrid, transferFun: real -> Color)
    var width = grid.width
    var height = grid.height
    var im = RGBImage.stackAlloc(width, height)
    for x = 0,width do
      for y = 0,height do
        im(x,y) = transferFun(grid(x,y)(0))
      end
    end
    return im
  end
end)

local GridSaver = templatize(function(real, Colorizer)
  local RealGrid = image.Image(real, 1)
  local RGBImage = image.Image(double, 3)
  local gridToRGB = RealGridToRGBImage(real)
  local colorTransferFun = Colorizer(real)
  return terra(grid: &RealGrid, imgFilename: rawstring)
    var im = gridToRGB(grid, colorTransferFun)
    [RGBImage.save(uint8)](&im, image.Format.PNG, imgFilename)
  end
end)

-- Render the set of gathered samples into a movie
local renderSamplesToMovie = function(samples, moviename, GridSaver)
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
      var imagePtr = &samples(i).value
      GridSaver(imagePtr, framename)
      framenumber = framenumber + 1
    end
  end
  renderFrames()
  util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
  util.wait(string.format("rm -f %s", movieframewildcard))
  print("done.")
end

local testTurbulence = function(params, imgFilename)
  local RGBImage = image.Image(double, 3)
  local terra test()
    var lattice = [TurbulenceLattice(real)](params.size, params.size, params.turbSize)
    [GridSaver(real, LightnessColorizer)](&lattice, imgFilename)
  end
  test()
end

local testMarble = function(params, imgFilename)
  local p = params
  local RGBImage = image.Image(double, 3)
  local terra test()
    var lattice = [MarbleLattice(real)](p.size, p.size, p.xPeriod, p.yPeriod, p.turbPower, p.turbSize)
    [GridSaver(real, WeightedRGBColorizer)](&lattice, imgFilename)
  end
  test()
end

local testWood = function(params, imgFilename)
  local p = params
  local RGBImage = image.Image(double, 3)
  local terra test()
    var lattice = [WoodLattice(real)](p.size, p.size, p.xyPeriod, p.turbPower, p.turbSize)
    [GridSaver(real, WeightedRGBColorizer)](&lattice, imgFilename)
  end
  test()
end

return {
  lerp = lerp,
  ease = ease,
  bilerp = bilerp,
  RandomLattice = RandomLattice,
  TurbulenceLattice = TurbulenceLattice,
  TurbulenceSubSampledLattice = TurbulenceSubSampledLattice,
  MarbleLattice = MarbleLattice,
  WoodLattice = WoodLattice,
  HSLtoRGB = HSLtoRGB,
  TrivialColorizer = TrivialColorizer,
  LightnessColorizer = LightnessColorizer,
  WeightedRGBColorizer = WeightedRGBColorizer,
  RealGridToRGBImage = RealGridToRGBImage,
  GridSaver = GridSaver,
  renderSamplesToMovie = renderSamplesToMovie,
  testTurbulence = testTurbulence,
  testMarble = testMarble,
  testWood = testWood
}

end