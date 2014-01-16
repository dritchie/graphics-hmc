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
local Vector = terralib.require("vector")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

return function()

local logistic = macro(function(x)
  return `1.0 / (1.0 + ad.math.exp(-x))
end)

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
local bilerp = macro(function(image, x, y, VecT)
  return quote
    -- C.printf("step=%f,w=%d,h=%d\n",step,width,height)
    var x0 = [int](ad.math.floor(ad.val(x)))
    var x1 = [int](ad.math.ceil(ad.val(x)))
    var y0 = [int](ad.math.floor(ad.val(y)))
    var y1 = [int](ad.math.ceil(ad.val(y)))
    -- C.printf("(%d,%d),(%d,%d)\n",x0,y0,x1,y1)
    var tx = x - [double](x0)
    var ty = y - [double](y0)
    var v00 = VecT(image(x0, y0))
    var v01 = VecT(image(x0, y1))
    var v10 = VecT(image(x1, y0))
    var v11 = VecT(image(x1, y1))
    -- var v0 = lerp(v00, v01, ty)
    -- var v1 = lerp(v10, v11, ty)
    var v0 = ease(v00, v01, ty)
    var v1 = ease(v10, v11, ty)
  in
    -- lerp(v0, v1, tx)
    ease(v0, v1, tx)
  end
end)

local softEq = macro(function(x, target, softness)
  return `[rand.gaussian_logprob(real)](x, target, softness)
end)

-- Variational spline smoothness constraint
local SplineConstraint = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local temp = params.temp
  local rigidity = params.rigidity or 1.0
  local tension = params.tension or 0.5

  -- Discrete derivatives
  local Dx = macro(function(f, x, y, h)
    return `(f(x+1,y) - f(x-1,y))/(2*h)
  end)
  local Dy = macro(function(f, x, y, h)
    return `(f(x,y+1) - f(x,y-1))/(2*h)
  end)
  local Dxx = macro(function(f, x, y, h)
    return `(f(x+1,y) - 2.0*f(x,y) + f(x-1,y))/(2*h*h)
  end)
  local Dyy = macro(function(f, x, y, h)
    return `(f(x,y+1) - 2.0*f(x,y) + f(x,y-1))/(2*h*h)
  end)
  local Dxy = macro(function(f, x, y, h)
    return `(f(x+1,y+1) - f(x+1,y) - f(x,y+1) + f(x,y))/(h*h)
  end)

  return pfn(terra(lattice: &RealGrid)
    var h = 1.0 / lattice.width
    var totalEnergy = real(0.0)
    for y=1,lattice.height-1 do
      for x=1,lattice.width-1 do
        var dx = Dx(lattice, x, y, h)
        var dy = Dy(lattice, x, y, h)
        var dxx = Dxx(lattice, x, y, h)
        var dyy = Dyy(lattice, x, y, h)
        var dxy = Dxy(lattice, x, y, h)
        var energy = 0.5 * rigidity *
          ((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
        totalEnergy = totalEnergy + energy(0)
      end
    end
    factor(-totalEnergy/(temp*(lattice.width-1)*(lattice.height-1)))
  end)
end)

-- Image symmetry constraint
local SymmetryConstraint = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local temp = params.temp
  return pfn(terra(lattice: &RealGrid)
    var width: uint = lattice.width
    var height: uint = lattice.height
    var totalEnergy = real(0.0)
    for y=1,height-1 do
      for x=1,width-1 do
        var x1: uint = width - x
        var v: real = lattice(x, y)(0)
        var v1: real = lattice(x1, y)(0)
        var d = v - v1
        totalEnergy = totalEnergy + (d * d)
      end
    end
    C.printf("%f\n",ad.val(totalEnergy))
    factor(-totalEnergy/(temp*(lattice.width)*(lattice.height)))
  end)
end)

-- Generate unconstrained random lattice
local RandomLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local width = params.width or params.size or 128
  local height = params.height or params.width
  local scale = params.scale or 0.01
  return pfn(terra()
    var lattice = RealGrid.stackAlloc(width, height)
    for y=0,height do
      for x=0,width do
        var val = logistic(gaussian(0.0, scale, {structural=false})/scale)
        -- [util.optionally(prior, function() return quote
        --   var p = prior
        --   val = 0.5 * (p(x,y)(0) + val)
        -- end end)]
        lattice(x,y)(0) = val
        -- lattice(x,y)(0) = uniform(0.5, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0, mass=1.0})
      end
    end
    return lattice
  end)
end)

-- Turbulence lattice (weighted sum of zoomed in windows on random lattice)
local TurbulenceLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local Vec1 = Vec(real, 1)
  local width = params.width or params.size or 128
  local height = params.height or params.size or 128
  local levels = params.levels or params.turbSize or 6.0
  return terra(R: &RealGrid)
    var maxZoom: double = ad.math.pow(2.0, levels)
    var Rturb = RealGrid.stackAlloc(width, height)
    var zero = R(0, 0) - R(0, 0) -- Get zero of lattice element type
    for x=0,width do
      for y=0,height do
        var val = zero
        var zoom = maxZoom
        while zoom >= 1.0 do
          var pix = bilerp(R, x / zoom, y / zoom, Vec1)
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

-- Turbulence lattice by top-down summing of random grid subdivisions
local TurbulenceBySubdivLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local DoubleAlphaGrid = image.Image(double, 2)
  local Vec1 = Vec(real, 1)
  local width = params.width or params.size or 128
  local height = params.height or params.size or 128
  local levels = params.levels or params.turbSize or 6.0
  return terra()
    -- Create random lattices for all subdivision levels
    var Rs: Vector(RealGrid)
    m.init(Rs)
    for i=0,levels do
      var subdivFactor = ad.math.pow(2, i)
      var currW = [int](ad.math.ceil(width / [double](subdivFactor))) + 1
      var currH = [int](ad.math.ceil(height / [double](subdivFactor))) + 1
      var R = [RandomLattice(real, {width=currW, height=currH})]()
      Rs:push(R)
    end

    -- Sum lattices together into result lattice
    var maxSubdivFactor = ad.math.pow(2, levels) -- for normalization
    var Rturb = RealGrid.stackAlloc(width, height)
    var first = Rs:getPointer(0)(0, 0)
    var zero = first - first -- Get zero of lattice element type
    for y=0,height do
      for x=0,width do
        var val = zero
        var subdivFactor = 1.0
        for i=0,levels do
          var xi = x / subdivFactor
          var yi = y / subdivFactor
          var Ri = Rs:getPointer(i)
          var pix = bilerp(@Ri, xi, yi, Vec1) --Ri(xi, yi)
          val = val + subdivFactor * pix
          subdivFactor = subdivFactor * 2.0
        end
        Rturb(x, y) = (0.5 / maxSubdivFactor) * val  -- normalization
      end
    end
    m.destruct(Rs)
    return Rturb
  end
end)

-- Turbulence lattice by summing subsampled versions of a random noise grid
local TurbulenceBySubsampleLattice = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local Vec1 = Vec(real, 1)
  local width = params.width or params.size or 128
  local height = params.height or params.size or 128
  local levels = params.levels or params.turbSize or 6.0
  return terra(R: &RealGrid)
    -- subsample random lattice up to # levels
    var Rs: Vector(RealGrid)
    m.init(Rs)
    for i=0,levels do
      var subdivFactor = [double](ad.math.pow(2.0, double(i)))
      var currW = [int](ad.math.ceil(width / subdivFactor)) + 1
      var currH = [int](ad.math.ceil(height / subdivFactor)) + 1
      var Ri = RealGrid.stackAlloc(currW, currH)
      for y=0,currH do
        for x=0,currW do
            var u = x / double(currW)
            var v = y / double(currH)
            var xR = u * (width - 1)
            var yR = v * (height - 1)
            Ri(x, y) = bilerp(R, xR, yR, Vec1)
        end
      end
      Rs:push(Ri)
    end

    -- Sum sub-sampled versions of lattice into result lattice
    var maxSubdivFactor = ad.math.pow(2.0, levels) -- for normalization
    var Rturb = RealGrid.stackAlloc(width, height)
    var first = Rs:getPointer(0)(0, 0)
    var zero = first - first -- Get zero of lattice element type
    for y=0,height do
      for x=0,width do
        var val = zero
        var subdivFactor = 1.0
        for i=0,levels do
          var Ri = Rs:getPointer(i)
          var xi = (x / double(width)) * (Ri.width - 1)
          var yi = (y / double(height)) * (Ri.height - 1)
          var pix = bilerp(@Ri, xi, yi, Vec1)
          val = val + subdivFactor * pix
          subdivFactor = subdivFactor * 2.0
        end
        var valNormed = (0.5 / maxSubdivFactor) * val -- normalization
        Rturb(x, y) = valNormed
      end
    end

    m.destruct(Rs)
    return Rturb
  end
end)


-- Marble-like patterns by summing "sine" repetitions over domain with turbulence
local Marbleify = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local p = params
  return terra(T: &RealGrid)
    var width: uint      = p.width
    var height: uint     = p.height
    var xPeriod: double  = p.xPeriod
    var yPeriod: double  = p.yPeriod
    var turbPower:double = p.turbPower
    var marble: RealGrid = RealGrid.stackAlloc(width, height)
    var xNorm: double    = double(xPeriod) / double(width)
    var yNorm: double    = double(yPeriod) / double(height)
    
    for x=0,width do
      for y=0,height do
        var xyValue = (x * xNorm) + (y * yNorm) + (turbPower * T(x, y)(0))
        marble(x, y)(0) = (ad.math.sin(xyValue * 3.14159) + 1.0) * 0.5
      end
    end
    return marble
  end
end)

-- Wood-like patterns by summing circular "sine" repetitions over domain with turbulence
local Woodify = templatize(function(real, params)
  local RealGrid = image.Image(real, 1)
  local p = params
  return terra(T: &RealGrid)
    var width: uint       = p.width
    var height: uint      = p.height
    var turbSize: double  = p.turbSize
    var turbPower: double = p.turbPower
    var xyPeriod: double  = p.xyPeriod
    var wood: RealGrid    = RealGrid.stackAlloc(width, height)
    
    for x=0,width do
      for y=0,height do
        var xVal = (x - width / 2.0) / width
        var yVal = (y - height / 2.0) / height
        var dVal = ad.math.sqrt(xVal * xVal + yVal * yVal) + (turbPower * T(x, y)(0))
        wood(x, y)(0) = 0.25 * (ad.math.sin(2.0 * xyPeriod * dVal * 3.14159) + 1.0)
      end
    end
    return wood
  end
end)

-- local TransformRealGrid = templatize(function(real)
--   local RealGrid = image.Image(real, 1)
--   return terra(grid: &RealGrid, transformFun: &RealGrid -> &RealGrid)
--     var width = grid.width
--     var height = grid.height
--     var im = RGBImage.stackAlloc(width, height)
--     for x = 0,width do
--       for y = 0,height do
--         im(x,y) = transferFun(grid(x,y)(0))
--       end
--     end
--     return im
--   end
-- end)

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
    var L = 0.5 + t / 2.0
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
  local p = params
  local RGBImage = image.Image(double, 3)
  local terra test()
    var R = [RandomLattice(real, params)]()
    var lattice = [TurbulenceLattice(real, params)](&R)
    [GridSaver(real, LightnessColorizer)](&lattice, imgFilename)
  end
  test()
end

local testMarble = function(params, imgFilename)
  local RGBImage = image.Image(double, 3)
  local terra test()
    var R = [RandomLattice(real, params)]()
    var T = [TurbulenceLattice(real, params)](&R)
    var lattice = [Marbleify(real, params)](&T)
    [GridSaver(real, WeightedRGBColorizer)](&lattice, imgFilename)
  end
  test()
end

local testWood = function(params, imgFilename)
  local RGBImage = image.Image(double, 3)
  local terra test()
    var R = [RandomLattice(real, params)]()
    var T = [TurbulenceLattice(real, params)](&R)
    var lattice = [Woodify(real, params)](&T)
    [GridSaver(real, WeightedRGBColorizer)](&lattice, imgFilename)
  end
  test()
end

return {
  lerp = lerp,
  ease = ease,
  bilerp = bilerp,
  softEq = softEq,
  SplineConstraint = SplineConstraint,
  SymmetryConstraint = SymmetryConstraint,
  RandomLattice = RandomLattice,
  TurbulenceLattice = TurbulenceLattice,
  TurbulenceBySubdivLattice = TurbulenceBySubdivLattice,
  TurbulenceBySubsampleLattice = TurbulenceBySubsampleLattice,
  Marbleify = Marbleify,
  Woodify = Woodify,
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