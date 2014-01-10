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
local U = terralib.require("logoUtils")

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
      var t = U.turbulence(lattice, x, y, 128.0)
      lattTurb(x, y) = t
    end
  end

  -- Create RGB image
  var im = RGBImage.stackAlloc(size, size)
  var h2r = [U.HSLtoRGB(double)]
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
end
-- testTurbulence()

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
  var h2r = [U.HSLtoRGB(double)]
  for x = 0,size do
    for y = 0,size do
      --var t = U.marble(lattice, x, y, xPeriod, yPeriod, turbPower, turbSize)
      var t = U.wood(lattice, x, y, xyPeriod, turbPower, turbSize)
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
