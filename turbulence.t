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
  var lattice = [U.RandomLattice(real)](size, size)

  -- Turbulence pass
  var lattTurb = [U.TurbulenceLattice(real)](size, size, 128)

  -- Create cloud RGB image
  var cloudizer = [U.LightnessColorizer(real)]
  var im = [U.RealGridToRGBImage(real)](&lattTurb, cloudizer)
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

  -- Lattice
  -- var lattice = [U.MarbleLattice(real)](size, size, xPeriod, yPeriod, turbPower, turbSize)
  var lattice = [U.WoodLattice(real)](size, size, xyPeriod, turbPower, turbSize)

  -- Create RGB image
  var im = RGBImage.stackAlloc(size, size)
  var woodizer = [U.WeightedRGBColorizer(real)]
  for x = 0,size do
    for y = 0,size do
      var t = lattice(x, y)(0)
      im(x, y) = woodizer(t)
    end
  end
  [RGBImage.save(uint8)](&im, image.Format.PNG, testImgFilename)
end
testMarble()


-- util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
-- util.wait(string.format("rm -f %s", movieframewildcard))
print("done.")
