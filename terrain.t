-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local mem = terralib.require("mem")
local image = terralib.require("image")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local util = terralib.require("util")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")

local constraints = terralib.require("terrainConstraints")
local U = terralib.require("terrainUtils")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

-- Simple 2D rectangle
local Rectangle = templatize(function(real)
  local Vec2 = Vec(real, 2)
  local struct RectangleT { min: Vec2, max: Vec2 }
  terra RectangleT:diagonal() return self.max - self.min end
  terra RectangleT:area()
    var d = self.diagonal()
    return d(0) * d(1)
  end
  terra RectangleT:intersectArea(o: &RectangleT)
    return 0.0
  end
  return RectangleT
end)

-- Representation of a terrain region
local Region = templatize(function(real)
  local RectangleT = Rectangle(real)
  local struct RegionT {}
  inheritance.staticExtend(RectangleT, RegionT)
  return RegionT
end)

-- Representation of a terrain map containing regions
local TerrainMap = templatize(function(real)
  local RegionT = Region(real)
  local RealGrid = image.Image(real, 1)
  local Vec2 = Vec(real, 2)
  local Vec2i = Vec(int, 2)
  
  local struct MapT
  {
    grid: RealGrid,
    regions: Vector(RegionT)
  }
  terra MapT:__construct(w: uint, h: uint): {}
    self.grid = RealGrid.stackAlloc(w, h)
    mem.init(self.regions)
  end
  terra MapT:__destruct()
    mem.destruct(self.regions)
  end
  terra MapT:__copy(o: &MapT)
    self.grid = mem.copy(o.grid)
    self.regions = mem.copy(o.regions)
  end

  -- Return value at uv coordinates
  terra MapT:getPixelByUV(u: real, v: real)
    var x = [int](ad.math.round(u * self.grid.width))
    var y = [int](ad.math.round(v * self.grid.height))
    return self.grid(x, y)
  end
  
  mem.addConstructors(MapT)
  return MapT
end)

local fractalSplineModel = templatize(function(real)
  local TerrainMapT = TerrainMap(real)
  local HeightConstraintT = constraints.HeightConstraint(real)
  local DeltaHeightConstraintT = constraints.DeltaHeightConstraint(real)
  local hE1 = HeightConstraintT(0.1, 0.3, 0, 15, 0, 15)
  local hE2 = HeightConstraintT(0.8, 1.0, 20, 30, 20, 35)
  local dHE = DeltaHeightConstraintT(0.2, 0, 100, 0, 70)

  return function(tgtImage, temp, rigidity, tension, c)
    return terra()
      var width = tgtImage.width
      var height = tgtImage.height 
      var terrainMap = TerrainMapT.stackAlloc(width, height)
      var lattice = terrainMap.grid

      -- Priors
      for y=0,height do
        for x=0,width do
          lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
        end
      end

      -- Variational spline derivative constraints
      var h = 1.0 / ad.math.fmax(width, height)
      for y = 1, height-1 do
        for x = 1, width-1 do
          var dx = U.Dx(lattice, x, y, h)
          var dy = U.Dy(lattice, x, y, h)
          var dxx = U.Dxx(lattice, x, y, h)
          var dyy = U.Dyy(lattice, x, y, h)
          var dxy = U.Dxy(lattice, x, y, h)
          var e_p = 0.5 * rigidity *
            ((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
          --var diff = lattice(x,y):distSq(tgtImage(x, y))
          --var e_d = 0.5 * c * diff
          var energy = e_p(0) --+ e_d
          factor(-energy / temp)
        end
      end
      -- var hE = hEnergy(&lattice, c, temp)
      -- var hE2 = hEnergy2(&lattice, c, temp)
      -- factor(-hE/temp)
      -- factor(-hE2/temp)
      factor(-0.5 * c * dHE(&lattice, &tgtImage) / temp)
      return lattice
    end
  end
end)

return
{
  TerrainMap = TerrainMap,
  fractalSplineModel = fractalSplineModel,
}