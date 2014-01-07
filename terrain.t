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

return
{
  TerrainMap = TerrainMap
}