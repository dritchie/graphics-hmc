terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand =terralib.require("prob.random")
local gl = terralib.require("gl")
local image = terralib.require("image")
local glUtils = terralib.require("glUtils")

local C = terralib.includecstring [[
#include <stdio.h>
]]

--------------------------------------------

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)

--------------------------------------------

local softEq = macro(function(x, target, softness)
  return `[rand.gaussian_logprob(real)](x, target, softness)
end)
local upperBound = macro(function(val, bound, softness)
  return quote
    if val > bound then
      factor(softEq(val-bound, 0.0, softness))
    end 
  end
end)
local lowerBound = macro(function(val, bound, softness)
  return quote
    if val < bound then
      factor(softEq(bound-val, 0.0, softness))
    end 
  end
end)
local bound = macro(function(val, lo, hi, softness)
  return quote
    lowerBound(val, lo, softness)
    upperBound(val, hi, softness)
  end
end)
local lowerClamp = macro(function(val, lo)
  return `ad.math.fmax(val, lo)
end)
local upperClamp = macro(function(val, hi)
  return `ad.math.fmin(val, hi)
end)
local clamp = macro(function(val, lo, hi)
  return `lowerClamp(upperClamp(val, hi), lo)
end)

--------------------------------------------

local ngaussian = macro(function(m, sd)
  return `gaussian(m, sd, {structural=false})
end)
local nuniformNoPrior = macro(function(lo, hi)
  return `uniform(lo, hi, {structural=false, hasPrior=false})
end)

--------------------------------------------


local Shape = templatize(function(real)
  local CircleT = glUtils.Circle(real)
  local struct ShapeT {}
  inheritance.staticExtend(CircleT, ShapeT)
  return ShapeT
end)

local ShapeGroup = templatize(function(real)
  local Vec2 = Vec(real, 2)
  local CircleT = glUtils.Circle(real)
  local ShapeT = Shape(real)
  local struct ShapeGroupT {
    children: Vector(ShapeT),
    childSizeRatio: real
  }
  inheritance.staticExtend(CircleT, ShapeGroupT)
  terra ShapeGroupT:__construct() : {}
    m.init(self.pos)
    self.size = 0.0
    m.init(self.children)
    self.childSizeRatio = 1.0 / 5.0
  end
  terra ShapeGroupT:__construct(pos: Vec2, size: real, color: Color3d, childSizeRatio: real) : {}
    self.pos = pos
    self.size = size
    self.color = color
    self.childSizeRatio = childSizeRatio
    m.init(self.children)
  end
  terra ShapeGroupT:render()
    ShapeT.render(self)
    for i=0,self.children.size do
      var child = self.children:getPointer(i)
      child:render()
    end
  end
  m.addConstructors(ShapeGroupT)
  return ShapeGroupT
end)

local Pattern = templatize(function(real)
  local ShapeGroupT = ShapeGroup(real)
  local struct PatternT {
    width: real,
    height: real,
    groups: Vector(ShapeGroupT),
  }
  terra PatternT:__construct() : {}
    self:__construct(0.0, 0.0)
  end
  terra PatternT:__construct(w: real, h: real) : {}
    self.width = w
    self.height = h
    m.init(self.groups)
  end
  terra PatternT:render()
    for i=0,self.groups.size do
      self.groups:getPointer(i):render()
    end
  end
  m.addConstructors(PatternT)
  return PatternT
end)

--------------------------------------------

local function model()
  local patternWidth = `100.0
  local patternHeight = `100.0
  local numShapeGroups = 3
  local numChildren = 2
  local Vec2 = Vec(real, 2)
  local ShapeT = Shape(real)
  local ShapeGroupT = ShapeGroup(real)
  local PatternT = Pattern(real)

  local groupColor = `Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
  local childColor = `Color3d.stackAlloc(255/255.0, 127/255.0, 14/255.0)

  --------------------------------------------

  local terra polar2rect(polarVec: Vec2)
    var r = polarVec(0)
    var theta = polarVec(1)
    return Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
  end

  --------------------------------------------

  local makeChild = pfn(terra(parent: &ShapeGroupT)
    var childSize = parent.size * parent.childSizeRatio
    var polarPos = Vec2.stackAlloc(1.5 * parent.size, nuniformNoPrior(0.0, [2*math.pi]))
    -- var polarPos = Vec2.stackAlloc(nuniformNoPrior(0.0, parent.size), nuniformNoPrior(0.0, [2*math.pi]))
    -- clamp(polarPos(0), 0.0, parent.size, 0.2)  -- Soft bound child position
    var childPos = parent.pos + polar2rect(polarPos)
    return ShapeT { childPos, childSize, childColor }
  end)

  local makeShapeGroup = pfn()
  local terra makeShapeGroupImpl(numChildren: int, pos: Vec2) : ShapeGroupT
    var size = 10.0 --ngaussian(10.0, 1.0)
    -- bound(size, 5.0, 15.0, 0.1)
    var childSizeRatio = 0.4 --nuniformNoPrior(size/5.0, size/2.0)
    var t = ShapeGroupT.stackAlloc(pos, size, groupColor, childSizeRatio)
    for i=0,numChildren do
      t.children:push(makeChild(&t))
    end
    -- Factor: children don't overlap
    for i=0,numChildren-1 do
      var p1 = t.children:getPointer(i)
      for j=i+1,numChildren do
        var p2 = t.children:getPointer(j)
        factor(softEq(p1:intersectArea(p2), 0.0, 0.2))
      end
    end
    return t
  end
  terra makeShapeGroupImpl(numChildren: int) : ShapeGroupT
    var pos = Vec2.stackAlloc(nuniformNoPrior(0.0, patternWidth), nuniformNoPrior(0.0, patternHeight))
    var t = makeShapeGroup(numChildren, pos)
    return t
  end
  makeShapeGroup:define(makeShapeGroupImpl)

  --------------------------------------------

  return terra()

    var pattern = PatternT.stackAlloc(patternWidth, patternHeight)
    var groups = &pattern.groups

    -- Spawn groups
    for i=0,numShapeGroups do
      var t = makeShapeGroup(numChildren)
      groups:push(t)
      m.destruct(t)
    end

    -- Non-overlap
    for i=0,groups.size-1 do
      for j=i+1,groups.size do
        var gi = groups:getPointer(i)
        var gj = groups:getPointer(j)
        -- group-group
        factor(softEq(gi:intersectArea(gj), 0.0, 1.0))
        -- group-child
        for k=0,numChildren do
          var gj_k = gj.children:getPointer(k)
          factor(softEq(gi:intersectArea(gj_k), 0.0, 1.0))
          -- child-child
          for l=0,numChildren do
            var gi_l = gi.children:getPointer(l)
            factor(softEq(gi_l:intersectArea(gj_k), 0.0, 1.0))
          end
        end
        

      end
    end

    -- Attract children to parent
    var d = real(0.0)
    var touchDist = groups:getPointer(0).size + groups:getPointer(0).children:getPointer(0).size
    for i=0,groups.size-1 do
      for j=i+1,groups.size do
        var gi = groups:getPointer(i)
        var gj = groups:getPointer(j) 
        for k=0,numChildren do
          var gi_k = gi.children:getPointer(k)
          var gj_k = gj.children:getPointer(k)
          factor(softEq(gi_k.pos:dist(gj.pos), 1.1*touchDist, 1.0))
          factor(softEq(gj_k.pos:dist(gi.pos), 1.1*touchDist, 1.0))
        end
      end
    end

    return pattern
  end
end

----------------------------------

local kernelType = HMC
-- local kernelType = RandomWalk
local hmcNumSteps = 100
local numsamps = 2000
local verbose = true

if kernelType == RandomWalk then numsamps = 0.75*hmcNumSteps*numsamps end
local kernel = nil
if kernelType == RandomWalk then kernel = RandomWalk() else kernel = HMC({numSteps=hmcNumSteps}) end
local terra doInference()
  return [mcmc(model, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

local moviename = arg[1] or "movie"
local imageWidth = 200
local imageHeight = 200
glUtils.renderSamples(samples, moviename, imageWidth, imageHeight)
