-- Other libraries we'll need
local mem = terralib.require("mem")
local image = terralib.require("image")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")

-- Returns total difference between heights in image region
-- and constant height range [hmin,hmax] over [xmin,xmax] x [ymin,ymax]
local HeightConstraint = templatize(function(real)
  local RealGrid = image.Image(real, 1)
  return function(hmin, hmax, xmin, xmax, ymin, ymax)
    return terra(image: &RealGrid)
      var err = real(0.0)
      for y = ymin, ymax do
        for x = xmin, xmax do
          var h = image(x,y)(0)
          var d = h - h  --gets type right but TODO: properly initialize to 0
          if (h > hmax) then
            d = h - hmax
          elseif (h < hmin) then
            d = h - hmin
          end
          err = err + (d * d)
        end
      end
      return err
    end
  end
end)

-- Returns total difference between height in image and tgtImage
-- with a +/- hDelta threshold
local DeltaHeightConstraint = templatize(function(real)
  local RealGrid = image.Image(real, 1)
  local DoubleGrid = image.Image(double, 1)
  return function(hDelta, xmin, xmax, ymin, ymax)
    return terra(image: &RealGrid, tgtImage: &DoubleGrid)
      var err = real(0.0)
      for y = ymin, ymax do
        for x = xmin, xmax do
          var tH = tgtImage(x, y)(0)
          var currH = image(x, y)(0)
          var hmax = tH + hDelta
          var hmin = tH - hDelta
          var d = currH - currH --gets type right but TODO: properly initialize to 0
          if (currH > hmax) then
            d = currH - hmax
          elseif (currH < hmin) then
            d = currH - hmin
          end
          err = err + (d * d)
        end
      end
      return err
    end
  end
end)

return
{
  HeightConstraint = HeightConstraint,
  DeltaHeightConstraint = DeltaHeightConstraint
}