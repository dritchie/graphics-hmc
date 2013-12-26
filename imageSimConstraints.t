
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local ad = terralib.require("ad")
local Vec = terralib.require("linalg").Vec
local image = terralib.require("image")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local softmaxTightness = 2.0
local softmax = macro(function(x, y)
	local t = softmaxTightness
	local invt = 1.0/t
	return `ad.math.pow(ad.math.pow(x, t) + ad.math.pow(y, y), invt)
end)

-- Convolutional similarity constraint: Evaluate constraint for multiple
--    transformations of the target image
local ConvSimConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage, txmin, txmax, txstep, tymin, tymax, tystep)
		return terra(image: &RealGrid)
			var iter = 0
			var energy = real(0.0)
			for ty=tymin,tymax,tystep do
				for tx=txmin,txmax,txstep do
					var mse = real(0.0)
					var totalWeight = 0.0
					for y=0,image.height do
						for x=0,image.width do
							var _x = x - tx
							var _y = y - ty
							if _x >= 0 and _x < tgtImage.width and _y >= 0 and _y < tgtImage.height
								and tgtImage(_x,_y)(1) > 0.0
								then
								totalWeight = totalWeight + tgtImage(_x,_y)(1)
								var diff = image(x,y)(0) - tgtImage(_x,_y)(0)
								diff = diff*diff*tgtImage(_x,_y)(1)	-- weight by alpha
								mse = mse + diff
							end
						end
					end
					mse = mse / totalWeight
					energy = energy + mse
					-- if iter == 0 then
					-- 	energy = mse
					-- else
					-- 	energy = softmax(energy, mse)
					-- 	-- energy = ad.math.fmax(energy, mse)
					-- end
					iter = iter + 1
				end
			end
			return energy
		end
	end
end)

-- Direct image similarity constraint
local DirectSimConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage)
		return terra(image: &RealGrid)
			var err = real(0.0)
			for y=0,image.height do
				for x=0,image.width do
					if x <= tgtImage.width and y <= tgtImage.height then
						var diff = image(x,y)(0) - tgtImage(x,y)(0)
						diff = diff*diff*tgtImage(x,y)(1)	-- weight by alpha
						err = err + diff
					end
				end
			end
			return err / (tgtImage.height*tgtImage.width)
		end
	end
end)

local lerp = macro(function(lo, hi, t)
	return `(1.0-t)*lo + t*hi
end)

-- x and y may not be ints, and may be dual nums
local bilerp = macro(function(image, x, y)
	return quote
		var x0 = [int](ad.val(x))
		var y0 = [int](ad.val(y))
		var x1 = x0 + 1
		var y1 = y0 + 1
		var tx = x - [double](x0)
		var ty = y - [double](y0)
		var v00 = image(x0, y0)
		var v01 = image(x0, y1)
		var v10 = image(x1, y0)
		var v11 = image(x1, y1)
		var v0 = lerp(v00, v01, ty)
		var v1 = lerp(v10, v11, ty)
	in
		lerp(v0, v1, tx)
	end
end)

-- Transformed similarity constraint: Evaluate constraint for a single
--    transform of the target image, but this transform is controlled
--    via random variables
local TransformedSimConstraint = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage)
		return terra(image: &RealGrid, center: Vec2)
			var centerInPixels = Vec2.stackAlloc(center(0)*[double](image.width), center(1)*[double](image.height))
			var halfWidth = [double](tgtImage.width)/2.0
			var halfHeight = [double](tgtImage.height)/2.0
			var topLeft = centerInPixels - Vec2.stackAlloc(halfWidth, halfHeight)
			var err = real(0.0)
			var totalWeight = real(0.0)
			for y=0,image.height do
				for x=0,image.width do
					var _x = [double](x) - topLeft(0)
					var _y = [double](y) - topLeft(1)
					if (_x >= 0.0 and _x < [double](tgtImage.width) and
						_y >= 0.0 and _y < [double](tgtImage.height)) then
						-- Bilinear interpolation for now (switch to bicubic if
						--    derivative discontinuities are a problem?)
						var pix = bilerp(image, _x, _y)
						var color = pix(0)
						var alpha = pix(1)
						if alpha > 0.0 then
							var diff = image(x,y)(0) - color
							diff = diff*diff*alpha	-- weight by alpha
							err = err + diff
							totalWeight = totalWeight + alpha
						end
					end
				end
			end
			err = err / totalWeight
			-- C.printf("%g\n", ad.val(err))
			return err
		end
	end
end)

return
{
	DirectSimConstraint = DirectSimConstraint,
	ConvolutionalSimConstraint = ConvSimConstraint,
	TransformedSimConstraint = TransformedSimConstraint
}



