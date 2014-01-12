
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local ad = terralib.require("ad")
local Vec = terralib.require("linalg").Vec
local image = terralib.require("image")
local rand = terralib.require("prob.random")

local C = terralib.includecstring [[
#include <stdio.h>
]]

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
local bilerp = macro(function(image, x, y, Vec2)
	return quote
		var x0 = [int](ad.math.floor(ad.val(x)))
		var x1 = [int](ad.math.ceil(ad.val(x)))
		var y0 = [int](ad.math.floor(ad.val(y)))
		var y1 = [int](ad.math.ceil(ad.val(y)))
		var tx = x - [double](x0)
		var ty = y - [double](y0)
		var v00 = Vec2(image(x0, y0))
		var v01 = Vec2(image(x0, y1))
		var v10 = Vec2(image(x1, y0))
		var v11 = Vec2(image(x1, y1))
		-- var v0 = lerp(v00, v01, ty)
		-- var v1 = lerp(v10, v11, ty)
		var v0 = ease(v00, v01, ty)
		var v1 = ease(v10, v11, ty)
	in
		-- lerp(v0, v1, tx)
		ease(v0, v1, tx)
	end
end)

local softmaxTightness = 40.0
local softmax = macro(function(x, y)
	local t = softmaxTightness
	local invt = 1.0/t
	return `ad.math.pow(ad.math.pow(x, t) + ad.math.pow(y, t), invt)
end)

-- Convolutional similarity constraint: Evaluate constraint for multiple
--    transformations of the target image
local ConvSimConstraint = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage, txmin, txmax, txstep, tymin, tymax, tystep)
		return terra(image: &RealGrid)
			var energy = real(0.0)
			for ty=tymin,tymax,tystep do
				for tx=txmin,txmax,txstep do
					var mse = real(0.0)
					var totalWeight = real(0.0)
					for y=0,image.height do
						for x=0,image.width do
							var _x = [double](x) - tx
							var _y = [double](y) - ty
							if _x >= 0.0 and _x <= [double](tgtImage.width-1) and
								 _y >= 0.0 and _y <= [double](tgtImage.height-1) then
								-- var pix = bilerp(tgtImage, _x, _y, Vec2)
								var pix = tgtImage([int](_x), [int](_y))
								var color = pix(0)
								var alpha = pix(1)
								if alpha > 0.0 then
									var diff = image(x,y)(0) - color
									diff = diff*diff*alpha	-- weight by alpha
									mse = mse + diff
									totalWeight = totalWeight + alpha
								else
									var diff = image(x,y)(0)
									diff = diff*diff
									totalWeight = totalWeight + 1.0
								end
							end
						end
					end
					if totalWeight > 0.0 then
						mse = mse / totalWeight
						-- mse = ad.math.exp(-mse)
						mse = [rand.gaussian_logprob(real)](mse, 0.0, 0.05)
						mse = ad.math.exp(mse)
					end
					energy = energy + mse
					-- if energy == 0.0 then
					-- 	energy = mse
					-- else
					-- 	energy = softmax(energy, mse)
					-- 	-- energy = ad.math.fmax(energy, mse)
					-- end
				end
			end
			-- C.printf("%g            \n", ad.val(energy))
			-- return energy
			return -ad.math.log(energy)
		end
	end
end)

-- Direct image similarity constraint
local DirectSimConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage)
		return terra(image: &RealGrid)
			var err = real(0.0)
			var totalWeight = 0.0
			for y=0,image.height do
				for x=0,image.width do
					if x < tgtImage.width and y < tgtImage.height then
						var color = tgtImage(x,y)(0)
						var alpha = tgtImage(x,y)(1)
						if alpha > 0.0 then
							var diff = image(x,y)(0) - color
							-- C.printf("%g          \n", ad.val(image(x,y)(0)))
							diff = diff*diff*alpha	-- weight by alpha
							err = err + diff
							totalWeight = totalWeight + alpha
						end
					end
				end
			end
			err = err / totalWeight
			-- C.printf("%g         \n", ad.val(err))
			return err
		end
	end
end)

-- Returns total difference between image and tgtImage outside a +/- hDelta threshold
local DirectSimDeltaConstraint = templatize(function(real)
	local RealGrid = image.Image(real, 1)
	local DoubleGrid = image.Image(double, 2)
	return function(hDelta)
		return terra(image: &RealGrid, tgtImage: &DoubleGrid)
			var err = real(0.0)
			for y = 0, image.height do
				for x = 0, image.width do
					var tH = 0.0
					if x < tgtImage.width and y < tgtImage.height then
						tH = tgtImage(x, y)(0)
					end
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

-- Transformed similarity constraint: Evaluate constraint for a single
--    transform of the target image, but this transform is controlled
--    via random variables
local TransformedSimConstraint = templatize(function(real)
	-- local Vec2 = Vec(real, 2)
	local Vec2 = Vec(double, 2)
	local RealGrid = image.Image(real, 1)
	return function(tgtImage, softness)
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
					if (_x >= 0.0 and _x <= [double](tgtImage.width-1) and
						_y >= 0.0 and _y <= [double](tgtImage.height-1)) then
						var pix = bilerp(tgtImage, _x, _y, Vec2)
						var color = pix(0)
						var alpha = pix(1)
						if alpha > 0.0 then
							var localerr = [rand.gaussian_logprob(real)](image(x,y)(0), color, softness)
							localerr = localerr*alpha
							err = err + localerr
							totalWeight = totalWeight + alpha
						end
					else
						-- Assume zero target outside target image
						var color = real(0.0)
						var alpha = real(1.0)
						var localerr = [rand.gaussian_logprob(real)](image(x,y)(0), color, softness)
						localerr = localerr*alpha
						err = err + localerr
						totalWeight = totalWeight + alpha
					end
				end
			end
			if totalWeight > 0.0 then
				err = err / totalWeight
			end
			-- C.printf("%g                  \n", ad.val(err))
			return err
		end
	end
end)

return
{
	DirectSimConstraint = DirectSimConstraint,
	DirectSimDeltaConstraint = DirectSimDeltaConstraint,
	ConvolutionalSimConstraint = ConvSimConstraint,
	TransformedSimConstraint = TransformedSimConstraint
}



