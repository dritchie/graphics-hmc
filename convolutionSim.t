
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local ad = terralib.require("ad")
local Vec = terralib.require("linalg").Vec
local image = terralib.require("image")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local DoubleGrid = image.Image(double, 1);

local softmaxTightness = 2.0
local softmax = macro(function(x, y)
	local t = softmaxTightness
	local invt = 1.0/t
	return `ad.math.pow(ad.math.pow(x, t) + ad.math.pow(y, y), invt)
end)

-- Convolutional similarity constraint
local ConvSimConstraint = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local RealGrid = image.Image(real, 1);
	return function(tgtImage, txmin, txmax, txstep, tymin, tymax, tystep)
		return terra(image: &RealGrid)
			var iter = 0
			var score = real(0.0)
			for ty=tymin,tymax,tystep do
				for tx=txmin,txmax,txstep do
					var mse = real(0.0);
					var numPixelsContributing = 0
					for y=0,image.height do
						for x=0,image.width do
							var _x = x - tx;
							var _y = y - ty;
							if _x >= 0 and _x < tgtImage.width and _y >= 0 and _y < tgtImage.height then
								numPixelsContributing = numPixelsContributing + 1
								mse = mse + image(x,y):distSq(tgtImage(_x,_y))
							end
						end
					end
					mse = mse / numPixelsContributing;
					score = score + mse
					-- if iter == 0 then
					-- 	score = mse
					-- else
					-- 	score = softmax(score, mse)
					-- 	-- score = ad.math.fmax(score, mse)
					-- end
					iter = iter + 1
				end
			end
			return score
		end
	end
end)

return ConvSimConstraint



