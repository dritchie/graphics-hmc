-- Include quicksand
local rand = terralib.require("prob.random")
terralib.require("prob")

local ad = terralib.require("ad")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")
local Pattern = terralib.require("pattern")
local C = terralib.includecstring [[
#include <stdlib.h>
#include <stdio.h>

#ifndef MATHPATCH_H
#define MATHPATCH_H

inline double fmin(double a, double b)
{
	return (a<b)? a : b;
}

inline double fmax(double a, double b)
{
	return (a>b)? a : b;
}

#endif
]]

local useRGB = false

local gaussian_logprob = rand.gaussian_logprob

local function clamp(value)
	return `ad.math.fmin(ad.math.fmax(value, 0.0), 1.0)	
end

local RGBtoLAB = templatize(function(real)
	local Color = Vec(real, 3)
	return terra(rgb:Color)
	    var gamma = 2.2
	    var red = ad.math.pow(rgb(0), gamma)
	    var green = ad.math.pow(rgb(1), gamma)
	    var blue = ad.math.pow(rgb(2), gamma)

	    --sRGB to xyz using the D65 illuminant
	    --transformation from http://www.brucelindbloom.com
	    var M = array(
	        array(0.4124564, 0.3575761, 0.1804375),
	        array(0.2126729, 0.7151522, 0.0721750),
	        array(0.0193339, 0.1191920, 0.9503041))

	    var x = M[0][0] * red + M[0][1] * green + M[0][2] * blue
	    var y = M[1][0] * red + M[1][1] * green + M[1][2] * blue
	    var z = M[2][0] * red + M[2][1] * green + M[2][2] * blue


	    var XR = 0.95047
	    var YR = 1.00000
	    var ZR = 1.08883

	    var e = 216 / 24389.0
	    var k = 24389 / 27.0

	    var xR = x / XR
	    var yR = y / YR
	    var zR = z / ZR

	    var fx = real(0.0)
	    if (xR > e) then fx = ad.math.pow(xR, 1 / 3.0) else fx = (k * xR + 16) / 116.0 end

	    var fy = real(0.0)
	    if (yR > e) then fy = ad.math.pow(yR, 1 / 3.0) else fy = (k * yR + 16) / 116.0 end
	    var fz = real(0.0)
	    if (zR > e) then fz = ad.math.pow(zR, 1 / 3.0) else fz = (k * zR + 16) / 116.0 end

	    var cieL = 116 * fy - 16
	    var cieA = 500 * (fx - fy)
	    var cieB = 200 * (fy - fz) 

	    var result = Color.stackAlloc(cieL, cieA, cieB)
	    return result
	end
end)

local LABtoRGB = templatize(function(real)
	local Color = Vec(real, 3)
	return terra(lab:Color)
	    var gamma = 2.2
	    var e = 216 / 24389.0
	    var k = 24389 / 27.0

	    var XR = 0.95047
	    var YR = 1.0
	    var ZR = 1.08883

	    var cieL = lab(0)
	    var cieA = lab(1)
	    var cieB = lab(2)

	    var fy = (cieL + 16) / 116.0
	    var fx = (cieA / 500.0) + fy
	    var fz = fy - cieB / 200.0

	    var M = array(array(3.2404542, -1.5371385, -0.4985314),
	        array(-0.9692660, 1.8760108, 0.0415560),
	        array(0.0556434, -0.2040259, 1.0572252))
	    var xR = ad.math.pow(fx, 3.0)
	    var zR = ad.math.pow(fz, 3.0)

	    if (xR <= e) then
	    	xR = (116 * fx - 16) / k
	    end

	    var yR = 0.0
	    if (cieL > (k * e))  then yR = ad.math.pow((cieL + 16) / 116.0, 3.0) else yR = cieL / k end
	    
	    if (zR <= e) then zR = (116 * fz - 16) / k end

	    var x = xR * XR
	    var y = yR * YR
	    var z = zR * ZR

	    var r = M[0][0] * x + M[0][1] * y + M[0][2] * z
	    var g = M[1][0] * x + M[1][1] * y + M[1][2] * z
	    var b = M[2][0] * x + M[2][1] * y + M[2][2] * z

	    var red = ad.math.pow([clamp(r)], 1.0 / gamma)
	    var green = ad.math.pow([clamp(g)], 1.0 / gamma)
	    var blue = ad.math.pow([clamp(b)], 1.0 / gamma)

	    var result = Color.stackAlloc(red, green, blue)
	    return result
	end
end)


--Prefer lightness, bimodal lightness for backgrounds
local UnaryLightnessConstraint = templatize(function(real)
	local RealPattern = Pattern(real)
	return terra(pattern: &RealPattern)
		var mu = 0.8
		var mu2 = 0.3
		var sigma = 0.4

		var totalScore = real(0.0)
		for i=0,pattern.numGroups do
			var L = real(0.0)
			if (useRGB) then
				L = [RGBtoLAB(real)](pattern(i))(0)/100.0
			else
				L = pattern(i)(0)/100.0
			end
			var score = [gaussian_logprob(real)](L, mu, sigma)

			--if it's the background allow darkness, otherwise prefer lighter colors
			if (i == pattern.backgroundId) then
				score = (0.7*ad.math.exp(score) + 0.3*ad.math.exp([gaussian_logprob(real)](L, mu2, sigma)))
				score = ad.math.log(score)
			end
			totalScore = totalScore + pattern.sizes:get(i)*score
		end
		return totalScore
	end
end)

-- Prefer saturated foregrounds, desaturated backgrounds
local UnarySaturationConstraint = templatize(function(real)
	local RealPattern = Pattern(real)
	return terra(pattern: &RealPattern)
		var high = 0.7
		var low = 0.3
		var sigma = 1.0
		var totalScore = real(0.0)
		for i=0,pattern.numGroups do
			var lab = pattern(i)
			if (useRGB) then
				lab = [RGBtoLAB(real)](pattern(i))
			end
			var chroma = lab(1)*lab(1) + lab(2)*lab(2)
			var saturation = real(0.0)
			if ((chroma + lab(0)*lab(0)) > 0.0) then
				saturation = ad.math.sqrt(chroma)/ad.math.sqrt(chroma + lab(0)*lab(0))
			end
			if (saturation > 1) then
				C.printf("saturation over %g", ad.val(saturation))
			end
			if (i == pattern.backgroundId) then
				var score = [gaussian_logprob(real)](saturation, low, sigma)
				totalScore = totalScore + pattern.sizes:get(i)*score
			else
				var score = [gaussian_logprob(real)](saturation, high, sigma)
				totalScore = totalScore + pattern.sizes:get(i)*score
			end
		end
		return totalScore
	end
end)


--Prefer perceptual difference of some amount
local BinaryPerceptualConstraint = templatize(function(real)
	local Color = Vec(real, 3)
	local RealPattern = Pattern(real)
	local ColorList = Vector(Color)
	return terra(pattern: &RealPattern)
		var mu = 0.3
		var sigma = 0.2
		var totalScore = real(0.0)
		--var black = Color.stackAlloc(0,0,0)
		--var labs = ColorList.stackAlloc(pattern.numGroups, black)
		--convert colors to lab
		--for i=0,pattern.numGroups do
		--	labs:set(i, [RGBtoLAB(real)](pattern(i)))
		--end

		var maxDist = ad.math.sqrt(100.0*100.0+200.0*200.0+200.0*200.0)
		for i=0,pattern.adjacencies.size do
			var adj = pattern.adjacencies:get(i)
			var first = pattern(adj:get(0))
			var second = pattern(adj:get(1))
			
			if (useRGB) then
				first = [RGBtoLAB(real)](first)
				second = [RGBtoLAB(real)](second)
			end

			var dist = first:dist(second)/maxDist
			--C.printf("dist %g\n", ad.val(dist))
			var score = [gaussian_logprob(real)](dist, mu, sigma)
			totalScore = totalScore + score
		end
		return totalScore/pattern.adjacencies.size
	end
end)

local BinaryLightnessConstraint = templatize(function(real)
	local Color = Vec(real, 3)
	local RealPattern = Pattern(real)
	local ColorList = Vector(Color)
	return terra(pattern: &RealPattern)
		var mu = 0.2
		var sigma = 0.4
		var mu_background = 0.3
		var sigma_background = 0.4
		var totalScore = real(0.0)

		var maxDist = 100.0
		var numAdj = pattern.adjacencies.size
		for i=0,pattern.adjacencies.size do
			var adj = pattern.adjacencies:get(i)
			var first = real(0.0)
			var second = real(0.0)
				
			if (useRGB) then
				first = [RGBtoLAB(real)](pattern(adj:get(0)))(0)
				second = [RGBtoLAB(real)](pattern(adj:get(1)))(0)
			else
				first = pattern(adj:get(0))(0)
				second = pattern(adj:get(1))(0)
			end

			var dist = ad.math.fabs(first-second)/maxDist

			if (adj:get(0)==pattern.backgroundId or adj:get(1)==pattern.backgroundId) then
				var score = [gaussian_logprob(real)](dist, mu_background, sigma_background)
				totalScore = totalScore + score
			else
				var score = [gaussian_logprob(real)](dist, mu, sigma)
				totalScore = totalScore + score
			end

		end
		return totalScore/numAdj
	end
end)


return {
	LABtoRGB = LABtoRGB,
	RGBtoLAB = RGBtoLAB,
	UnaryLightnessConstraint = UnaryLightnessConstraint,
	UnarySaturationConstraint = UnarySaturationConstraint,
	BinaryPerceptualConstraint = BinaryPerceptualConstraint,
	BinaryLightnessConstraint = BinaryLightnessConstraint,
	useRGB = useRGB
}





