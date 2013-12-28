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
#include <math.h>

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


local Color = Vec(real, 3)

local function clamp(value)
	return `C.fmin(C.fmax(value, 0.0), 1.0)	
end

--for now, copy this over from rand.t
--figure out how to call from terra?
local terra gaussian_logprob(x:double, mu:double, sigma:double)
    var xminusmu = x - mu
    return -.5*(1.8378770664093453 + 2*ad.math.log(sigma) + xminusmu*xminusmu/(sigma*sigma))
end

local RGBtoLAB = terra(rgb:Color)
    var gamma = 2.2
    var red = C.pow(rgb(0), gamma)
    var green = C.pow(rgb(1), gamma)
    var blue = C.pow(rgb(2), gamma)

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

    var fx = 0.0
    if (xR > e) then fx = C.pow(xR, 1 / 3.0) else fx = (k * xR + 16) / 116.0 end

    var fy = 0.0
    if (yR > e) then fy = C.pow(yR, 1 / 3.0) else fy = (k * yR + 16) / 116.0 end
    var fz = 0.0
    if (zR > e) then fz = C.pow(zR, 1 / 3.0) else fz = (k * zR + 16) / 116.0 end

    var cieL = 116 * fy - 16
    var cieA = 500 * (fx - fy)
    var cieB = 200 * (fy - fz) 

    var result = Color.stackAlloc(cieL, cieA, cieB)
    return result
end

local LABtoRGB = terra(lab:Color)
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
    var xR = C.pow(fx, 3.0)
    var zR = C.pow(fz, 3.0)

    if (xR <= e) then
    	xR = (116 * fx - 16) / k
    end

    var yR = 0.0
    if (cieL > (k * e))  then yR = C.pow((cieL + 16) / 116.0, 3.0) else yR = cieL / k end
    
    if (zR <= e) then zR = (116 * fz - 16) / k end

    var x = xR * XR
    var y = yR * YR
    var z = zR * ZR

    var r = M[0][0] * x + M[0][1] * y + M[0][2] * z
    var g = M[1][0] * x + M[1][1] * y + M[1][2] * z
    var b = M[2][0] * x + M[2][1] * y + M[2][2] * z

    var red = C.pow([clamp(r)], 1.0 / gamma)
    var green = C.pow([clamp(g)], 1.0 / gamma)
    var blue = C.pow([clamp(b)], 1.0 / gamma)

    var result = Color.stackAlloc(red, green, blue)
    return result
end



--Prefer lightness, bimodal lightness for backgrounds
local UnaryLightnessConstraint = templatize(function(real)
	local RealPattern = Pattern(real)
	return terra(pattern: &RealPattern)
		var mu = 0.8
		var mu2 = 0.3
		var sigma = 0.4

		var totalScore = 0.0
		for i=0,pattern.numGroups do
			var L = RGBtoLAB(pattern(i))(0)/100.0
			var score = gaussian_logprob(L, mu, sigma)

			--if it's the background allow darkness, otherwise prefer lighter colors
			if (i == pattern.backgroundId) then
				score = (C.exp(score) + C.exp(gaussian_logprob(L, mu2, sigma)))/2.0
				score = C.log(score)
			end
			totalScore = totalScore + score
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
		var totalScore = 0.0
		for i=0,pattern.numGroups do
			var lab = RGBtoLAB(pattern(i))
			var chroma = lab(1)*lab(1) + lab(2)*lab(2)
			var saturation = 0.0
			if ((chroma + lab(0)*lab(0)) > 0.0) then
				saturation = C.sqrt(chroma)/C.sqrt(chroma + lab(0)*lab(0))
			end
			--C.printf("saturation %f\n", saturation)
			if (saturation > 1) then
				C.printf("saturation over %f", saturation)
			end
			if (i == pattern.backgroundId) then
				var score = gaussian_logprob(saturation, low, sigma)
				totalScore = totalScore + score
			else
				var score = gaussian_logprob(saturation, high, sigma)
				totalScore = totalScore + score
			end
		end
		return totalScore
	end
end)


--Prefer perceptual difference of some amount
local BinaryPerceptualConstraint = templatize(function(real)
	local RealPattern = Pattern(real)
	local ColorList = Vector(Color)
	return terra(pattern: &RealPattern)
		var mu = 0.3
		var sigma = 0.2
		var totalScore = 0.0
		var labs = ColorList.stackAlloc(pattern.numGroups, Color.stackAlloc(0,0,0))
		--convert colors to lab
		for i=0,pattern.numGroups do
			labs:set(i, RGBtoLAB(pattern(i)))
		end

		var maxDist = C.sqrt(100.0*100.0+200.0*200.0+200.0*200.0)

		for i=0,pattern.adjacencies.size do
			var adj = pattern.adjacencies:get(i)
			var first = adj:get(0)
			var second = adj:get(1)
			var dist = labs:get(first):dist(labs:get(second))/maxDist
			--C.printf("dist %f\n", dist)
			var score = gaussian_logprob(dist, mu, sigma)
			totalScore = totalScore + score
		end
		return totalScore
	end
end)

return {
	LABtoRGB = LABtoRGB,
	RGBtoLAB = RGBtoLAB,
	UnaryLightnessConstraint = UnaryLightnessConstraint,
	UnarySaturationConstraint = UnarySaturationConstraint,
	BinaryPerceptualConstraint = BinaryPerceptualConstraint
}





