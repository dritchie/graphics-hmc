-- Include quicksand
local rand = terralib.require("prob.random")
terralib.require("prob")

-- Other libraries we'll need
--local image = terralib.require("image")
local m = terralib.require("mem")
local util = terralib.require("util")
local Vector = terralib.require("vector")
local Vec = terralib.require('linalg').Vec
local Pattern = terralib.require("pattern")
local ColorUtils = terralib.require('colorUtils')
local ad = terralib.require("ad")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
]]


local function colorCompatModel()
	local temp = 1
	local constraintTemp = 1
	local glowRange = 3
	local hueRange = 10
	local compatScale = 0.1 -- scaling compatibility to other factors...

	local RealPattern = Pattern(real)
	
	local lightnessFn = ColorUtils.UnaryLightnessConstraint(real)
	local saturationFn = ColorUtils.UnarySaturationConstraint(real)
	local diffFn = ColorUtils.BinaryPerceptualConstraint(real)
	local lightnessDiffFn = ColorUtils.BinaryLightnessConstraint(real)

	local logistic = macro(function(x)
                return `1.0 / (1.0 + ad.math.exp(-x))
    	end)

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)

	local terra glowConstraint(pattern:&RealPattern, l_indices:Vector(int), h_indices:Vector(int), l_range:double, h_range:double)
		var endIdx = l_indices.size-1
		var maxldiff = 100.0
		for i=0,endIdx do
			var light = [ColorUtils.RGBtoLAB(real)](pattern(l_indices:get(i)))
			var dark = [ColorUtils.RGBtoLAB(real)](pattern(l_indices:get(i+1)))
			var ldiff = (light(0)-dark(0))/maxldiff
			var target = 15/maxldiff

			--constrain lightness
			factor(softEq(ldiff, target, l_range/maxldiff)/constraintTemp)

		end

		endIdx = h_indices.size-1
		var maxhdiff = 282.9
		for i=0,endIdx do
			var light = [ColorUtils.RGBtoLAB(real)](pattern(h_indices:get(i)))
			var dark = [ColorUtils.RGBtoLAB(real)](pattern(h_indices:get(i+1)))
			var adiff = light(1)-dark(1)
			var bdiff = light(2)-dark(2)
			var hdiff = ad.math.sqrt(adiff*adiff+bdiff*bdiff)/maxhdiff
			var target = 20/maxhdiff

			--constrain hue
			factor(softEq(hdiff, target, h_range/maxhdiff)/constraintTemp)
		end
	end

	--factor over three variables. lightness delta between two vars should be the same as the lightness delta between the other two vars. The first lightness delta can be anything. 
	--(i.e. lightness values should be equally spaced and in one direction)
	-- TODO: the hue delta between two vars should also be the same as between the other two vars (i.e. hue values should be equally spaced and in one direction)
	local terra glowConstraint2(pattern:&RealPattern, l_indices:Vector(int), h_indices:Vector(int), l_range:double, h_range:double)
		var endIdx = l_indices.size-2
		var maxldiff = 100.0
		var firstRange = 0.3
		var logisticScale = 100
		var minContrast = 0.1
		for i=0,endIdx do
			var light = [ColorUtils.RGBtoLAB(real)](pattern(l_indices:get(i)))
			var dark = [ColorUtils.RGBtoLAB(real)](pattern(l_indices:get(i+1)))
			var darkest = [ColorUtils.RGBtoLAB(real)](pattern(l_indices:get(i+2)))

			--enforce positivity
			if (i == 0) then
				var firstDiff = (light(0)-dark(0))/maxldiff
				var aboveMin = logistic((firstDiff-minContrast)*logisticScale)/constraintTemp
				factor(softEq(aboveMin,1,l_range/maxldiff))
			end

			var ldiff_0 = (light(0)-dark(0))/maxldiff
			var ldiff_1 = (dark(0)-darkest(0))/maxldiff

			--var target = ad.math.sqrt(ldiff_0*ldiff_0 - ldiff_1*ldiff_1)/maxldiff

			--constrain lightness
			factor(softEq(ldiff_1, ldiff_0, l_range/maxldiff)/constraintTemp)

		end

		endIdx = h_indices.size-1
		var maxhdiff = 282.9
		for i=0,endIdx do
			var light = [ColorUtils.RGBtoLAB(real)](pattern(h_indices:get(i)))
			var dark = [ColorUtils.RGBtoLAB(real)](pattern(h_indices:get(i+1)))
			var adiff = light(1)-dark(1)
			var bdiff = light(2)-dark(2)
			var hdiff = ad.math.sqrt(adiff*adiff+bdiff*bdiff)/maxhdiff
			var target = 20/maxhdiff

			--constrain hue
			factor(softEq(hdiff, target, h_range/maxhdiff)/constraintTemp)
		end

	end
	
	return terra()
		var numGroups = 5
		--------BIRD PATTERN
		-- var adjacencies = Vector.fromItems(Vector.fromItems(0,1), 
		-- 									Vector.fromItems(0,2), 
		-- 									Vector.fromItems(0,3), 
		-- 									Vector.fromItems(0,4), 
		-- 									Vector.fromItems(1,2), 
		-- 									Vector.fromItems(1,3), 
		-- 									Vector.fromItems(1,4),
		-- 									Vector.fromItems(2,3),
		-- 									Vector.fromItems(2,4),
		-- 									Vector.fromItems(3,4))
		-- var sizes = Vector.fromItems(0.623075, 0.04915, 0.0777, 0.09085, 0.159225)
		-- var tid = 105065 --bird pattern
		-- var backgroundId = 0

		--------FIREFLY PATTERN
		var adjacencies = Vector.fromItems(Vector.fromItems(0,1), 
											Vector.fromItems(1,2), 
											Vector.fromItems(1,4),
											Vector.fromItems(2,3),
											Vector.fromItems(2,4),
											Vector.fromItems(3,4))
		var sizes = Vector.fromItems(0.002575, 0.01575, 0.057375, 0.7566, 0.1677)
		var tid = 36751
		var backgroundId = 3
		
		var pattern = RealPattern.stackAlloc(numGroups, adjacencies, backgroundId, tid, sizes)
		m.destruct(adjacencies)

		-- Priors
		var scale = 1.0
		for i=0,numGroups do
			for c=0,3 do
				--pattern(i)(c) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				var value = gaussian(0.0, scale, {structural=false})
				pattern(i)(c) = logistic(value/scale)
			end
		end

		-- Constraints
		var lightness = 0.9*lightnessFn(&pattern) --Unary lightness might be more arbitrary
		var saturation = 1.3*saturationFn(&pattern)
		var diff = 1.4*diffFn(&pattern)
		var lightnessDiff = 0.5*lightnessDiffFn(&pattern) 

		factor(compatScale*(lightness+saturation+diff+lightnessDiff)/temp)

		--glowConstraint(&pattern, Vector.fromItems(0,1,2,3,4), Vector.fromItems(0,1,2,3), glowRange, hueRange)
		glowConstraint2(&pattern, Vector.fromItems(0,1,2,3,4), Vector.fromItems(0,1,2,3), glowRange, hueRange)

		return pattern
	end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local kernelType = HMC
--local kernelType = RandomWalk
local hmcNumSteps = 1
local numsamps = 10000
local verbose = true

if kernelType == RandomWalk then numsamps = hmcNumSteps*numsamps end
local kernel = nil
if kernelType == RandomWalk then
        kernel = RandomWalk()
else
        kernel = HMC({numSteps=hmcNumSteps})
end

local terra doInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	return [mcmc(colorCompatModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
-- Garbage collect the returned vector of samples
--    (i.e. free the memory when it is safe to do so)
local samples = m.gc(doInference())

-- Write out links to the colorlovers images
io.write("Saving samples to file...")
io.flush()
local terra ToByte(num:double)
	var value = [int](C.floor(C.fmax(C.fmin(num*255+0.5, 255), 0)))
	if (num < 0 or num > 1) then
		C.printf("%f\n", num)
	end
	if (value > 255 or value < 0) then
		C.printf("%d\n", value)
	end
	return value
end
local terra SaveToFile()
	var file_ptr = C.fopen("colorSamples.txt", "w");
	for i=0,numsamps do
		var pattern = samples(i).value
		var tid = pattern.templateId
    	C.fprintf(file_ptr, "http://www.colourlovers.com/patternPreview/%d/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x.png\n", tid,
    			  ToByte(pattern(0)(0)), ToByte(pattern(0)(1)), ToByte(pattern(0)(2)),
    			  ToByte(pattern(1)(0)), ToByte(pattern(1)(1)), ToByte(pattern(1)(2)),
    			  ToByte(pattern(2)(0)), ToByte(pattern(2)(1)), ToByte(pattern(2)(2)),		  
    			  ToByte(pattern(3)(0)), ToByte(pattern(3)(1)), ToByte(pattern(3)(2)),
    			  ToByte(pattern(4)(0)), ToByte(pattern(4)(1)), ToByte(pattern(4)(2))   			  
    	)
    end
    C.fclose(file_ptr)
end

SaveToFile()

print("done.")






