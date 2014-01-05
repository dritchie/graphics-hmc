-- Include quicksand
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
	local temp = 0.01
	local RealPattern = Pattern(real)
	
	local lightnessFn = ColorUtils.UnaryLightnessConstraint(real)
	local saturationFn = ColorUtils.UnarySaturationConstraint(real)
	local diffFn = ColorUtils.BinaryPerceptualConstraint(real)
	local lightnessDiffFn = ColorUtils.BinaryLightnessConstraint(real)

	local logistic = macro(function(x)
                return `1.0 / (1.0 + ad.math.exp(-x))
    	end)
	
	return terra()
		var numGroups = 5
		var adjacencies = Vector.fromItems(Vector.fromItems(0,1), 
											Vector.fromItems(0,2), 
											Vector.fromItems(0,3), 
											Vector.fromItems(0,4), 
											Vector.fromItems(1,2), 
											Vector.fromItems(1,3), 
											Vector.fromItems(1,4),
											Vector.fromItems(2,3),
											Vector.fromItems(2,4),
											Vector.fromItems(3,4))
		var sizes = Vector.fromItems(0.623075, 0.04915, 0.0777, 0.09085, 0.159225)
		var tid = 105065 --bird pattern
		var backgroundId = 0
		
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

		factor((lightness+saturation+diff+lightnessDiff)/temp)

		return pattern
	end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 10000
local verbose = true
local kernel = HMC({numSteps=20})	-- Defaults to trajectories of length 1
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






