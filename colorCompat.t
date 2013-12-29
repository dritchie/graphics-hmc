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
		var tid = 105065 --bird pattern
		var backgroundId = 0
		
		var pattern = RealPattern.stackAlloc(numGroups, adjacencies, backgroundId, tid)
		m.destruct(adjacencies)

		-- Priors
		for i=0,numGroups do
			for c=0,3 do
				pattern(i)(c) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
			end
		end

		-- Constraints
		var lightness = 0 --lightnessFn(&pattern) --Unary lightness might be more arbitrary
		var saturation = saturationFn(&pattern)
		var diff = diffFn(&pattern)
		var lightnessDiff = lightnessDiffFn(&pattern) 

		factor((lightness+saturation+diff+lightnessDiff)/temp)

		return pattern
	end
end


-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 10000
local verbose = true
local kernel = HMC({numSteps=1})	-- Defaults to trajectories of length 1
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






