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
local templatize = terralib.require("templatize")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
]]

--init random number generator
rand.initrand()

local useRGB = ColorUtils.useRGB


local CreateFireflies = templatize(function(real)
	local RealPattern = Pattern(real)

	return terra()
		--------FIREFLY PATTERN
		var numGroups = 5
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

		return pattern
	end
end)

local GetPatternLogProb = templatize(function(real)

	local temp = 1
	local glowRange = 1--5
	local hueRange = 2
	local compatScale = 0.01

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
		var result = real(0.0)
		for i=0,endIdx do
			var light = pattern(l_indices:get(i))
			var dark = pattern(l_indices:get(i+1))
			if (useRGB) then
				light = [ColorUtils.RGBtoLAB(real)](light)
				dark = [ColorUtils.RGBtoLAB(real)](dark)
			end

			var ldiff = (light(0)-dark(0))/maxldiff
			var target = 20/maxldiff

			--constrain lightness
			--factor(softEq(ldiff, target, l_range/maxldiff))
			result = result + softEq(ldiff, target, l_range/maxldiff)

		end

		endIdx = h_indices.size-1
		var maxhdiff = 282.9
		for i=0,endIdx do
			var light = pattern(h_indices:get(i))
			var dark = pattern(h_indices:get(i+1))
			if (useRGB) then
				light = [ColorUtils.RGBtoLAB(real)](light)
				dark = [ColorUtils.RGBtoLAB(real)](dark)
			end

			var adiff = light(1)-dark(1)
			var bdiff = light(2)-dark(2)
			var hdiff = ad.math.sqrt(adiff*adiff+bdiff*bdiff)/maxhdiff
			var target = 0.0/maxhdiff

			--constrain hue
			--factor(softEq(hdiff, target, h_range/maxhdiff))
			result = result + softEq(hdiff, target, h_range/maxhdiff)
		end
		return result
	end
	

	return terra(pattern:&RealPattern)
		var lightness = 0.9*lightnessFn(pattern) --Unary lightness might be more arbitrary
		var saturation = 1.3*saturationFn(pattern)
		var diff = 1.4*diffFn(pattern)
		var lightnessDiff = 0.5*lightnessDiffFn(pattern) 

		var glowScore = glowConstraint(pattern, Vector.fromItems(0,1,2,3,4), Vector.fromItems(0,1,2), glowRange, hueRange)
		
		var score = compatScale*(lightness+saturation+diff+lightnessDiff)/temp + glowScore
		return score
	end
end)



local function colorCompatModel()
	local temp = 1
	local glowRange = 1--5
	local hueRange = 2
	local compatScale = 0.01 -- scaling compatibility to other factors...

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

		var pattern:RealPattern = [CreateFireflies(real)]()

		-- Priors
		var scale = 1.0
		for i=0,numGroups do
			for c=0,3 do
				--pattern(i)(c) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
				var value = gaussian(0.0, scale, {structural=false})
				pattern(i)(c) = logistic(value/scale)
				if (not useRGB) then
					--scale to LAB coordinates
					if (c == 0) then
						pattern(i)(c) = pattern(i)(c) * 100.0
					else
						pattern(i)(c) = pattern(i)(c)*200.0 - 100.0
					end
				end
			end
		end

		-- Constraints
		var score = [GetPatternLogProb(real)](&pattern)
		factor(score)

		return pattern
	end
end



-- Do HMC inference on the model
local hmcNumSteps = 20
local numsamps = 10000--10000
local verbose = true

local terra doHMCInference()
	-- mcmc returns Vector(Sample), where Sample has 'value' and 'logprob' fields
	--return [mcmc(colorCompatModel, kernel, {numsamps=numsamps, verbose=verbose})]
	return [mcmc(colorCompatModel,  HMC({numSteps=hmcNumSteps}) , {numsamps=numsamps, verbose=verbose})]
end

local terra doRandomInference()
	return [mcmc(colorCompatModel, RandomWalk(), {numsamps=hmcNumSteps*numsamps, verbose=verbose})]	
end

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


local function SaveToFile(name, samples)
	local outsamples = string.format("%s.txt",name)
	local outhtml = string.format("%s.html",name)
	print(outsamples)
	print(outhtml)

	local terra SaveToFile()
		var numsamps = samples.size
		var scoreThresh = 0---5
		var file_ptr = C.fopen(outsamples, "w")
		C.fprintf(file_ptr, "link,score\n")

	    var html_ptr = C.fopen(outhtml,"w")
		C.fprintf(html_ptr, "<html><head></head><body><h1>Samples with logprob above %d. No sequential duplicates</h1>\n",scoreThresh) --arbitrary threshold

		var previous = samples(0).value
		for i=0,numsamps do
			var pattern = samples(i).value
			var tid = pattern.templateId
			var colors = Vector.fromItems(pattern(0), pattern(1), pattern(2), pattern(3), pattern(4))
			if (not useRGB) then
				for c=0,5 do
					colors(c) = [ColorUtils.LABtoRGB(real)](colors(c))
				end
			end


	    	C.fprintf(file_ptr, "http://www.colourlovers.com/patternPreview/%d/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x.png,%f\n", tid,
	    			  ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)),
	    			  ToByte(colors(1)(0)), ToByte(colors(1)(1)), ToByte(colors(1)(2)),
	    			  ToByte(colors(2)(0)), ToByte(colors(2)(1)), ToByte(colors(2)(2)),		  
	    			  ToByte(colors(3)(0)), ToByte(colors(3)(1)), ToByte(colors(3)(2)),
	    			  ToByte(colors(4)(0)), ToByte(colors(4)(1)), ToByte(colors(4)(2)),
	    			  samples(i).logprob   			  
	    	)

	    	var same = true
	    	for c=0,5 do
	    		if (not(pattern(c)(0) == previous(c)(0)) or not(pattern(c)(1) == previous(c)(1)) or not(pattern(c)(2) == previous(c)(2))) then
	    			same = false
	    		end
	    	end
	    	if (samples(i).logprob > scoreThresh and not same) then  	
		    	C.fprintf(html_ptr, "<img src='http://www.colourlovers.com/patternPreview/%d/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x.png' title='%f'/>\n", tid,
		    			  ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)),
		    			  ToByte(colors(1)(0)), ToByte(colors(1)(1)), ToByte(colors(1)(2)),
		    			  ToByte(colors(2)(0)), ToByte(colors(2)(1)), ToByte(colors(2)(2)),		  
		    			  ToByte(colors(3)(0)), ToByte(colors(3)(1)), ToByte(colors(3)(2)),
		    			  ToByte(colors(4)(0)), ToByte(colors(4)(1)), ToByte(colors(4)(2)),
		    			  samples(i).logprob 			  
		    	)
	    	end
	    	previous = pattern
	    end
	    C.fclose(file_ptr)


		C.fprintf(html_ptr, "</body></html>\n")
	    C.fclose(html_ptr)
	end
	SaveToFile()
end

local function Eval(randomSamples, hmcSamples, hmcNumSteps)
	local numGroups = 5
	local dims = numGroups*3
	local SampleValue = Vec(double, dims)
	local SampleValueList = Vector(SampleValue)

	local RealPattern = Pattern(real)
	local RealPatternList = Vector(RealPattern)

	local terra ComputeMean(patternValues:&SampleValueList)
		var mean = SampleValue.stackAlloc()
		var numsamps = patternValues.size
		for i=0,numsamps do
				mean = mean + patternValues(i)
			end
		if (numsamps > 0) then
			mean = mean/numsamps
		end
		return mean
	end

	local terra ComputeVariance(patternValues:&SampleValueList, mean:SampleValue)
		--compute the variance
		var variance = 0.0
		var numsamps = patternValues.size
		for i=0,numsamps do
			var patternValue = patternValues(i)
			variance = variance + mean:distSq(patternValue)
		end
		if (numsamps > 0) then
			variance = variance/numsamps
		end
		return variance
	end 

	--Since there are such strict constraints, it's really difficult to get
	--decent scoring samples...
	local estimateNumSamps = 500000
	local terra EstimateTrueStats()
		C.printf("Estimating true stats...\n")
		--var numsamps = 1000000
		var numsamps = estimateNumSamps
		-- var samples = RealPatternList.stackAlloc()
		-- for i=0,numsamps do
		-- 	var sample = [CreateFireflies(real)]()
		-- 	for g=0,numGroups do
		-- 		sample(g)(0) = rand.random()
		-- 		sample(g)(1) = rand.random()
		-- 		sample(g)(2) = rand.random()

		-- 		if (not useRGB) then
		-- 			sample(g)(0) = sample(g)(0) * 100
		-- 			sample(g)(1) = sample(g)(1) * 200 - 100
		-- 			sample(g)(2) = sample(g)(2) * 200 - 100
		-- 		end

		-- 	end
		-- 	samples:push(sample)
		-- 	C.printf("sample:%d\r",i)
		-- end
		--var samples = [mcmc(colorCompatModel,  HMC({numSteps=hmcNumSteps}) , {numsamps=estimateNumSamps, verbose=true})]
		var samples = [mcmc(colorCompatModel,  RandomWalk() , {numsamps=estimateNumSamps, verbose=true})]

		var patternValues = SampleValueList.stackAlloc()
		for i=0,numsamps do
			--var pattern = samples(i)
			var pattern = samples(i).value
			var patternValue = SampleValue.stackAlloc()
			for g=0,numGroups do
				patternValue(3*g) = pattern(g)(0)
				patternValue(3*g+1) = pattern(g)(1)
				patternValue(3*g+2) = pattern(g)(2)
			end
			patternValues:push(patternValue)
		end

		-- var mean = SampleValue.stackAlloc()
		-- var totalScore = 0.0
		-- for i=0,numsamps do
		-- 	var score = C.exp([GetPatternLogProb(real)](&samples(i)))
		-- 	mean = mean + score*patternValues(i)
		-- 	totalScore = totalScore + score
		-- end
		-- if (totalScore > 0) then
		-- 	mean = mean/totalScore
		-- end

		-- var variance = 0.0
		-- for i=0,numsamps do
		-- 	--var logscore = [GetPatternLogProb(real)](&samples(i))
		-- 	var score = C.exp([GetPatternLogProb(real)](&samples(i)))
		-- 	variance = variance + score*(mean:distSq(patternValues(i)))
		-- 	--C.printf("dist %f score %f\n", mean:distSq(patternValues(i)), logscore)
		-- end
		-- if (totalScore > 0) then
		-- 	variance = variance/totalScore
		-- end
		var mean = ComputeMean(&patternValues)
		var variance = ComputeVariance(&patternValues, mean)

		C.printf("estimated mean:\n")
		for i=0,dims do
			C.printf("%f\t", mean(i))
		end
		C.printf("\nestimated variance: %f\n", variance)


		C.printf("done!\n")
		return mean, variance
	end

	--compute autocorrelation of samples
	local function AutoCorrelation(fname, samples, mean, variance)
		print("Computing autocorrelation...")
		local terra AutoCorrelation()
			
			var numsamps = samples.size
			var file_ptr = C.fopen(fname, "w")
			C.fprintf(file_ptr, "timeLag,autocorrelation")
			--for d=0,dims do
			--	C.fprintf(file_ptr,",dim%d",d)
			--end
			C.fprintf(file_ptr,"\n")

			var patternValues = SampleValueList.stackAlloc()

			--convert patterns to vectors
			for i=0,numsamps do
				var pattern = samples(i).value
				var patternValue = SampleValue.stackAlloc()
				for g=0,numGroups do
					patternValue(3*g) = pattern(g)(0)
					patternValue(3*g+1) = pattern(g)(1)
					patternValue(3*g+2) = pattern(g)(2)
				end
				patternValues:push(patternValue)
			end

			--compute the mean
			--var mean = stats(0)--ComputeMean(&patternValues)

			--compute the variance
			--var variance = stats(1)--ComputeVariance(&patternValues, mean)

			--compute the autocorrelation at different times t
			--write to file
			var area = 0.0
			for t=0,numsamps do
				var autoCorrelation = 0.0--SampleValue.stackAlloc()
				for i=0,(numsamps-t) do
					autoCorrelation = autoCorrelation + (patternValues(i)-mean):dot(patternValues(i+t)-mean)
					--autoCorrelation = autoCorrelation + (patternValues(i)):dot(patternValues(i+t))
				end
				if (numsamps-t > 0) then
					autoCorrelation = autoCorrelation/(numsamps-t)
					autoCorrelation = autoCorrelation/variance
					C.fprintf(file_ptr, "%d",t)
					C.fprintf(file_ptr,",%f", autoCorrelation)
					C.fprintf(file_ptr,"\n")
					area = area + C.fabs(autoCorrelation)
				end
			end

			C.fclose(file_ptr)
			C.printf("Area %f\n", area)
			C.printf("Average area %f\n", area/(numsamps))

		end

		AutoCorrelation()
		print("done!\n")

	end

	local function ESJD(samples, numSteps)
		print("Computing ESJD")
		local terra ESJD()
			var numsamps = samples.size
			var patternValues = SampleValueList.stackAlloc()

			--convert patterns to vectors
			for i=0,numsamps do
				var pattern = samples(i).value
				var patternValue = SampleValue.stackAlloc()
				for g=0,numGroups do
					--if (not(g==3)) then
						patternValue(3*g) = pattern(g)(0)
						patternValue(3*g+1) = pattern(g)(1)
						patternValue(3*g+2) = pattern(g)(2)
					--end
				end
				patternValues:push(patternValue)
			end

			var esjd = 0.0
			for t=0,(numsamps-numSteps) do
				esjd = esjd + patternValues(t):dist(patternValues(t+numSteps))
			end
			esjd = esjd/(numsamps-numSteps)
			C.printf("ESJD %f\n", esjd)
		end
		ESJD()
		print("done!\n")
	end

	--compute the variance in the samples at different score thresholds, and number of samples
	local function HighScoreVariance(randomSamples, hmcSamples)
		local terra HighScoreVariance()
			
			var file_ptr = C.fopen("scoreVariance.csv", "w")
			C.fprintf(file_ptr, "scoreThresh,type,number,variance\n")

			--convert patterns to vectors
			var randomNumSamps = randomSamples.size
			var randomPatternValues = SampleValueList.stackAlloc()
			for i=0,randomNumSamps do
				var pattern = randomSamples(i).value
				var patternValue = SampleValue.stackAlloc()
				for g=0,numGroups do
					patternValue(3*g) = pattern(g)(0)
					patternValue(3*g+1) = pattern(g)(1)
					patternValue(3*g+2) = pattern(g)(2)
				end
				randomPatternValues:push(patternValue)
			end

			var hmcNumSamps = hmcSamples.size
			var hmcPatternValues = SampleValueList.stackAlloc()

			--convert patterns to vectors
			for i=0,hmcNumSamps do
				var pattern = hmcSamples(i).value
				var patternValue = SampleValue.stackAlloc()
				for g=0,numGroups do
					patternValue(3*g) = pattern(g)(0)
					patternValue(3*g+1) = pattern(g)(1)
					patternValue(3*g+2) = pattern(g)(2)
				end
				hmcPatternValues:push(patternValue)
			end

			for thresh=-10,5 do
				var validHMCSamples = SampleValueList.stackAlloc()
				var validRandomSamples = SampleValueList.stackAlloc()
				for i=0, hmcNumSamps do
					if (hmcSamples(i).logprob > thresh) then
						validHMCSamples:push(hmcPatternValues:get(i))
					end
				end
				for i=0,randomNumSamps do
					if (randomSamples(i).logprob > thresh) then
						validRandomSamples:push(randomPatternValues:get(i))
					end
				end

				var hmcVariance = ComputeVariance(&validHMCSamples, ComputeMean(&validHMCSamples))
				var randomVariance = ComputeVariance(&validRandomSamples, ComputeMean(&validRandomSamples))
				C.fprintf(file_ptr,"%d,hmc,%d,%f\n", thresh,validHMCSamples.size, hmcVariance)
				C.fprintf(file_ptr,"%d,random,%d,%f\n", thresh,validRandomSamples.size, randomVariance)

			end

		end
		HighScoreVariance()
	end


	local terra ComputeHMCStats()
		--compute the variance
		var patternValues = SampleValueList.stackAlloc()
		var samples = hmcSamples
		var numsamps = samples.size

		--convert patterns to vectors
		for i=0,numsamps do
			var pattern = samples(i).value
			var patternValue = SampleValue.stackAlloc()
			for g=0,numGroups do
				patternValue(3*g) = pattern(g)(0)
				patternValue(3*g+1) = pattern(g)(1)
				patternValue(3*g+2) = pattern(g)(2)
			end
			patternValues:push(patternValue)
		end
		var mean = ComputeMean(&patternValues)
		var variance = ComputeVariance(&patternValues, mean)
		return mean, variance
	end

	local terra ComputeRandomStats()
		--compute the variance
		var patternValues = SampleValueList.stackAlloc()
		var samples = randomSamples
		var numsamps = samples.size

		--convert patterns to vectors
		for i=0,numsamps do
			var pattern = samples(i).value
			var patternValue = SampleValue.stackAlloc()
			for g=0,numGroups do
				patternValue(3*g) = pattern(g)(0)
				patternValue(3*g+1) = pattern(g)(1)
				patternValue(3*g+2) = pattern(g)(2)
			end
			patternValues:push(patternValue)
		end
		var mean = ComputeMean(&patternValues)
		var variance = ComputeVariance(&patternValues, mean)
		return mean, variance
	end

	local hmean, hvariance = ComputeHMCStats()
	local rmean, rvariance = ComputeRandomStats()

	local terra CombineStats(hmean:SampleValue, rmean:SampleValue, hvariance:double, rvariance:double)
		var mean = 0.5 * rmean + 0.5 * hmean
		var variance = 0.5 * rvariance + 0.5 * hvariance

		C.printf("estimated mean:\n")
		for i=0,dims do
			C.printf("%f\t", mean(i))
		end
		C.printf("\nestimated variance: %f\n", variance)

		return mean, variance
	end

	--local mean, variance = CombineStats(hmean, rmean, hvariance, rvariance)
	local mean, variance = EstimateTrueStats()

	

	

	C.printf("HMC ESJD\n")
	ESJD(hmcSamples,1)
	C.printf("Random ESJD\n")
	ESJD(randomSamples, hmcNumSteps)
	C.printf("Random ESJD step 1\n")
	ESJD(randomSamples, 1)


	C.printf("HMC:\n")
	AutoCorrelation("HMCAutoCorrelation.csv", hmcSamples, mean, variance)
	C.printf("Random:\n")
	AutoCorrelation("randomAutoCorrelation.csv", randomSamples, mean, variance)
	HighScoreVariance(randomSamples, hmcSamples)

end





-- Garbage collect the returned vector of samples
--    (i.e. free the memory when it is safe to do so)
io.write("\nHMC.\n")
io.flush()
local hmcSamples = m.gc(doHMCInference())


-- Write out links to the colorlovers images
SaveToFile("HMCColorSamples", hmcSamples)


-- io.write("Random.\n")
-- io.flush()
-- local randomSamples = m.gc(doRandomInference())
-- SaveToFile("RandomColorSamples", randomSamples)

-- Eval(randomSamples, hmcSamples, hmcNumSteps)


print("done.")






