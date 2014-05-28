-- Include quicksand
package.path = "../?.t;" .. package.path 
local rand = terralib.require("prob.random")
local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")
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
local image = terralib.require("image")
local rendering = terralib.require("rendering")
local gl = terralib.require("gl")
local colors = terralib.require("colors")


-- C standard library stuff
local C = terralib.includecstring [[
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
]]

--init random number generator
rand.initrand()

local useRGB = ColorUtils.useRGB



local LoadPattern = templatize(function(real)
	local RealPattern = Pattern(real)
	local IntVector = Vector(int)
	local DoubleVector = Vector(double)
	local Adjacency = Vector(IntVector)
	local dirname = "imageMeshes"

	return terra(pid:int)
		----read in the pattern from file
		var filename : int8[1024]
		C.sprintf(filename, "%s/%d.txt", dirname, pid)
		var numGroups = 0

		var tid = pid
		var backgroundId = 0
		var numAdjacencies = 0
		var adjacencies = Adjacency.stackAlloc()
		var sizes = DoubleVector.stackAlloc()	
		var l_indices = Adjacency.stackAlloc()
		var h_indices = Adjacency.stackAlloc()

		var file = C.fopen(filename, "r")
		
		--read the template id, bid, the number of groups, number of adj
		C.fscanf(file, "%d\t%d\t%d\t%d", &tid, &backgroundId, &numGroups, &numAdjacencies)
		C.printf("%d %d %d %d\n", tid, backgroundId, numGroups, numAdjacencies)

		--read in the areas and adjacencies
		for i=0,numGroups do
			var size = 0.0
			C.fscanf(file, "%lf", &size)
			--C.printf("%f\n", size)
			sizes:push(size)
		end

		for i=0,numAdjacencies do
			var tuple = Vector.fromItems(0,0)
			C.fscanf(file, "%d\t%d", &tuple(0), &tuple(1))
			--C.printf("%d\t%d\n", tuple(0), tuple(1))
			adjacencies:push(tuple)
		end

		--read the constraints
		var numLGroups = 0
		C.fscanf(file, "%d", &numLGroups)
		for g=0,numLGroups do
			var numL = 0
			C.fscanf(file, "%d", &numL)
			var l_group = IntVector.stackAlloc()
			for i=0,numL do
				var idx = 0
				C.fscanf(file, "%d", &idx)
				C.printf("%d\n", idx)
				l_group:push(idx)
			end
			l_indices:push(l_group)
		end

		var numHGroups = 0
		C.fscanf(file, "%d", &numHGroups)
		for g=0,numHGroups do
			var numH = 0
			C.fscanf(file, "%d", &numH)
			var h_group = IntVector.stackAlloc()
			for i=0,numH do
				var idx = 0
				C.fscanf(file, "%d", &idx)
				C.printf("%d\n", idx)
				h_group:push(idx)
			end
			h_indices:push(h_group)
		end

		C.printf("Adjacencies %d, l_indices %d, h_indices %d, sizes %d\n", adjacencies.size, l_indices.size, h_indices.size, sizes.size)


		var pattern = RealPattern.stackAlloc(numGroups, adjacencies, backgroundId, tid, sizes, l_indices, h_indices)
		return pattern

	end
end)


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

		var l_indices = Vector.fromItems(0,1,2,3,4)
		var h_indices = Vector.fromItems(0,1,2)
			
		var pattern = RealPattern.stackAlloc(numGroups, adjacencies, backgroundId, tid, sizes, l_indices, h_indices)
		m.destruct(adjacencies)

		return pattern
	end
end)

local GetPatternLogProb = templatize(function(real)

	local temp = 1
	local glowRange = 5--5--1--5
	local hueRange = 5--5--2
	local adjustmentFactor = 100

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


	local terra glowConstraint(pattern:&RealPattern, l_indices:Vector(Vector(int)), h_indices:Vector(Vector(int)), l_range:double, h_range:double)
		var result = real(0.0)
		--return result
		
		var gIdx = l_indices.size
		for g=0,gIdx do
			var endIdx = l_indices(g).size-1
			var maxldiff = 100.0
			
			for i=0,endIdx do
				var light = pattern(l_indices(g):get(i))
				var dark = pattern(l_indices(g):get(i+1))
				-- if (useRGB) then
				-- 	light = [ColorUtils.RGBtoLAB(real)](light)
				-- 	dark = [ColorUtils.RGBtoLAB(real)](dark)
				-- end

				var ldiff = (light(0)-dark(0))/maxldiff
				-- var target = 20/maxldiff
				var target = 15/maxldiff

				--constrain lightness
				result = result + softEq(ldiff, target, l_range/maxldiff)

			end
		end

		gIdx = h_indices.size

		for g=0,gIdx do
			var endIdx = h_indices(g).size-1
			var maxhdiff = 282.9
			for i=0,endIdx do
				var light = pattern(h_indices:get(g):get(i))
				var dark = pattern(h_indices:get(g):get(i+1))
				-- if (useRGB) then
				-- 	light = [ColorUtils.RGBtoLAB(real)](light)
				-- 	dark = [ColorUtils.RGBtoLAB(real)](dark)
				-- end

				var adiff = light(1)-dark(1)
				var bdiff = light(2)-dark(2)
				var hdiff = ad.math.sqrt(adiff*adiff+bdiff*bdiff)/maxhdiff
				var target = 0.0/maxhdiff--0.0/maxhdiff

				--constrain hue
				result = result + softEq(hdiff, target, h_range/maxhdiff)

				-- var hdiff2 = (light(1)*dark(1) + light(2)*dark(2))
				-- var lightMag = light(1)*light(1)+light(2)*light(2)
				-- var darkMag = dark(1)*dark(1)+dark(2)*dark(2)
				-- if (lightMag > 0 and darkMag > 0) then
				-- 	hdiff2 = hdiff2/(lightMag*darkMag)
				-- else
				-- 	hdiff2 = 0.0
				-- end


				-- result = result + softEq(hdiff2, 1.0, 0.2)

				-- var ldiff = light(0)-dark(0)
				-- var hdiff3 = (adiff*adiff + bdiff*bdiff)/(adiff*adiff+bdiff*bdiff+ldiff*ldiff)
				-- result = result + softEq(hdiff3, 0.0, 0.1)

			end 
			-- var totalhdiff = 20.0/maxhdiff * endIdx
			-- --also add a restriction on the first hue to the last hue to encourage one direction
			-- var light = pattern(h_indices:get(g):get(0))
			-- var dark = pattern(h_indices:get(g):get(endIdx))
			-- if (useRGB) then
			-- 	light = [ColorUtils.RGBtoLAB(real)](light)
			-- 	dark = [ColorUtils.RGBtoLAB(real)](dark)
			-- end
			-- var adiff = light(1)-dark(1)
			-- var bdiff = light(2)-dark(2)
			-- var hdiff = ad.math.sqrt(adiff*adiff+bdiff*bdiff)/maxhdiff
			-- result = result + softEq(hdiff, totalhdiff, h_range/maxhdiff)

		end
		return result
	end
	

	return terra(pattern:&RealPattern)
		var lightness = 0.0--0.9*lightnessFn(pattern) --Unary lightness might be more arbitrary
		var saturation = saturationFn(pattern)--1.3*saturationFn(pattern)
		var diff = diffFn(pattern)--1.4*diffFn(pattern)
		var lightnessDiff = lightnessDiffFn(pattern) --0.5*lightnessDiffFn(pattern) 

		--Vector.fromItems(0,1,2,3,4), Vector.fromItems(0,1,2)
		var glowScore = glowConstraint(pattern, pattern.l_indices, pattern.h_indices, glowRange, hueRange)
		
		var score = adjustmentFactor* ((lightness+saturation+diff+lightnessDiff) + glowScore)
		return score
	end
end)

local function GetModel(curPattern)

	local function colorCompatModel()
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
			var numGroups = curPattern.numGroups
			var pattern:RealPattern = RealPattern.stackAlloc(curPattern.numGroups, curPattern.adjacencies, curPattern.backgroundId, curPattern.templateId, curPattern.sizes, curPattern.l_indices, curPattern.h_indices)

			-- Priors
			var scale = 1.0
			for i=0,numGroups do
				for c=0,3 do
					--pattern(i)(c) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
					--var value = gaussian(0.0, scale, {structural=false})
					--pattern(i)(c) = logistic(value/scale)
					 pattern(i)(c) = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})
					--pattern(i)(c) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
					if (not useRGB) then
						--scale to LAB coordinates
						if (c == 0) then
							pattern(i)(c) = pattern(i)(c) * 100.0
						else
							pattern(i)(c) = pattern(i)(c)*200.0 - 100.0
						end
					end
				end
				-- Convert to LAB, for scoring
				if (useRGB) then
					pattern(i) = [ColorUtils.RGBtoLAB(real)](pattern(i))
				end
			end

			-- Constraints
			var score = [GetPatternLogProb(real)](&pattern)
			factor(score)

			return pattern
		end

	end

	return colorCompatModel
end







local terra ToByte(num:double)
	var value = [int](C.floor(C.fmax(C.fmin(num*255+0.5, 255), 0)))
	return value
end

local function SaveToFile(name, samples, curPattern)
	local outsamples = string.format("%s.txt",name)
	local outhtml = string.format("%s.html",name)
	local ColorVector = Vector(Vec(real, 3))
	print(outsamples)
	print(outhtml)
	print(curPattern.templateId)

	local saturationFn = ColorUtils.BGSaturation(real)
				

	local terra SaveToFile()
		var oobCount = 0
		var numsamps = samples.size
		--base the threshold on the number of hue constraints...0 when there are 2 constraints, each constraint asdds about 4 (lightness is about 3.7)
		var numConstraints = 0
		for i=0,curPattern.h_indices.size do
			numConstraints = numConstraints + curPattern.h_indices(i).size-1
		end
		for i=0,curPattern.l_indices.size do
			numConstraints = numConstraints + curPattern.l_indices(i).size-1
		end
		var scoreThresh = -24 + 4*numConstraints--0---5
		var file_ptr = C.fopen(outsamples, "w")
		C.fprintf(file_ptr, "link,score\n")

	    var html_ptr = C.fopen(outhtml,"w")
		C.fprintf(html_ptr, "<html><head></head><body><h1>Samples with logprob above %d. No sequential duplicates</h1>\n",scoreThresh) --arbitrary threshold

		C.printf("Sample id %d numsamps %d\n", samples(0).value.templateId, numsamps)
		var previous = samples(0).value
		for i=0,numsamps do
			var pattern = samples(i).value
			var tid = pattern.templateId

			var colors = ColorVector.stackAlloc()--Vector.fromItems(pattern(0), pattern(1), pattern(2), pattern(3), pattern(4))
			for c=0, curPattern.numGroups do
				colors:push(pattern(c))
			end
			if (not useRGB) then
				var oobBool = false
				for c=0,curPattern.numGroups do
					--check if out of bounds
					var oob = [ColorUtils.LABtoRGBNoClamp(real)](colors(c))

					for b=0,3 do
						if (oob(b) < 0 or oob(b) > 1) then
							--C.printf("Out of bounds!\n")
							oobBool = true
							break
						end
					end				
					colors(c) = [ColorUtils.LABtoRGB(real)](colors(c))
				end
				if (oobBool) then
					oobCount = oobCount + 1
				end
			end


	    	-- C.fprintf(file_ptr, "http://www.colourlovers.com/patternPreview/%d/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x.png,%f\n", tid,
	    	-- 		  ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)),
	    	-- 		  ToByte(colors(1)(0)), ToByte(colors(1)(1)), ToByte(colors(1)(2)),
	    	-- 		  ToByte(colors(2)(0)), ToByte(colors(2)(1)), ToByte(colors(2)(2)),		  
	    	-- 		  ToByte(colors(3)(0)), ToByte(colors(3)(1)), ToByte(colors(3)(2)),
	    	-- 		  ToByte(colors(4)(0)), ToByte(colors(4)(1)), ToByte(colors(4)(2)),
	    	-- 		  samples(i).logprob   			  
	    	-- )
			-- C.fprintf(file_ptr, "<div style='width:50px;height:50px;background-color:#%02x%02x%02x;float:left;color:#%02x%02x%02x' >XXXX</div>,%f\n", ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)), samples(i).logprob,samples(i).logprob)
			var satScore = saturationFn(colors(curPattern.backgroundId))
			for c=0,curPattern.numGroups do
				C.fprintf(file_ptr,"#%02x%02x%02x",ToByte(colors(c)(0)), ToByte(colors(c)(1)), ToByte(colors(c)(2)))
				if (c < curPattern.numGroups-1) then
					C.fprintf(file_ptr,"+")
				else
					--C.fprintf(file_ptr,",%f\n", samples(i).logprob)
					C.fprintf(file_ptr,",%f,%f\n", samples(i).logprob,satScore)
				end
			end

	   --  	var same = true
	   --  	for c=0,curPattern.numGroups do
	   --  		if (not(pattern(c)(0) == previous(c)(0)) or not(pattern(c)(1) == previous(c)(1)) or not(pattern(c)(2) == previous(c)(2))) then
	   --  			same = false
	   --  		end
	   --  	end
	   --  	if (samples(i).logprob > scoreThresh and not same) then  	
		  --   	C.fprintf(html_ptr, "<img src='http://www.colourlovers.com/patternPreview/%d/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x/%02x%02x%02x.png' title='%f'/>\n", tid,
		  --   			  ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)),
		  --   			  ToByte(colors(1)(0)), ToByte(colors(1)(1)), ToByte(colors(1)(2)),
		  --   			  ToByte(colors(2)(0)), ToByte(colors(2)(1)), ToByte(colors(2)(2)),		  
		  --   			  ToByte(colors(3)(0)), ToByte(colors(3)(1)), ToByte(colors(3)(2)),
		  --   			  ToByte(colors(4)(0)), ToByte(colors(4)(1)), ToByte(colors(4)(2)),
		  --   			  samples(i).logprob 			  
		  --   	)
				-- --C.fprintf(file_ptr, "<div style='width:50px;height:50px;background-color:#%02x%02x%02x;color:#%02x%02x%02x' >XXXX</div>,%f\n", ToByte(colors(0)(0)), ToByte(colors(0)(1)), ToByte(colors(0)(2)), samples(i).logprob, samples(i).logprob)
	   --  	end
	   --  	previous = pattern
	    end
	    C.printf("Out of bounds %d\n", oobCount)
	    C.fclose(file_ptr)


		-- C.fprintf(html_ptr, "</body></html>\n")
	    -- C.fclose(html_ptr)
	end
	SaveToFile()
end

local function Eval(randomSamples, hmcSamples, hmcNumSteps, pid, curPattern)
	local numGroups = curPattern.numGroups
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
	local terra SampleValues()
		C.printf("Estimating true stats...\n")
		var numsamps = estimateNumSamps
		var samples = [mcmc(GetModel(curPattern),  RandomWalk() , {numsamps=estimateNumSamps, verbose=true})]

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
		return patternValues
	end


	-- compute autocorrelation of samples
	local function AutoCorrelation(fname, samples, pid)
		print("Computing autocorrelation...")

		local terra AutoCorrelation()
			var numsamps = samples.size
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

			var hmcValues = SampleValueList.stackAlloc()

			--Get the hmc samples as vectors
			for i=0,numsamps do
				var pattern = hmcSamples(i).value
				var patternValue = SampleValue.stackAlloc()
				for g=0,numGroups do
					patternValue(3*g) = pattern(g)(0)
					patternValue(3*g+1) = pattern(g)(1)
					patternValue(3*g+2) = pattern(g)(2)
				end
				hmcValues:push(patternValue)
			end

			var trueMean = expectation(hmcValues)
			var trueVar = variance(hmcValues, trueMean)


			var ac = autocorrelation(patternValues, trueMean, trueVar)
			var buf : int8[1024]
			C.sprintf(buf, "%s",  fname)
			saveAutocorrelation(&ac, buf)
			m.destruct(patternValues)
			m.destruct(ac)
			m.destruct(hmcValues)

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
	local function HighScoreVariance(randomSamples, hmcSamples, pid)
		local fname = string.format("%d_scoreVariance.csv", pid)
		local terra HighScoreVariance()
			
			var file_ptr = C.fopen(fname, "w")
			C.fprintf(file_ptr, "tid,scoreThresh,type,number,variance\n")

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
				C.fprintf(file_ptr,"%d,%d,hmc,%d,%f\n", pid,thresh,validHMCSamples.size, hmcVariance)
				C.fprintf(file_ptr,"%d,%d,random,%d,%f\n", pid,thresh,validRandomSamples.size, randomVariance)

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

	--local hmean, hvariance = ComputeHMCStats()
	--local rmean, rvariance = ComputeRandomStats()

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
	--local mean, variance = EstimateTrueStats()
	-- local patternValues = SampleValues()

	C.printf("HMC ESJD\n")
	ESJD(hmcSamples,1)
	C.printf("Random ESJD\n")
	ESJD(randomSamples, 1)

	util.wait(string.format("mkdir Stats"))
	C.printf("HMC:\n")
	AutoCorrelation(string.format("Stats/%d_HMCAutoCorrelation.csv",pid), hmcSamples, pid)
	C.printf("Random:\n")
	AutoCorrelation(string.format("Stats/%d_randomAutoCorrelation.csv", pid), randomSamples, pid)
	HighScoreVariance(randomSamples, hmcSamples, pid)

end

-------------------------------------
local function renderInitFn(samples, im)
	local dirname = "imageMeshes"
	local RGBImage = image.Image(uint8, 3)
	return quote
		--get image dimensions
		var pattern = &samples(0).value
		var filename : int8[1024]
		C.sprintf(filename, "%s/%d_template.png", dirname, pattern.templateId)
		var template = RGBImage.load(image.Format.PNG, filename)

		--init opengl for writing log prob
		-- var argc = 0
		-- gl.glutInit(&argc, nil)
		-- gl.glutInitWindowSize(template.width, template.height)
		-- gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		-- gl.glutCreateWindow("Render")
		-- gl.glViewport(0, 0, template.width, template.height)
		im:resize(template.width, template.height)
	end
end

local function renderDrawFn(sample, im, idx)
	local dirname = "imageMeshes"
	local RGBImage = image.Image(uint8, 3)
	return quote
		var pattern = &sample.value

		--load the image...
		var filename : int8[1024]
		C.sprintf(filename, "%s/%d_template.png", dirname, pattern.templateId)
		var template = RGBImage.load(image.Format.PNG, filename)

		for i=0,template.width do
			for j=0,template.height do
				var num = template(i,j)(0)
				im(i,j) = pattern(num)*255
			end
		end

		-- gl.glDrawPixels(im.width,im.height,gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
		-- gl.glFlush()
		-- [rendering.displaySampleInfo("TopLeft")](idx, sample.logprob)

		-- gl.glFlush()
		-- gl.glReadPixels(0, 0, im.width, im.height, gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end


---------------------------------START SCRIPT

print("script")
-- Do HMC inference on the model
local hmcNumSteps = 100
local numsamps = 1000
local verbose = true
local burnInSamps = 100
local hmcLag = 1
local hmcKernel = HMC({numSteps = hmcNumSteps})
local randomKernel = GaussianDrift({bandwidthAdapt=true})


-- Pattern ids to process, id correspondences in comments below
local ids = {777}
--house: 40053
--bug: 809
--robot: 7080
--rocket: 777



local function HMCInference(curPattern)
	local lag = 2*hmcNumSteps

	local model = probcomp(function()
		return GetModel(curPattern)()
	end)

	return terra()
	 	var x = [mcmc(GetModel(curPattern),  HMC({numSteps=hmcNumSteps}) , {numsamps=numsamps, lag=hmcLag, verbose=verbose})]()
	 	return x
	 	-- C.printf("HMC\n")
	 	
	 -- 	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]
		-- var samps = [SampleVectorType(model)].stackAlloc()
		-- C.printf("burn in\n")
		-- -- Burn in using SSMH
		-- var burnInKern = [randomKernel()]
		-- var burnInSamps : &SampleVectorType(model) = [(`nil)]
		-- currTrace = [inf.mcmcSample(model, {numsamps=numsamps, lag=lag, verbose=true})](currTrace, burnInKern, burnInSamps)
		-- m.delete(burnInKern)
		-- C.printf("mix\n")
		-- -- Continue mixing using whatever kernel we asked for
		-- var mixKern = [hmcKernel()]
		-- currTrace = [inf.mcmcSample(model, {numsamps=numsamps, verbose=true})](currTrace, mixKern, &samps)
		-- m.delete(mixKern)
		-- m.delete(currTrace)
		-- return samps
	end

end

local function RandomInference(curPattern)
	local lag = 2*hmcNumSteps*hmcLag

	local model = probcomp(function()
		return GetModel(curPattern)()
	end)

	 return terra()	 	
	 	return [mcmc(GetModel(curPattern), GaussianDrift({bandwidthAdapt=true}), {numsamps=numsamps, lag=lag, verbose=verbose})]()
		-- C.printf("Random\n")
	 -- 	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(testcomp)]
		-- var samps = [SampleVectorType(model)].stackAlloc()
		
		-- -- Burn in using SSMH
		-- C.printf("burn in\n")
		-- var burnInKern = [randomKernel()]
		-- var burnInSamps : &SampleVectorType(model) = [(`nil)]
		-- currTrace = [inf.mcmcSample(model, {numsamps=burnInSamps, lag=lag, verbose=true})](currTrace, burnInKern, burnInSamps)
		-- m.delete(burnInKern)
		
		-- -- Continue mixing using whatever kernel we asked for
		-- C.printf("mix\n")
		-- var mixKern = [randomKernel()]
		-- currTrace = [inf.mcmcSample(model, {numsamps=numsamps, lag=lag, verbose=true})](currTrace, mixKern, &samps)
		-- m.delete(mixKern)
		-- m.delete(currTrace)
		-- return samps
	 end
end

local function ConvertBackToRGB(samples)
	return terra()
 		for i=0,numsamps do
			var pattern = samples(i).value
			for g=0,pattern.numGroups do
				--convert back to RGB after converting to LAB for scoring
				pattern(g) = [ColorUtils.LABtoRGB(real)](pattern(g))
			end
			samples(i).value = pattern
		end
	end
end



for pid=1,#ids do
	print ("in loop")
	patternId = ids[pid]
	local curPattern = LoadPattern(real)(patternId)
	local hmcfn = HMCInference(curPattern)
	local randfn = RandomInference(curPattern)

	print (curPattern.templateId)
	print(curPattern.numGroups)

	io.write("\nHMC.\n")
	io.flush()
	local hmcSamples = m.gc(hmcfn())--m.gc(HMCInference(curPattern)())

	print("doneHMC")
	if (useRGB) then
		ConvertBackToRGB(hmcSamples)()
	end

	moviename = arg[1] or "movie-hmc"
	rendering.renderSamples(hmcSamples, renderInitFn, renderDrawFn, moviename, "renders", true)

	-- Write out links to the colorlovers images
	SaveToFile(string.format("Stats/%d_HMCColorSamples", patternId), hmcSamples, curPattern)

	io.write("Random.\n")
	io.flush()
	local randomSamples = m.gc(randfn())
	if (useRGB) then
		ConvertBackToRGB(randomSamples)()
	end

	moviename = arg[2] or "movie-rand"
	rendering.renderSamples(randomSamples, renderInitFn, renderDrawFn, moviename, "renders", true)

	SaveToFile(string.format("Stats/%d_RandomColorSamples", patternId), randomSamples, curPattern)

	-- Autocorrelation
	Eval(randomSamples, hmcSamples, hmcNumSteps, patternId, curPattern)

	m.gc(hmcSamples)
	m.gc(randomSamples)

	hmcSamples:__destruct()
	randomSamples:__destruct()


	print("done.")

end


print("done.")






