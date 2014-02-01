local inference = terralib.require("prob.inference")
local MCMCKernel = inference.MCMCKernel
local m = terralib.require("mem")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local util = terralib.require("util")
local inheritance = terralib.require("inheritance")
local templatize = terralib.require("templatize")
local trace = terralib.require("prob.trace")
local BaseTrace = trace.BaseTrace
local BaseTraceD = BaseTrace(double)
local BaseTraceAD = BaseTrace(ad.num)
local erph = terralib.require("prob.erph")
local RandVar = erph.RandVar

local C = terralib.includecstring [[
#include <stdio.h>
]]


-- Gradient ascent kernel
-- (Not actually a proper MCMC kernel, but useful as a point of comparison)
local GradientAscentKernel = templatize(function(stepSize)
	local struct GradientAscentKernelT
	{
		adTrace: &BaseTraceAD,
		adVars: Vector(&RandVar(ad.num)),
		adPositions: Vector(ad.num),
		positions: Vector(double),
		gradient: Vector(double)
	}
	inheritance.dynamicExtend(MCMCKernel, GradientAscentKernelT)

	terra GradientAscentKernelT:__construct()
		self.adTrace = nil
	end

	terra GradientAscentKernelT:__destruct() : {}
		m.delete(self.adTrace)
		m.destruct(self.adVars)
		m.destruct(self.adPositions)
		m.destruct(self.positions)
		m.destruct(self.gradient)
	end
	inheritance.virtual(GradientAscentKernelT, "__destruct")

	terra GradientAscentKernelT:init(trace: &BaseTraceD)
		self.adTrace = [BaseTraceD.deepcopy(ad.num)](trace)
		self.adVars = self.adTrace:freeVars(false, true)
		self.adPositions = [Vector(ad.num)].stackAlloc()
		for i=0,self.adVars.size do
			self.adVars(i):getRealComponents(&self.adPositions)
		end
		self.positions = [Vector(double)].stackAlloc(self.adPositions.size, 0.0)
		for i=0,self.positions.size do
			self.positions(i) = ad.val(self.adPositions(i))
		end
		self.gradient = [Vector(double)].stackAlloc(self.positions.size, 0.0)
	end

	local copyNonstructRealsIntoTrace = templatize(function(realType)
			return terra (reals: &Vector(realType), trace: &BaseTrace(realType))
			var index = 0U
			var currVars = trace:freeVars(false, true)
			for i=0,currVars.size do
				currVars:get(i):setRealComponents(reals, &index)
			end
			m.destruct(currVars)
		end
	end)

	terra GradientAscentKernelT:next(currTrace: &BaseTraceD) : &BaseTraceD
		if self.adTrace == nil then
			self:init(currTrace)
		end

		for i=0,self.positions.size do
			self.adPositions(i) = self.positions(i)
		end
		[copyNonstructRealsIntoTrace(ad.num)](&self.adPositions, self.adTrace)
		[trace.traceUpdate({structureChange=false})](self.adTrace)
		self.adTrace.logprob:grad(&self.adPositions, &self.gradient)

		-- Gradient step
		for i=0,self.positions.size do
			self.positions(i) = self.positions(i) + stepSize*self.gradient(i)
		end

		[copyNonstructRealsIntoTrace(double)](&self.positions, currTrace)
		[trace.traceUpdate{structureChange=false, factorEval=false}](currTrace)

		-- C.printf("%g                    \n", currTrace.logprob)

		return currTrace
	end
	inheritance.virtual(GradientAscentKernelT, "next")

	terra GradientAscentKernelT:name() : rawstring return [GradientAscentKernelT.name] end
	inheritance.virtual(GradientAscentKernelT, "name")

	terra GradientAscentKernelT:stats() : {}
	end
	inheritance.virtual(GradientAscentKernelT, "stats")

	m.addConstructors(GradientAscentKernelT)
	return GradientAscentKernelT
end)



local GradientAscent = util.fnWithDefaultArgs(function(...)
	local GradientAscentKernelT = GradientAscentKernel(...)
	return function() return `GradientAscentKernelT.heapAlloc() end
end,
{{"stepSize", 0.01}})


return GradientAscent





