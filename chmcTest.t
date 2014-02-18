terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local spec = terralib.require("prob.specialize")
local newton = terralib.require("prob.newtonProj")
local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")
local ad = terralib.require("ad")

local C = terralib.includecstring [[
#include <stdio.h>
]]

-- 2D Gaussian constrained to simple 1D constant manifold
local xmean = `2.0
local xsd = `1.0
local ymean = `1.0
local ysd = `0.5
local simpleGaussianModel = spec.probcomp(function()
	return terra()
		var x = gaussian(xmean, xsd, {structural=false})
		var y = gaussian(ymean, ysd, {structural=false})
		-- Constrain y to be ymean
		manifold(y - ymean, 0.0001)
		return x
		-- return y
	end
end)

-- More complicated Gaussian model taken from the CHMC paper
local means = {`0.0, `0.0, `0.0, `0.0}
local sds = {`1.0, `1.0, `0.01, `0.01}
local complexGaussianModel = spec.probcomp(function()
	local VecT = Vec(real, 4)
	return terra()
		var x = VecT.stackAlloc(gaussian([means[1]], [sds[1]], {structural=false}),
								gaussian([means[2]], [sds[2]], {structural=false}),
								gaussian([means[3]], [sds[3]], {structural=false}),
								gaussian([means[4]], [sds[4]], {structural=false}))
		manifold(x(0) + x(1) + x(2) + x(3), 0.0001)
		manifold(x(0) + x(1) - x(2) + x(3), 0.0001)
		return x
	end
end)

-- model = simpleGaussianModel
model = complexGaussianModel

local terra doInference()
	-- Initialization
	var samples = [inf.SampleVectorType(model)].stackAlloc()
	var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]
	var initialState = [inf.extractReturnValue(model)](currTrace)
	C.printf("Initial state: "); initialState:print(); C.printf("\n")

	-- Burn in (find somewhere on the manifold)
	currTrace = [newton.newtonPlusHMCManifoldProjection(model, {numSteps=1000}, {numsamps=50, verbose=true})](currTrace, &samples)
	var burnedInState = [inf.extractReturnValue(model)](currTrace)
	C.printf("Burned-in state: "); burnedInState:print(); C.printf("\n")
	
	-- CHMC
	var kernel = [HMC({numSteps=10, constrainToManifold=true, verbosity=0})()]
	-- var kernel = [HMC({numSteps=10, constrainToManifold=true, verbosity=0, stepSizeAdapt=false, stepSize=0.015})()] --0.00025
	currTrace = [inf.mcmcSample(model, {numsamps=1000, burnin=100, verbose=true})](currTrace, kernel, &samples)
	m.delete(kernel)

	m.delete(currTrace)
	var E = expectation(samples)
	m.destruct(samples)

	-- var target = xmean
	-- var target = ymean
	var target = [Vec(double, 4)].stackAlloc([means])

	var thresh = 0.07
	-- C.printf("E: %g\n", E)
	-- C.printf("err: %g\n", ad.math.fabs(E - target))
	-- util.assert(ad.math.fabs(E - target) < thresh, "Expected value was %g, should have been %g\n", E, target)
	C.printf("E: "); E:print(); C.printf("\n")
	C.printf("err: %g\n", (E - target):norm())

end
doInference()