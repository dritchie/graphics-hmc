terralib.require("prob")


local boundMag = `1000.0
local gaussVar = `1.0
-- local gaussVar = `100.0

local model = probcomp(function()
	return terra()
		return gaussian(0.0, gaussVar, {structural=false, lowerBound=-boundMag, upperBound=boundMag, initialVal=0.5*boundMag})
		-- return boundMag*gaussian(0.0, gaussVar, {structural=false, lowerBound=-1.0, upperBound=1.0, initialVal=0.5})
		-- return gaussian(0.0, gaussVar, {structural=false, initialVal=0.5*boundMag})
	end
end)

local terra domcmc()
	return [mcmc(model, HMC({numSteps=10}), {numsamps=1000, verbose=true})]
end
domcmc()