
-- TODO: Abstract this to work anywhere, not just on statics3d/test.t

-- Parameters are:
--   * exampleToRun (default="Please specify 'exampleToRun'"): which example to run
--   * doComparison (default=true): do two runs comparing HMC to SVMH, or just one?
--   * forwardSample (default=false): true if we should just run a forward sample pass (for simple testing/debugging)
--   * doHMC (default=true): whether we should run HMC or gaussian drift MH
--   * numSamps (default=1000): how many samples to collect
--   * numHMCSteps (default=100): how many leapfrog trajectory steps to use
--   * gaussianBandwidth (default=1.0): proposal bandwidth for gaussian drift MH kernel
--   * numBurnInSamps (default=0): how many samples to burn in for
--   * saveBurnIn (default=false): whether burn-in samples are considered for further output/analysis
--   * outputdir (default=../renders): the directory where output gets written
--   * name (default="output"): the basename for all output files
--   * renderMovie (default=true): true if we should render visual output
--   * imgRes (default=500): x resolution of rendered images
--   * deleteImages (default=true): true if we should delete frames after the .mp4 is created.
--   * genAverageImg (default=false): true if we should generate an average image (requires 'renderMovie=true' and 'deleteImages=false')
--   * computeAutocorr (default=false): true if we should compute autocorrelation
--   * saveBlueprints (default=false): true if we should save construction schematics for sampled structures

local Params =
{
	-- Store defaults here
	exampleToRun = "Please specify 'exampleToRun'",
	doComparison = true,
	forwardSample = false,
	doHMC = true,
	numSamps = 1000,
	numHMCSteps = 100,
	gaussianBandwidth = 1.0,
	numBurnInSamps = 0,
	saveBurnIn = false,
	outputdir = "../renders",
	name = "output",
	renderMovie = true,
	imgRes = 500,
	deleteImages = true,
	genAverageImg = false,
	computeAutocorr = false,
	saveBlueprints = false
}
Params.__index = Params

function Params.new()
	local obj = {}
	setmetatable(obj, Params)
	return obj
end

function Params:print()
	print("Params:\n----------")
	for k,v in pairs(Params) do
		if type(v) ~= "function" and type(v) ~= "table" then
			print(k, self[k])
		end
	end
	print("----------")
end

function Params:loadFile(filename)
	local function str2bool(str)
		if str == "true" then return true
		elseif str == "false" then return false
		else error(string.format("Attempt to convert unrecognized string value to boolean; expected 'true' or 'false', got %s", str))
		end
	end
	for line in io.lines(filename) do
		-- Skip empty lines
		if line ~= "" then
			-- Tokenize
			local toks = {}
			for t in string.gmatch(line, "%S+") do table.insert(toks, t) end
			local cmd = toks[1]
			local arg = toks[2]
			-- Skip comments
			if cmd:sub(1,1) ~= "#" then
				if
					cmd == "exampleToRun" then self.exampleToRun = arg elseif
					cmd == "doComparison" then self.doComparison = str2bool(arg) elseif
					cmd == "forwardSample" then self.forwardSample = str2bool(arg) elseif
					cmd == "doHMC" then self.doHMC = str2bool(arg) elseif
					cmd == "numSamps" then self.numSamps = tonumber(arg) elseif
					cmd == "numHMCSteps" then self.numHMCSteps = tonumber(arg) elseif
					cmd == "gaussianBandwidth" then self.gaussianBandwidth = tonumber(arg) elseif
					cmd == "numBurnInSamps" then self.numBurnInSamps = tonumber(arg) elseif
					cmd == "saveBurnIn" then self.saveBurnIn = str2bool(arg) elseif
					cmd == "outputdir" then self.outputdir = arg elseif
					cmd == "name" then self.name = arg elseif
					cmd == "renderMovie" then self.renderMovie = str2bool(arg) elseif
					cmd == "imgRes" then self.imgRes = tonumber(arg) elseif
					cmd == "deleteImages" then self.deleteImages = str2bool(arg) elseif
					cmd == "genAverageImg" then self.genAverageImg = str2bool(arg) elseif
					cmd == "computeAutocorr" then self.computeAutocorr = str2bool(arg) elseif
					cmd == "saveBlueprints" then self.saveBlueprints = str2bool(arg)
				end
			end
		end
	end
	return self
end


return Params


