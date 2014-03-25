terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local colors = terralib.require("colors")

local staticsUtils = terralib.require("staticsUtils")
local staticsExamples = terralib.require("staticsExamples")

local C = terralib.includecstring [[
#include <stdio.h>
#include <string.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)

----------------------------------

local staticsModel = probcomp(function()
	local gravityConstant = 9.8

	local Vec2 = Vec(real, 2)
	local ForceT = staticsUtils.Force(real)
	local RigidObjectT = staticsUtils.RigidObject(real)
	local BeamT = staticsUtils.Beam(real)
	local RigidSceneT = staticsUtils.RigidScene(real)
	local Connections = staticsUtils.Connections()

	----------------------------------

	-- Toggle whether we're considering each residual independently
	--   or using the RMS of all of them
	-- (Force and Torque residuals are treated separately, though)
	local rmseRes = true
	-- Toggle whether we're normalizing residuals by the overall average
	--    external force magnitude, or by per-element total average force magnitudes.
	local overallExtFMagNorm = true
	-- Toggle diagnostic output for residuals
	local printResiduals = false

	-- Check if a beam's residuals are all less than 3 sigma from 0. If not,
	--    color the beam red to mark it as unstable.
	local unstableColor = colors.Tableau10.Red
	local checkStability = nil
	if rmseRes then
		checkStability = terra(obj: &RigidObjectT, fres: real, tres: real, fsigma: real, tsigma: real)
			var threeSigmaF = 3.0*fsigma
			var threeSigmaT = 3.0*tsigma
			var beam = [&BeamT](obj)
			var allStable = fres < threeSigmaF and tres < threeSigmaT
			if not allStable then
				beam.color = Color3d.stackAlloc([unstableColor])
			end
		end
	else
		checkStability = terra(obj: &RigidObjectT, fres: Vec2, tres: &Vector(real), fsigma: real, tsigma: real)
			var threeSigmaF = 3.0*fsigma
			var threeSigmaT = 3.0*tsigma
			var beam = [&BeamT](obj)
			var allStable = ad.math.fabs(fres(0)) < threeSigmaF and ad.math.fabs(fres(1)) < threeSigmaF
			if allStable then
				for i=0,tres.size do
					if not (ad.math.fabs(tres(i)) < threeSigmaT) then
						allStable = false
						break
					end
				end
			end
			if not allStable then
				beam.color = Color3d.stackAlloc([unstableColor])
			end
		end
	end

	-- Enforce static equilibrium of a scene given some connections
	local fresSoftness = `0.01  -- Allow deviation of 1% the average force magnitude
	local tresSoftness = `0.01
	local terra enforceStability(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection)) : {}

		-- Apply gravity to everything affected by it
		for i=0,scene.objects.size do
			var obj = scene.objects(i)
			if obj.active and obj.affectedByGravity then
				obj:applyExternalLoad(gravityConstant, obj:mass(), obj:centerOfMass())
			end
		end

		var avgForceMag = real(0.0)
		[util.optionally(overallExtFMagNorm, function() return quote
			-- Calculate average external force magnitude
			-- ***
			-- Using this, rather than per-element averages of all forces,
			--    gives more stable behavior, though I suppose it's technically
			--    'less correct'.
			-- ***    
			var nForces = 0
			for i=0,scene.objects.size do
				var obj = scene.objects(i)
				if obj.active then
					for j=0,obj.forces.size do
						avgForceMag = avgForceMag + obj.forces(j).vec:norm()
						nForces = nForces + 1
					end
				end
			end
			avgForceMag = avgForceMag / nForces
		end end)]

		-- Apply internal forces
		for i=0,connections.size do
			connections(i):applyForces()
		end

		-- Calculate residuals and apply factors
		-- (Individual residual style)
		var fres = Vec2.stackAlloc(0.0, 0.0)
		var tres = [Vector(real)].stackAlloc()
		[util.optionally(printResiduals, function() return quote
			C.printf("==============================\n")
		end end)]
		for i=0,scene.objects.size do
			if scene.objects(i).active then
				tres:clear()
				var avgFmag, avgTmag = scene.objects(i):calculateResiduals(&fres, &tres)

				-- Version 1: RMSE residuals
				[util.optionally(rmseRes, function() return quote
					-- Compute residuals from data given to us
					var fres_ = fres:norm()
					var tres_ = real(0.0)
					for j=0,tres.size do
						var trj = tres(j)
						tres_ = tres_ + trj*trj
					end
					tres_ = ad.math.sqrt(tres_ / tres.size)
					-- Normalize residuals
					[util.optionally(overallExtFMagNorm, function() return quote
						fres_ = fres_ / avgForceMag
						tres_ = tres_ / avgForceMag
					end end)]
					[util.optionally(not overallExtFMagNorm, function() return quote
						if avgFmag > 0.0 then fres_ = fres_ / avgFmag end
						if avgTmag > 0.0 then tres_ = tres_ / avgTmag end
					end end)]
					-- Visualize stable/unstable elements
					checkStability(scene.objects(i), fres_, tres_, fresSoftness, tresSoftness)
					-- Add stability factors
					factor(softeq(fres_, 0.0, fresSoftness))
					factor(softeq(tres_, 0.0, tresSoftness))
					[util.optionally(printResiduals, function() return quote
						C.printf("fres: %g, tres: %g\n", ad.val(fres_), ad.val(tres_))
					end end)]
				end end)]

				-- Version 2: Individual residuals
				[util.optionally(not rmseRes, function() return quote
					-- Divide tres's by tres.size so that elements with more torque equations
					--    aren't over-emphasized.
					for j=0,tres.size do tres(j) = tres(j) / double(tres.size) end
					-- Normalize residuals
					[util.optionally(overallExtFMagNorm, function() return quote
						fres = fres / avgForceMag
						for j=0,tres.size do tres(j) = tres(j) / avgForceMag end
					end end)]
					[util.optionally(not overallExtFMagNorm, function() return quote
						if avgFmag > 0.0 then fres = fres / avgFmag end
						if avgTmag > 0.0 then for j=0,tres.size do tres(j) = tres(j) / avgTmag end end
					end end)]
					-- Visualize stable/unstable elements
					checkStability(scene.objects(i), fres, &tres, fresSoftness, tresSoftness)
					-- Add stability factors
					factor(softeq(fres(0), 0.0, fresSoftness))
					factor(softeq(fres(1), 0.0, fresSoftness))
					for j=0,tres.size do
						factor(softeq(tres(j), 0.0, tresSoftness))
					end
				end end)]

				[util.optionally(printResiduals, function() return quote
					C.printf("---------------------\n")
				end end)]

			end
		end
		m.destruct(tres)
	end
	enforceStability = pfn(enforceStability)

	----------------------------------

	local examples = staticsExamples(gravityConstant, Connections)

	-- local example = examples.simpleNailTest
	-- local example = examples.aFrameTest
	local example = examples.funkyTable
	-- local example = examples.arch

	return terra()
		var scene, connections = example()
		enforceStability(&scene, &connections)
		connections:clearAndDelete()
		m.destruct(connections)
		return scene
	end
end)

----------------------------------

local gl = terralib.require("gl")
local rendering = terralib.require("rendering")
gl.exposeConstants({"GL_VIEWPORT", {"GLUT_BITMAP_HELVETICA_18", "void*"}})

local terra displayString(font: &opaque, str: rawstring)
	if str ~= nil and C.strlen(str) > 0 then
		while @str ~= 0 do
			gl.glutBitmapCharacter(font, @str)
			str = str + 1
		end
	end
end

local lpfont = gl.mGLUT_BITMAP_HELVETICA_18()
local lpcolor = colors.Black
local terra displayLogprob(lp: double)
	gl.glMatrixMode(gl.mGL_PROJECTION())
	gl.glLoadIdentity()
	var viewport : int[4]
	gl.glGetIntegerv(gl.mGL_VIEWPORT(), viewport)
	gl.gluOrtho2D(double(viewport[0]), double(viewport[2]), double(viewport[1]), double(viewport[3]))
	gl.glMatrixMode(gl.mGL_MODELVIEW())
	gl.glLoadIdentity()
	var str : int8[64]
	C.sprintf(str, "lp: %g", lp)
	gl.glColor3f([lpcolor])
	gl.glRasterPos2f(viewport[0] + 0.02*viewport[2], viewport[1] + 0.9*viewport[3])
	displayString(lpfont, str)
end

local imageWidth = 500
local function renderInitFn(samples, im)
	return quote
		var argc = 0
		gl.glutInit(&argc, nil)
		var scene0 = &samples(0).value
		var aspectRatio = scene0.height / scene0.width
		var imageHeight = int(aspectRatio*imageWidth)
		gl.glutInitWindowSize(imageWidth, imageWidth)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		im:resize(imageWidth, imageHeight)
	end
end

-- local forceScale = 0.2
local forceScale = 0.01
-- local forceScale = 0.0
-- local forceScale = 1.0
local function renderDrawFn(sample, im)
	return quote
		var scene = &sample.value
		gl.glClearColor(1.0, 1.0, 1.0, 1.0)
		gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
		scene:draw(forceScale)
		displayLogprob(sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

----------------------------------

local GradientAscent = terralib.require("gradientAscent")
local newton = terralib.require("prob.newtonProj")
local inf = terralib.require("prob.inference")
local trace = terralib.require("prob.trace")

local numsamps = 1000
local verbose = true
local temp = 1.0
local kernel = HMC({numSteps=1000, verbosity=0,
					relaxManifolds=true,
					temperAcceptHamiltonian=false,
					temperGuideHamiltonian=false,
					temperInitialMomentum=false})
-- local kernel = GradientAscent({stepSize=0.0001})
-- local kernel = GaussianDrift({bandwidth = function(temp) return `lerp(0.5, 3.0, temp/100.0) end})
-- local kernel = GaussianDrift({bandwidth = 1.5})
-- local kernel = RandomWalk()
local scheduleFn = macro(function(iter, currTrace)
	return quote
			-- currTrace.temperature = lerp(100.0, 1.0, iter/double(numsamps))
			-- currTrace.temperature = expinterp(100.0, 1.0, iter/double(numsamps))
			-- currTrace.temperature = 100.0
	end
end)
kernel = Schedule(kernel, scheduleFn)
local model = staticsModel
local terra doInference()
	return [mcmc(model, kernel, {numsamps=numsamps, verbose=verbose})]
	-- return [forwardSample(model, numsamps)]

	-- -- Initialization
	-- var samples = [inf.SampleVectorType(model)].stackAlloc()
	-- var currTrace : &trace.BaseTrace(double) = [trace.newTrace(model)]

	-- -- Burn in (find somewhere on the manifold)
	-- samples:push([inf.SampleType(model)].stackAlloc([inf.extractReturnValue(model)](currTrace), 0.0))
	-- currTrace = [newton.newtonPlusHMCManifoldProjection(staticsModel, {numSteps=1000}, {numsamps=50, verbose=true}, 2500)](currTrace, &samples)
	-- samples:push([inf.SampleType(model)].stackAlloc([inf.extractReturnValue(model)](currTrace), 0.0))
	
	-- -- CHMC
	-- var kernel = [HMC({numSteps=10, constrainToManifold=true, verbosity=0})()]
	-- currTrace = [inf.mcmcSample(model, {numsamps=numsamps, verbose=verbose})](currTrace, kernel, &samples)
	-- m.delete(kernel)

	-- m.delete(currTrace)
	-- return samples
end

local samples = m.gc(doInference())
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, imageWidth)









