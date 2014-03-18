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

	-- Check if a beam's residuals are all less than 3 sigma from 0. If not,
	--    color the beam red to mark it as unstable.
	local unstableColor = colors.Tableau10.Red
	local terra checkStability(obj: &RigidObjectT, fres: Vec2, tres: &Vector(real), fsigma: real, tsigma: real)
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

	-- Enforce static equilibrium of a scene given some connections
	local printResiduals = false
	local forceResidualRelativeErrorSD = `0.01  -- Allow deviation of 1% the average force magnitude
	local torqueResidualRelativeErrorSD = `0.01
	local terra enforceStability(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection)) : {}

		-- Apply gravity to everything affected by it
		for i=0,scene.objects.size do
			var obj = scene.objects(i)
			if obj.active and obj.affectedByGravity then
				obj:applyExternalLoad(gravityConstant, obj:mass(), obj:centerOfMass())
			end
		end

		-- Calculate the average (external) force magnitude
		-- (We'll use this to figure out the 'relative' error represented by
		--  force and torque residuals)
		var avgForceMag = real(0.0)
		var numForces = 0
		for i=0,scene.objects.size do
			for j=0,scene.objects(i).forces.size do
				avgForceMag = avgForceMag + scene.objects(i).forces(j).vec:norm()
				numForces = numForces + 1
			end
		end
		avgForceMag = avgForceMag / numForces

		var fresSoftness = forceResidualRelativeErrorSD * avgForceMag
		var tresSoftness = torqueResidualRelativeErrorSD * avgForceMag

		-- Apply internal forces
		for i=0,connections.size do
			connections(i):applyForces()
		end

		-- Calculate residuals and apply factors
		-- (Individual residual style)
		var fres = Vec2.stackAlloc(0.0, 0.0)
		var tres = [Vector(real)].stackAlloc()
		[util.optionally(printResiduals, function() return quote
			C.printf("=============================\n")
			C.printf("avgForceMag: %g\n", ad.val(avgForceMag))
			C.printf("=============================\n")
		end end)]
		for i=0,scene.objects.size do
			if scene.objects(i).active then
				tres:clear()
				scene.objects(i):calculateResiduals(&fres, &tres)
				checkStability(scene.objects(i), fres, &tres, fresSoftness, tresSoftness)
				[util.optionally(printResiduals, function() return quote
					C.printf("relative fres: (%g, %g)\n", ad.val(fres(0)/avgForceMag), ad.val(fres(1)/avgForceMag))
				end end)]
				-- -- TEST: normalize by mass
				-- var m = scene.objects(i):mass()
				-- fres = fres / m
				-- -- end TEST
				manifold(fres(0), fresSoftness)
				manifold(fres(1), fresSoftness)
				for j=0,tres.size do
					var trj = tres(j)
					[util.optionally(printResiduals, function() return quote
						C.printf("relative tres: %g\n", ad.val(trj/avgForceMag))
					end end)]
					-- -- TEST: normalize by mass
					-- trj = trj / m
					-- -- end TEST
					manifold(trj, tresSoftness)
				end
				[util.optionally(printResiduals, function() return quote
					C.printf("------------------------\n")
				end end)]
			end
		end
		m.destruct(tres)
	end
	enforceStability = pfn(enforceStability)

	----------------------------------

	local examples = staticsExamples(gravityConstant, Connections)

	-- local example = examples.simpleNailTest
	local example = examples.aFrameTest

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
local forceScale = 0.02
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









