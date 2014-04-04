terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local inheritance = terralib.require("inheritance")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local colors = terralib.require("colors")

local staticsUtils = terralib.require("staticsUtils")
local staticsExamples = terralib.require("staticsExamples")

local lpsolve = terralib.require("lpsolve")
local HashMap = terralib.require("hashmap")

local C = terralib.includecstring [[
#include <stdio.h>
#include <string.h>
inline void flush() { fflush(stdout); }
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)

local lerp = macro(function(a, b, t)
	return `(1.0-t)*a + t*b
end)

----------------------------------

local gravityConstant = 9.8
local staticsModel = probcomp(function()
	local Vec2 = Vec(real, 2)
	local ForceT = staticsUtils.Force(real)
	local RigidObjectT = staticsUtils.RigidObject(real)
	local BeamT = staticsUtils.Beam(real)
	local RigidSceneT = staticsUtils.RigidScene(real)
	local Connections = staticsUtils.Connections()

	----------------------------------

	--- Test for stability exactly by solving the static equilibrium LP ---
	-- (We only run this test for non-ad traces, i.e. the endpoint of an HMC trajectory)
	-- NOTE: This currently assumes that we only have FrictionalContacts and that the only
	--    external force is gravity
	local isExactlyStable = nil
	if real == double then
		-- We identify each contact by its position in the connections array.
		--    The compressive force variable will be numbered 1 + 2*index
		--    The friction force variable will be numbered 1 + 2*index + 1
		--    (The 1 + is because LP_SOLVE is 1-based internally)
		local compressionVarIDForContact = macro(function(contactID)
			return `1 + 2*contactID
		end)
		local frictionVarIDForContact = macro(function(contactID)
			return `1 + 2*contactID + 1
		end)
		-- The sign of the force variable flips depending on the direction
		--    of the contact normal and the object under examination
		local fsign = macro(function(obj, contact)
			return quote
				var sgn = 1.0
				if contact.contactNormal:dot(obj:centerOfMass() - contact.contactPoint) < 0.0 then
					sgn = -1.0
				end
			in
				sgn
			end
		end)
		isExactlyStable = terra(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection))
			-- Assume all connections are frictional contacts, so two DoFs per connection
			var numVars = 2*connections.size

			-- Build a map from objects to their index in the scene list
			var obj2index = [HashMap(&RigidObjectT, uint)].stackAlloc()
			for i=0,scene.objects.size do
				obj2index:put(scene.objects(i), i)
			end

			-- Build a map from objects to the IDs of the contacts that involve them
			var obj2contact = [Vector(Vector(uint))].stackAlloc(scene.objects.size)
			for i=0,connections.size do
				-- util.assert([inheritance.isInstanceOf(Connections.FrictionalContact)](connections(i)),
				-- 	"isExactlyStable currently requires all connections to be FrictionalContacts\n")
				var contact = [&Connections.FrictionalContact](connections(i))
				obj2contact(obj2index(contact.objs[0])):push(i)
				obj2contact(obj2index(contact.objs[1])):push(i)
			end

			-- *** BUILD THE LP *** --

			-- Initialize
			var lp = lpsolve.make_lp(0, numVars)
			lpsolve.set_verbose(lp, 0)

			-- Constrain all the friction forces to be within the friction cone
			var indices = [Vector(int)].stackAlloc()
			var coeffs = [Vector(double)].stackAlloc()
			indices:resize(2)
			coeffs:resize(2)
			for i=0,connections.size do
				var contact = [&Connections.FrictionalContact](connections(i))
				-- Variables have a default lower bound of 0, but friction forces can go negative
				-- We have to explicitly say that friction force variables are unbounded
				lpsolve.set_unbounded(lp, frictionVarIDForContact(i))
				-- Now we introduce the two friction cone constraints:
				-- frictionForce >= -frictionCoeff*compressiveForce
				--   --> frictionForce + frictionCoeff*compressiveForce >= 0
				indices(0) = frictionVarIDForContact(i)
				indices(1) = compressionVarIDForContact(i)
				coeffs(0) = 1.0
				coeffs(1) = contact.frictionCoeff
				lpsolve.add_constraintex(lp, 2, &coeffs(0), &indices(0), lpsolve.GE, 0.0)
				-- frictionForce <= frictionCoeff*compressiveForce
				--   --> frictionForce - frictionCoeff*compressiveForce <= 0
				indices(0) = frictionVarIDForContact(i)
				indices(1) = compressionVarIDForContact(i)
				coeffs(0) = 1.0
				coeffs(1) = -contact.frictionCoeff
				lpsolve.add_constraintex(lp, 2, &coeffs(0), &indices(0), lpsolve.LE, 0.0)
			end

			-- Use equality constraints to enforce equilibrium
			for i=0,scene.objects.size do
				var obj = scene.objects(i)
				if obj.active then
					var gravityForce = 0.0
					if obj.affectedByGravity then 
						gravityForce = -gravityConstant*obj:mass()
					end
					-- C.printf("gravityForce: %g\n", gravityForce)
					var numContacts = obj2contact(i).size
					indices:resize(2*numContacts)
					coeffs:resize(2*numContacts)

					-- -- Test just set friction forces to zero
					-- for j=0,numContacts do
					-- 	indices(0) = frictionVarIDForContact(j)
					-- 	coeffs(0) = 1.0
					-- 	lpsolve.add_constraintex(lp, 1, &coeffs(0), &indices(0), lpsolve.EQ, 0.0)
					-- end

					-- Force balance gives two constraints: one for x and one for y
					-- x constraint: sum of horizontal forces is zero
					for j=0,numContacts do
						var cid = obj2contact(i)(j)
						var contact = [&Connections.FrictionalContact](connections(cid))
						indices(2*j) = compressionVarIDForContact(cid)
						indices(2*j+1) = frictionVarIDForContact(cid)
						coeffs(2*j) = fsign(obj, contact) * contact.contactNormal(0)
						coeffs(2*j+1) = fsign(obj, contact) * contact.contactTangent(0)
					end
					lpsolve.add_constraintex(lp, 2*numContacts, &coeffs(0), &indices(0), lpsolve.EQ, 0.0)
					-- y constraint: sum of vertical forces counteracts gravity
					for j=0,numContacts do
						var cid = obj2contact(i)(j)
						var contact = [&Connections.FrictionalContact](connections(cid))
						indices(2*j) = compressionVarIDForContact(cid)
						indices(2*j+1) = frictionVarIDForContact(cid)
						coeffs(2*j) = fsign(obj, contact) * contact.contactNormal(1)
						coeffs(2*j+1) = fsign(obj, contact) * contact.contactTangent(1)
					end
					lpsolve.add_constraintex(lp, 2*numContacts, &coeffs(0), &indices(0), lpsolve.EQ, -gravityForce)

					-- Torque balance: We sum up the DOFs of all the forces on this object, then take
					--    min(numDOFs - 2, 1) as the number of places we should evaluate net torque
					var numDOFs = 0
					for j=0,obj.forces.size do numDOFs = numDOFs + obj.forces(j).dof end
					-- var numTorquePoints = numDOFs - 2; if numTorquePoints == 0 then numTorquePoints = 1 end
					-- var numTorquePoints = 1
					var evalpoints = [Vector(Vec2)].stackAlloc()
					-- obj:getCentersOfRotation(numTorquePoints, &evalpoints)
					evalpoints:push(obj:centerOfMass())
					-- Evaluate net torque at every one of these points
					for j=0,evalpoints.size do
						var torquePoint = evalpoints(j)
						-- Build up the net torque here
						for k=0,numContacts do
							var cid = obj2contact(i)(k)
							var contact = [&Connections.FrictionalContact](connections(cid))
							var v = contact.contactPoint - torquePoint
							indices(2*k) = compressionVarIDForContact(cid)
							indices(2*k+1) = frictionVarIDForContact(cid)
							-- Torque is F x v, where F is sign * (compress * normal + friction * tangent)
							-- This simplifies to the following coefficients:
							--    compress: sign * (v x normal)
							--    friction: sign * (v x tangent)
							coeffs(2*k) = staticsUtils.cross(v, fsign(obj, contact) * contact.contactNormal)
							coeffs(2*k+1) = staticsUtils.cross(v, fsign(obj, contact) * contact.contactTangent)
						end
						-- Net internal torque must counteract torque due to gravity
						var gravTorque = staticsUtils.cross(obj:centerOfMass() - torquePoint, Vec2.stackAlloc(0.0, -gravityForce))
						lpsolve.add_constraintex(lp, 2*numContacts, &coeffs(0), &indices(0), lpsolve.EQ, -gravTorque)
					end
					m.destruct(evalpoints)
				end
			end

			-- Solve LP, check for feasibility
			var retcode = lpsolve.solve(lp)
			var isStable = false
			if retcode == lpsolve.OPTIMAL then
				isStable = true
			end
			-- lpsolve.print_lp(lp)
			-- C.printf("\nlpsolve return code: %d\n", retcode)
			-- lpsolve.print_solution(lp, 1)

			m.destruct(obj2index)
			m.destruct(obj2contact)
			m.destruct(indices)
			m.destruct(coeffs)
			lpsolve.delete_lp(lp)

			return isStable
		end
	end

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

	-- Color a beam by its 'stability,' using residual as a proxy for stability
	-- DEPRECATED. We use the LP_SOLVE-based method below, instead
	local stabColor0 = colors.Tableau10.Blue
	local stabColor1 = colors.Tableau10.Green
	local stabColor2 = colors.Tableau10.Yellow
	local stabColor3 = colors.Tableau10.Red
	local terra colorByResidual(maxResRatio: real)
		var mrr = ad.val(maxResRatio)
		if mrr < 1.0 then return lerp(Color3d.stackAlloc(stabColor0), Color3d.stackAlloc(stabColor1), mrr)
		elseif mrr < 2.0 then return lerp(Color3d.stackAlloc(stabColor1), Color3d.stackAlloc(stabColor2), mrr-1.0)
		elseif mrr < 3.0 then return lerp(Color3d.stackAlloc(stabColor2), Color3d.stackAlloc(stabColor3), mrr-3.0)
		else return Color3d.stackAlloc(stabColor3) end
	end
	local visualizeResiduals = nil
	if rmseRes then
		visualizeResiduals = terra(obj: &RigidObjectT, fres: real, tres: real, fsigma: real, tsigma: real)
			var beam = [&BeamT](obj)
			var maxResRatio = ad.math.fmax(fres/fsigma, tres/tsigma)
			beam.color = colorByResidual(maxResRatio)
		end
	else
		visualizeResiduals = terra(obj: &RigidObjectT, fres: Vec2, tres: &Vector(real), fsigma: real, tsigma: real)
			var beam = [&BeamT](obj)
			var maxResRatio = ad.math.fmax(ad.math.fabs(fres(0))/fsigma, ad.math.fabs(fres(1))/fsigma)
			for i=0,tres.size do
				maxResRatio = ad.math.fmax(maxResRatio, ad.math.fabs(tres(i))/tsigma)
			end	
			beam.color = colorByResidual(maxResRatio)
		end
	end

	-- Color the whole structure red if it fails the LP_SOLVE stability check
	local colorIfUnstable = nil
	if real == double then
		colorIfUnstable = terra(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection))
			if not isExactlyStable(scene, connections) then
				for i=0,scene.objects.size do
					var obj = scene.objects(i)
					if obj.active then
						var beam = [&BeamT](obj)
						beam.color = Color3d.stackAlloc([colors.Tableau10.Red])
					end
				end
			end
		end
	else
		colorIfUnstable = terra(scene: &RigidSceneT, connections: &Vector(&Connections.RigidConnection)) end
	end

	-- Enforce static equilibrium of a scene given some connections
	-- local fresSoftness = `0.01  -- Allow deviation of 1% the average force magnitude
	-- local tresSoftness = `0.01
	local fresSoftness = `0.005
	local tresSoftness = `0.005
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

		-- Flag the whole structure with a red color if it can't be made stable
		colorIfUnstable(scene, connections)

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
					-- -- Visualize residuals
					-- visualizeResiduals(scene.objects(i), fres_, tres_, fresSoftness, tresSoftness)
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
					visualizeResiduals(scene.objects(i), fres, &tres, fresSoftness, tresSoftness)
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
	-- local example = examples.funkyTable
	-- local example = examples.arch
	-- local example = examples.linearChainRegularBlockStack
	local example = examples.linearChainNonRegularBlockStack

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

local ensureEven = macro(function(x)
	return quote
		var y = x
		if y % 2 ~= 0 then y = y + 1 end
	in
		y
	end
end)
local imageWidth = 500
local function renderInitFn(samples, im)
	return quote
		var argc = 0
		gl.glutInit(&argc, nil)
		var scene0 = &samples(0).value
		var aspectRatio = scene0.height / scene0.width
		var imageHeight = int(aspectRatio*imageWidth)
		imageHeight = ensureEven(imageHeight)
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		im:resize(imageWidth, imageHeight)
	end
end

-- local forceScale = 0.03
local forceScale = 0.0
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

----------------------------------

-- Take a starting sample scene, run dynamics on it,
--    and output a list of samples for all the scenes
--    in the dynamics trajectory.
-- Kind of janky, but we do it this way to take advantage of existing infrastructure
--    for rendering lists of samples 
local function genDynamicsSamples(computation)
	local staticsDynamics = terralib.require("staticsDynamics")
	local SampType = inf.SampleType(computation)
	local frictionCoeff = 0.5
	return terra(sample: &SampType, timestep: double, numTimeSteps: uint)
		C.printf("Simulating dynamics\n")
		var scene = &sample.value
		var newsamps = [inf.SampleVectorType(computation)].stackAlloc()
		var sim = [staticsDynamics.RigidSceneSim].stackAlloc(scene, gravityConstant, frictionCoeff)
		for i=0,numTimeSteps do
			C.printf(" timestep %d/%d\r", i+1,numTimeSteps)
			var newscene = sim:step(timestep)
			newsamps:push([inf.SampleType(computation)].stackAlloc(newscene, sample.logprob))
			m.destruct(newscene)
		end
		C.printf("\nDONE\n")
		m.destruct(sim)
		return newsamps
	end
end

-- TODO: Functions to compute dynamics for a randomly-chosen stable/unstable structure
--    from the original list of samples.

-- TODO: Add a dynamics check to every sample, before we render it? Change its color if it
--    falls down under dynamics?

----------------------------------

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
-- rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename)

local dynsamples = m.gc(genDynamicsSamples(model)(samples:getPointer(800), 1.0/120.0, 1000))
rendering.renderSamples(dynsamples, renderInitFn, renderDrawFn, moviename)









