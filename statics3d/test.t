terralib.require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = terralib.require("s3dLib")
local util = terralib.require("util")
local gl = terralib.require("gl")
local rendering = terralib.require("rendering")
local Vec = terralib.require("linalg").Vec
local colors = terralib.require("colors")

local testcomp = terralib.require("examples.blockStack")


-------------------------------------------------------

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
		var aspectRatio = scene0.camera.aspect
		var imageHeight = int(aspectRatio*imageWidth)
		imageHeight = ensureEven(imageHeight)
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE() or gl.mGLUT_DEPTH())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		im:resize(imageWidth, imageHeight)
	end
end

local renderForces = false
local RenderSettings = s3dLib(testcomp).RenderSettings
local unstableColor = colors.Tableau10.Red
local function renderDrawFn(sample, im)
	return quote
		var renderSettings = RenderSettings.stackAlloc()
		renderSettings.renderForces = renderForces
		var renderScene = &sample.value
		if not renderScene.scene:isStable() then
			renderSettings.activeBodyColor = [Vec(double, 4)].stackAlloc([unstableColor], 1.0)
		end
		renderScene:render(&renderSettings)
		[rendering.displayLogprob("BottomLeft")](sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------

local numsamps = 400
local doHMC = true
local numHMCSteps = 1000
if not doHMC then numsamps = 2*numsamps*numHMCSteps end
local go = forwardSample(testcomp, numsamps)
if doHMC then
	go = mcmc(testcomp, HMC({numSteps=1000}), {numsamps=numsamps, verbose=true})
else
	go = mcmc(testcomp, GaussianDrift({bandwidth=0.003}), {numsamps=numsamps, verbose=true})
end
local samples = go()
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, "../renders")






