terralib.require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = terralib.require("s3dLib")
local m = terralib.require("mem")
local util = terralib.require("util")
local gl = terralib.require("gl")
local rendering = terralib.require("rendering")
local AutoPtr = terralib.require("autopointer")
local Vec = terralib.require("linalg").Vec

local glutils = terralib.require("glutils")
util.importEntries(glutils, "Camera", "Light")

local Vec3d = Vec(double, 3)

local testcomp = probcomp(function()
	local s3d = s3dLib()
	util.importAll(s3d)

	local gravityConst = `9.8
	local upVector = global(Vec3d)
	upVector:getpointer():__construct(0.0, 0.0, 1.0)

	local frelTol = 0.01
	local trelTol = 0.005

	local mm = macro(function(x)
		return `0.001*x
	end)

	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		var camdist = mm(200.0)
		camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
		camera.target = Vec3d.stackAlloc(0.0, 0.0, 0.0)
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		renderScene:addLight(light)

		-- Set up stuff in the scene --

		-- Bodies
		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(1000.0), mm(1000.0), mm(10.0))
		var groundBody = Body.oak(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)
		var boxShape = Box.heapAlloc(); boxShape:makeBox(Vec3.stackAlloc(0.0, 0.0, 0.0), mm(60.0), mm(60.0), mm(60.0))
		var boxBody = Body.oak(boxShape)
		renderScene.scene.bodies:push(boxBody)

		-- Connections
		renderScene.scene.connections:push(RectRectContact.heapAlloc(boxBody, groundBody, boxShape:botFace(), groundShape:topFace(), false))

		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)

		return renderScene
	end
end)

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

local renderForces = true
local RenderSettings = s3dLib(testcomp).RenderSettings
local function renderDrawFn(sample, im)
	return quote
		var renderSettings = RenderSettings.stackAlloc()
		renderSettings.renderForces = renderForces
		var renderScene = &sample.value
		renderScene:render(&renderSettings)
		[rendering.displayLogprob("BottomLeft")](sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------

local numsamps = 100
-- local go = forwardSample(testcomp, 1)
local go = mcmc(testcomp, HMC({numSteps=1000}), {numsamps=numsamps, verbose=true})
-- local go = mcmc(testcomp, GaussianDrift({bandwidth=0.07}), {numsamps=numsamps, verbose=true})
local samples = go()
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, "../renders")






