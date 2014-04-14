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
local Camera = terralib.require("glutils").Camera

local Vec3d = Vec(double, 3)

local testcomp = probcomp(function()
	local s3d = s3dLib()
	util.importAll(s3d)

	local gravityConst = `9.8
	local upVector = global(Vec3d)
	upVector:getpointer():__construct(0.0, 0.0, 1.0)

	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		camera.eye = Vec3d.stackAlloc(0.0, 0.0, 0.0)
		camera.target = Vec3d.stackAlloc(0.0, 1.0, 0.0)
		camera.up = upVector
		camera.fovy = 45.0
		camera.aspect = 1.0
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))

		-- Set up stuff in the scene
		var boxShape = Box.heapAlloc(); boxShape:makeBox(Vec3.stackAlloc(0.0, 1.0, 0.0), 0.25, 0.25, 0.25)
		var boxBody = Body.oak(boxShape)
		renderScene.scene.bodies:push(boxBody)

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

local function renderDrawFn(sample, im)
	return quote
		var renderSettings = RenderSettings.stackAlloc()
		var renderScene = &sample.value
		renderScene:render(&renderSettings)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------

local samples = m.gc(forwardSample(testcomp, 1)())
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename, "../renders")




