local gl = require("gl")
local image = require("image")
local util = require("util")
local colors = require("colors")

local C = terralib.includecstring [[
#include <stdio.h>
#include <string.h>
]]

local RGBImage = image.Image(uint8, 3)


-- drawFn should take a single sample and generate the code to render that sample
local function renderSamples(samples, initFn, drawFn, moviename, rendersDir, doDeleteImages)
	doDeleteImages = (doDeleteImages == nil) and true or doDeleteImages
	local rendersDir = rendersDir or "renders"
	local moviefilename = string.format("%s/%s.mp4", rendersDir, moviename)
	local movieframebasename = string.format("%s/%s", rendersDir, moviename) .. "_%06d.png"
	local movieframewildcard = string.format("%s/%s", rendersDir, moviename) .. "_*.png"
	io.write("Rendering video...")
	io.flush()
	local numsamps = samples.size
	local frameSkip = math.ceil(numsamps / 1000.0)--1000.0)
	local terra renderFrames()
		var im = RGBImage.stackAlloc()
		[initFn(samples, im)]
		var framename: int8[1024]
		var framenumber = 0
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			var samp = samples:getPointer(i)
			[drawFn(samp, im, i)]
			[RGBImage.save()](&im, image.Format.PNG, framename)
		end
	end
	renderFrames()

	-- For handling non powers of 2 size images
	util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p -start_number 0 -vf \"scale=trunc(iw/2)*2:trunc(ih/2)*2\" %s 2>&1", movieframebasename, moviefilename))
	-- util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))


	-- Use this line instead for higher video quality (hasn't been very necessary in my experiments thus far)
	-- util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p -b:v 20M %s 2>&1", movieframebasename, moviefilename))
	if doDeleteImages then
		util.wait(string.format("rm -f %s", movieframewildcard))
	end
	print("done.")
end



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
local function displaySampleInfo(location)
	util.luaAssertWithTrace(location == "TopLeft" or location == "BottomLeft",
		"displayLogprob location must be either 'TopLeft' or 'BottomLeft'")
	return terra(sampleindex: uint, lp: double)
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		var viewport : int[4]
		gl.glGetIntegerv(gl.mGL_VIEWPORT(), viewport)
		gl.gluOrtho2D(double(viewport[0]), double(viewport[2]), double(viewport[1]), double(viewport[3]))
		gl.glMatrixMode(gl.mGL_MODELVIEW())
		gl.glLoadIdentity()
		var str : int8[64]
		C.sprintf(str, "[%u] lp: %g", sampleindex, lp)
		-- Windows...
		-- C.sprintf(str, "[%d] lp: %d", sampleindex, lp)
		gl.glColor3f([lpcolor])
		[util.optionally(location == "TopLeft", function() return quote
			gl.glRasterPos2f(viewport[0] + 0.02*viewport[2], viewport[1] + 0.9*viewport[3])
		end end)]
		[util.optionally(location == "BottomLeft", function() return quote
			gl.glRasterPos2f(viewport[0] + 0.02*viewport[2], viewport[1] + 0.02*viewport[3])
		end end)]
		displayString(lpfont, str)
	end
end

return
{
	renderSamples = renderSamples,
	displaySampleInfo = displaySampleInfo
}
