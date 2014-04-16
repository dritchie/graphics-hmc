local gl = terralib.require("gl")
local image = terralib.require("image")
local util = terralib.require("util")
local colors = terralib.require("colors")

local C = terralib.includecstring [[
#include <stdio.h>
#include <string.h>
]]

local RGBImage = image.Image(uint8, 3)


-- drawFn should take a single sample and generate the code to render that sample
local function renderSamples(samples, initFn, drawFn, moviename, rendersDir)
	local rendersDir = rendersDir or "renders"
	local moviefilename = string.format("%s/%s.mp4", rendersDir, moviename)
	local movieframebasename = string.format("%s/%s", rendersDir, moviename) .. "_%06d.png"
	local movieframewildcard = string.format("%s/%s", rendersDir, moviename) .. "_*.png"
	io.write("Rendering video...")
	io.flush()
	local numsamps = samples.size
	local frameSkip = math.ceil(numsamps / 1000.0)
	local terra renderFrames()
		var im = RGBImage.stackAlloc()
		[initFn(samples, im)]
		var framename: int8[1024]
		var framenumber = 0
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			var samp = samples:getPointer(i)
			[drawFn(samp, im)]
			[RGBImage.save()](&im, image.Format.PNG, framename)
		end
	end
	renderFrames()
	util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
	util.wait(string.format("rm -f %s", movieframewildcard))
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
local function displayLogprob(location)
	util.luaAssertWithTrace(location == "TopLeft" or location == "BottomLeft",
		"displayLogprob location must be either 'TopLeft' or 'BottomLeft'")
	return terra(lp: double)
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
	displayLogprob = displayLogprob
}