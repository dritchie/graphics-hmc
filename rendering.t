local gl = terralib.require("gl")
local image = terralib.require("image")
local util = terralib.require("util")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local RGBImage = image.Image(uint8, 3)


-- drawFn should take a single sample and generate the code to render that sample
local function renderSamples(samples, initFn, drawFn, moviename)
	local moviefilename = string.format("renders/%s.mp4", moviename)
	local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
	local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
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


return
{
	renderSamples = renderSamples
}