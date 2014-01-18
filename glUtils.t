local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local gl = terralib.require("gl")
local image = terralib.require("image")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)

local terra drawPolyhedron(pos: Vec2d, rad: double, color: Color3d, subdivs: int) : {}
  gl.glPushMatrix()
  gl.glTranslated(pos(0), pos(1), 0.0)
  gl.glColor3d(color(0), color(1), color(2))
  gl.glBegin(gl.mGL_POLYGON())
  var angIncr = [2*math.pi] / double(subdivs)
  for i=0,subdivs do
    var ang = (i + 0.5) * angIncr
    gl.glVertex2d(rad*ad.math.cos(ang), rad*ad.math.sin(ang))
  end
  gl.glEnd()
  gl.glPopMatrix()
end

terra drawCircle(pos: Vec2d, rad: double, color: Color3d) : {}
  drawPolyhedron(pos, rad, color, 32)
end

local Circle = templatize(function(real)
  local Vec2 = Vec(real, 2)
  local struct CircleT { pos: Vec2, size: real, color: Color3d}
  terra CircleT:area() return [math.pi]*self.size*self.size end
  terra CircleT:intersectArea(other: &CircleT)
    var r = self.size
    var R = other.size
    var d = self.pos:dist(other.pos)
    if d > r+R then
      return real(0.0)
    end
    if R < r then
      r = other.size
      R = self.size
    end
    var d2 = d*d
    var r2 = r*r
    var R2 = R*R
    var x1 = r2*ad.math.acos((d2 + r2 - R2)/(2*d*r))
    var x2 = R2*ad.math.acos((d2 + R2 - r2)/(2*d*R))
    var x3 = 0.5*ad.math.sqrt((-d+r+R)*(d+r-R)*(d-r+R)*(d+r+R))
    return x1 + x2 - x3
  end
  terra CircleT:render()
    drawCircle(self.pos, self.size, self.color)
  end
  return CircleT
end)

local terra glutInitWindow(width: uint, height: uint)
  var argc = 0
  gl.glutInit(&argc, nil)
  gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_DOUBLE())
  gl.glutInitWindowSize(width, height)
  gl.glutCreateWindow("Render")
  gl.glViewport(0, 0, width, height)
end

local terra glutSetupOrtho2d(width: uint, height: uint)
  gl.glMatrixMode(gl.mGL_PROJECTION())
  gl.glLoadIdentity()
  gl.gluOrtho2D(0, width, 0, height)
  gl.glMatrixMode(gl.mGL_MODELVIEW())
  gl.glLoadIdentity()
end

local terra glReadPixels(im: &RGBImage)
  gl.glFlush()
  gl.glReadPixels(0, 0, im.width, im.height, gl.mGL_BGR(), gl.mGL_UNSIGNED_BYTE(), im.data)
end

local terra glClear()
  gl.glClearColor(1.0, 1.0, 1.0, 1.0)
  gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
end

local function renderSamples(samples, moviename, imageWidth, imageHeight)
  local moviefilename = string.format("renders/%s.mp4", moviename)
  local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
  local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
  io.write("Rendering video...")
  io.flush()
  local numsamps = samples.size
  local frameSkip = math.ceil(numsamps / 1000.0)
  local terra renderFrames()
    -- init opengl context (via glut window; sort of hacky)
    glutInitWindow(imageWidth, imageHeight)
    glutSetupOrtho2d(samples(0).value.width, samples(0).value.height)

    -- Render all frames, save to image, write to disk
    var im = RGBImage.stackAlloc(imageWidth, imageHeight)
    var framename: int8[1024]
    var framenumber = 0
    for i=0,numsamps,frameSkip do
      C.sprintf(framename, movieframebasename, framenumber)
      framenumber = framenumber + 1
      glClear()
      samples(i).value:render()
      glReadPixels(&im)
      [RGBImage.save()](&im, image.Format.PNG, framename)
    end
  end
  renderFrames()
  util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
  util.wait(string.format("rm -f %s", movieframewildcard))
  print("done.")
end

return {
  Circle = Circle,
  glutInitWindow = glutInitWindow,
  glutSetupOrtho2d = glutSetupOrtho2d,
  glReadPixels = glReadPixels,
  glClear = glClear,
  renderSamples = renderSamples
}