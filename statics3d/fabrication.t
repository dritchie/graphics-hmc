terralib.require("prob")

local m = terralib.require("mem")
local ad = terralib.require("ad")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")
local HashMap = terralib.require("hashmap")
local gl = terralib.require("gl")
local image = terralib.require("image")
local colors = terralib.require("colors")
local BBox = terralib.require("bbox")
local s3dLib = terralib.require("s3dLib")


local C = terralib.includecstring [[
#include "stdio.h"
#include "string.h"
#include "math.h"
]]

local RGBImage = image.Image(uint8, 3)
local Vec2d = Vec(double, 2)
local BBox2d = BBox(Vec2d)

gl.exposeConstants({"GL_VIEWPORT", {"GLUT_BITMAP_TIMES_ROMAN_24", "void*"}})

local terra displayString(font: &opaque, str: rawstring)
	if str ~= nil and C.strlen(str) > 0 then
		while @str ~= 0 do
			gl.glutBitmapCharacter(font, @str)
			str = str + 1
		end
	end
end

local mm = macro(function(x)
	return `0.001*x
end)


--------------
-- Rendering blueprints for cutting out blocks to build stacking structurs
-- This is not fully general. It assumes:
--    * Ground is body 0 and is ignored
--    * All bodies have QuadHex shape, and those shapes are planar extrusions of the
--      xz plane
--    * All connections are RectRectContacts.
--------------

local outlineColor = colors.Black
local contactColor = colors.Tableau10.Red
local lineThickness = `mm(0.5)
local bboxExpand = `mm(10.0)

local function saveBlueprints(pcomp)
	local s3d = s3dLib(pcomp)
	util.importAll(s3d)

	local v3tov2 = macro(function(vec)
		return `Vec2d.stackAlloc(vec(0), vec(2))
	end)

	local metersToPixels = macro(function(m, ppi)
		return `uint(C.ceil(m*39.3701*ppi))
	end)

	-- Get the face that we'll actually render for a single shape
	local terra getRenderFace(shape: &QuadHex)
		for i=0,6 do
			if -shape.faces[i]:normal()(1) > 0.999 then
				return &shape.faces[i]
			end
		end
		util.fatalError("getRenderFace - no face aligned with y\n")
	end

	-- Get the depth of a block
	local terra getDepth(shape: &QuadHex) : double
		-- Find face whose normal points along +z, check the length of
		--    the edge of that face that points along y
		var y = Vec3.stackAlloc(0.0, 1.0, 0.0)
		for i=0,6 do
			if shape.faces[i]:normal()(2) > 0.999 then
				var f = &shape.faces[i]
				var e1 = f:vertex(1) - f:vertex(0)
				if e1:collinear(y) then return e1:norm() end
				var e2 = f:vertex(2) - f:vertex(1)
				util.assert(e2:collinear(y), "getDepth - no edge on the z-aligned face was collinear with y\n")
				return e2:norm()
			end
		end
		util.fatalError("getDepth - no face aligned with z\n")
	end

	-- Get the block depth being used for the scene. Throws an error if
	--    blocks have different depths
	terra getDepth(scene: &Scene) : double
		-- Skip ground at index 0
		var depth = getDepth([&QuadHex](scene.bodies(1).shape))
		for i=2,scene.bodies.size do
			var d = getDepth([&QuadHex](scene.bodies(i).shape))
			util.assert(C.fabs(d - depth) < 1e-6,
				"getDepth - Found blocks with different depths: should be %g, was %g\n",
				depth, d)
		end
		return depth
	end

	-- Bound a face
	local terra boundFace(face: &Face(4))
		var bbox = BBox2d.stackAlloc()
		bbox:expand(v3tov2(face:vertex(0)))
		bbox:expand(v3tov2(face:vertex(1)))
		bbox:expand(v3tov2(face:vertex(2)))
		bbox:expand(v3tov2(face:vertex(3)))
		bbox:expand(bboxExpand)
		return bbox
	end

	-- Assuming n is a number of 24-bit pixels, make 24*n be 32-bit aligned
	-- (3-byte pixels, make 3*n be 4-byte aligned)
	local align32 = macro(function(n)
		return `uint(12*C.ceil((3*n)/12.0)/3)
	end)

	local Vec2u = Vec(uint, 2)
	local BBox2u = BBox(Vec2u)
	-- Get the image resolution needed for rendering this face
	local terra getFaceViewport(face: &Face(4), ppi: uint)
		var bbox = boundFace(face)
		bbox.maxs = bbox.maxs - bbox.mins
		return BBox2u.stackAlloc(
			Vec2u.stackAlloc(0, 0),
			Vec2u.stackAlloc(align32(metersToPixels(bbox.maxs(0), ppi)),
							 align32(metersToPixels(bbox.maxs(1), ppi)))
		)
	end

	-- Get the image resolution that we need to allocate in our GL framebuffer
	-- This is the number of pixels needed to render the largest block.
	local terra getFramebufferRes(scene: &Scene, ppi: uint)
		var overallbbox = BBox2u.stackAlloc()
		-- Skip ground at index 0
		for i=1,scene.bodies.size do
			var f = getRenderFace([&QuadHex](scene.bodies(i).shape))
			var bbox = getFaceViewport(f, ppi)
			overallbbox:unionWith(&bbox)
		end
		return align32(overallbbox.maxs(0)), align32(overallbbox.maxs(1))
	end

	-- Render the image for a single body
	local terra renderBodyBlueprint(body: &Body, body2id: &HashMap(&Body, uint), img: &RGBImage, ppi: uint)
		var shape = [&QuadHex](body.shape)
		var face = getRenderFace(shape)
		var bounds = boundFace(face)
		var viewport = getFaceViewport(face, ppi)
		gl.glClearColor([colors.White], 1.0)
		gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
		gl.glViewport(viewport.mins(0), viewport.mins(1), viewport.maxs(0), viewport.maxs(1))
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		gl.gluOrtho2D(bounds.mins(0), bounds.maxs(0), bounds.mins(1), bounds.maxs(1))
		gl.glMatrixMode(gl.mGL_MODELVIEW())
		gl.glLoadIdentity()

		-- Draw the polygon outline
		gl.glColor3d([outlineColor])
		gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_LINE())
		gl.glLineWidth(metersToPixels(lineThickness, ppi))
		gl.glBegin(gl.mGL_QUADS())
		gl.glVertex2d([Vec2d.elements(`v3tov2(face:vertex(0)))])
		gl.glVertex2d([Vec2d.elements(`v3tov2(face:vertex(1)))])
		gl.glVertex2d([Vec2d.elements(`v3tov2(face:vertex(2)))])
		gl.glVertex2d([Vec2d.elements(`v3tov2(face:vertex(3)))])
		gl.glEnd()

		-- Draw the id label for this block
		var buf : int8[32]
		var id = body2id(body)
		C.sprintf(buf, "%u", id)
		gl.glRasterPos2d([Vec2d.elements(`v3tov2(face:centroid()))])
		displayString(gl.mGLUT_BITMAP_TIMES_ROMAN_24(), buf)

		-- We only ended up storing what contact *points* were involved with each body,
		-- so to recover the contact polygon, we'll need to do a little work
		var body2contactpoints = [HashMap(&Body, Vector(&ContactPoint))].stackAlloc()
		for i=0,body.connections.size do
			var contactpoint = [&ContactPoint](body.connections(i))
			var cps : &Vector(&ContactPoint)
			var existed : bool
			if body == contactpoint.body1 then
				cps, existed = body2contactpoints:getOrCreatePointer(contactpoint.body2)
			else
				cps, existed = body2contactpoints:getOrCreatePointer(contactpoint.body1)
			end
			cps:push(contactpoint)
		end

		-- Draw the contact regions and id labels for adjacent blocks
		gl.glColor3d([contactColor])
		var it = body2contactpoints:iterator()
		[util.foreach(it, quote
			var otherBody = it:key()
			var cps = it:valPointer()

			-- Simplest way to get the contact region is to just draw a quad
			--   for the 3d region, projected into 2d
			gl.glBegin(gl.mGL_QUADS())
			gl.glVertex2d([Vec2d.elements(`v3tov2(cps(0).point))])
			gl.glVertex2d([Vec2d.elements(`v3tov2(cps(1).point))])
			gl.glVertex2d([Vec2d.elements(`v3tov2(cps(2).point))])
			gl.glVertex2d([Vec2d.elements(`v3tov2(cps(3).point))])
			gl.glEnd()

			-- Draw the other body's id at the center of the contact region
			id = body2id(otherBody)
			C.sprintf(buf, "%u", id)
			var center = v3tov2(0.25 * (cps(0).point + cps(1).point + cps(2).point + cps(3).point))
			center(1) = center(1) + bboxExpand*0.05
			gl.glRasterPos2d([Vec2d.elements(center)])
			displayString(gl.mGLUT_BITMAP_TIMES_ROMAN_24(), buf)

		end)]

		m.destruct(body2contactpoints)

		-- Copy framebuffer to image
		img:resize(viewport.maxs(0), viewport.maxs(1))
		gl.glFlush()
		gl.glReadPixels(0, 0, img.width, img.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), img.data)
	end


	return terra(scene: &Scene, directory: rawstring, ppi: uint)
		-- Ensure directory exists
		var depth = getDepth(scene)
		util.systemf("mkdir %s_depth=%g_ppi=%u", directory, depth, ppi)

		-- Init OpenGL stuff
		var argc = 0
		gl.glutInit(&argc, nil)
		var maxXres, maxYres = getFramebufferRes(scene, ppi)
		gl.glutInitWindowSize(maxXres, maxYres)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")

		-- Build a hashmap of bodies to ids (for labeling blueprints)
		var body2id = [HashMap(&Body, uint)].stackAlloc()
		for i=0,scene.bodies.size do
			body2id:put(scene.bodies(i), i)
		end

		-- For each body, render an image
		var buf : int8[1024]
		var img = RGBImage.stackAlloc()
		img:resize(maxXres, maxYres)
		-- Skip ground at index 0
		for i=1,scene.bodies.size do
			var body = scene.bodies(i)
			renderBodyBlueprint(body, &body2id, &img, ppi)
			C.sprintf(buf, "%s_depth=%g_ppi=%u/%u.png", directory, depth, ppi, i)
			img:save(image.Format.PNG, buf)
		end
		m.destruct(img)

		m.destruct(body2id)
	end
end

return
{
	saveBlueprints = saveBlueprints
}







