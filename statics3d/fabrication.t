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

	local metersToPixels = macro(function(m, ppi)
		return `uint(C.ceil(m*39.3701*ppi))
	end)

	-- Get the face that we'll actually need to cut out for a single shape
	local terra getCutFace(shape: &QuadHex)
		if shape:isFrontBackPlanarExtrusion() then
			return shape:frontFace()
		elseif shape:isLeftRightPlanarExtrusion() then
			return shape:leftFace()
		elseif shape:isBotTopPlanarExtrusion() then
			return shape:botFace()
		else
			util.fatalError("getCutFace - block is not a planar extrusion\n")
		end
	end

	-- Returns the name of the cardinal axis (x, y, z) that locally aligns with
	--    the normal of this face
	local terra normalAxisName(face: &Face(4), shape: &QuadHex)
		if face == shape:leftFace() or face == shape:rightFace() then
			return "X"
		elseif face == shape:frontFace() or face == shape:backFace() then
			return "Y"
		elseif face == shape:botFace() or face == shape:topFace() then
			return "Z"
		else
			util.fatalError("normalAxisName - This should be impossible.\n")
		end
	end

	local struct QuadFace2D
	{
		verts: Vec2d[4]
	}

	-- Project any point according to the face projection transform
	local terra projectPointByFace(face: &Face(4), p: Vec3)
		var n = face:normal()
		var p0 = face:vertex(0)
		p = p:projectToPlane(p0, n)
		var xaxis = (face:vertex(1) - face:vertex(0)); xaxis:normalize()
		var yaxis = n:cross(xaxis)
		return Vec2d.stackAlloc((p-p0):dot(xaxis), (p-p0):dot(yaxis))
	end

	-- Project the face into 2D along its normal.
	local terra projectFace(face: &Face(4))
		var face2d : QuadFace2D
		face2d.verts[0] = projectPointByFace(face, face:vertex(0))
		face2d.verts[1] = projectPointByFace(face, face:vertex(1))
		face2d.verts[2] = projectPointByFace(face, face:vertex(2))
		face2d.verts[3] = projectPointByFace(face, face:vertex(3))
		return face2d
	end

	-- Bound a face
	local terra boundFace(face: &Face(4))
		var face2d = projectFace(face)
		var bbox = BBox2d.stackAlloc()
		bbox:expand(face2d.verts[0])
		bbox:expand(face2d.verts[1])
		bbox:expand(face2d.verts[2])
		bbox:expand(face2d.verts[3])
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
			var f = getCutFace([&QuadHex](scene.bodies(i).shape))
			var bbox = getFaceViewport(f, ppi)
			overallbbox:unionWith(&bbox)
		end
		-- Also expand for all the contact faces
		for i=0,scene.connections.size do
			var contact = [&RectRectContact](scene.connections(i))
			var bbox : BBox2u
			if contact.contactPoints[0].body1 ~= scene.bodies(0) then
				bbox = getFaceViewport(contact.face1, ppi)
				overallbbox:unionWith(&bbox)
			end
			if contact.contactPoints[0].body2 ~= scene.bodies(0) then
				bbox = getFaceViewport(contact.face2, ppi)
				overallbbox:unionWith(&bbox)
			end
		end
		return align32(overallbbox.maxs(0)), align32(overallbbox.maxs(1))
	end

	-- Render the cutout image for a single body
	local terra renderBodyCutout(body: &Body, body2id: &HashMap(&Body, uint), img: &RGBImage, ppi: uint, directory: rawstring, index: uint)
		var shape = [&QuadHex](body.shape)
		var face = getCutFace(shape)
		var face2d = projectFace(face)
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
		gl.glVertex2d([Vec2d.elements(`face2d.verts[0])])
		gl.glVertex2d([Vec2d.elements(`face2d.verts[1])])
		gl.glVertex2d([Vec2d.elements(`face2d.verts[2])])
		gl.glVertex2d([Vec2d.elements(`face2d.verts[3])])
		gl.glEnd()

		-- Draw the id label for this block
		-- Also draw the axis that the normal points along
		var buf : int8[1024]
		var id = body2id(body)
		var axisName = normalAxisName(face, shape)
		C.sprintf(buf, "%u (%s)", id, axisName)
		gl.glRasterPos2d([Vec2d.elements(`projectPointByFace(face, face:centroid()))])
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
			gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, cps(0).point))])
			gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, cps(1).point))])
			gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, cps(2).point))])
			gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, cps(3).point))])
			gl.glEnd()

			-- Draw the other body's id at the center of the contact region
			var id = body2id(otherBody)
			C.sprintf(buf, "%u", id)
			var center = projectPointByFace(face, 0.25 * (cps(0).point + cps(1).point + cps(2).point + cps(3).point))
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

		-- Save to image
		C.sprintf(buf, "%s_ppi=%u/%u_cutout.png", directory, ppi, index)
		img:save(image.Format.PNG, buf)
	end


	-- Render 'top-down' contact images for a body
	local function genRenderContactDiagram(which, body, body2id, face2contact, img, ppi, directory, index)
		local getFace = nil
		if which == "Bot" then
			getFace = `([&QuadHex](body.shape)):botFace()
		elseif which == "Top" then
			getFace = `([&QuadHex](body.shape)):topFace()
		else
			error(string.format("genRenderContactDiagram - invalid 'which' parameter '%s'", which))
		end
		return quote
			var face = [getFace]
			var contact: &RectRectContact
			-- Only render a contact diagram if this face is actually in contact with something
			if face2contact:get(face, &contact) then
				var face2d = projectFace(face)
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

				-- Draw a filled polygon for the contact region
				gl.glColor3d([contactColor])
				gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_FILL())
				gl.glLineWidth(metersToPixels(lineThickness, ppi))
				gl.glBegin(gl.mGL_QUADS())
				gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, contact.contactPoints[0].point))])
				gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, contact.contactPoints[1].point))])
				gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, contact.contactPoints[2].point))])
				gl.glVertex2d([Vec2d.elements(`projectPointByFace(face, contact.contactPoints[3].point))])
				gl.glEnd()

				-- Draw the polygon outline
				gl.glColor3d([outlineColor])
				gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_LINE())
				gl.glLineWidth(metersToPixels(lineThickness, ppi))
				gl.glBegin(gl.mGL_QUADS())
				gl.glVertex2d([Vec2d.elements(`face2d.verts[0])])
				gl.glVertex2d([Vec2d.elements(`face2d.verts[1])])
				gl.glVertex2d([Vec2d.elements(`face2d.verts[2])])
				gl.glVertex2d([Vec2d.elements(`face2d.verts[3])])
				gl.glEnd()

				-- Draw the id label for this block
				-- Also draw the name of the face ("Bot" or "Top")
				var buf : int8[1024]
				var id = body2id(body)
				C.sprintf(buf, "%u (%s)", id, which)
				gl.glRasterPos2d([Vec2d.elements(`projectPointByFace(face, face:centroid()))])
				displayString(gl.mGLUT_BITMAP_TIMES_ROMAN_24(), buf)

				-- Copy framebuffer to image
				img:resize(viewport.maxs(0), viewport.maxs(1))
				gl.glFlush()
				gl.glReadPixels(0, 0, img.width, img.height,
					gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), img.data)

				-- Save to image
				C.sprintf(buf, "%s_ppi=%u/%u_contact_%s.png", directory, ppi, index, which)
				img:save(image.Format.PNG, buf)
			end
		end
	end
	local terra renderBodyContacts(body: &Body, body2id: &HashMap(&Body, uint), face2contact: &HashMap(&Face(4), &RectRectContact),
								   img: &RGBImage, ppi: uint, directory: rawstring, index: uint)
		[genRenderContactDiagram("Bot", body, body2id, face2contact, img, ppi, directory, index)]
		[genRenderContactDiagram("Top", body, body2id, face2contact, img, ppi, directory, index)]
	end


	return terra(scene: &Scene, directory: rawstring, ppi: uint)
		-- Ensure directory exists
		util.systemf("mkdir %s_ppi=%u", directory, ppi)

		-- Init OpenGL stuff
		var maxXres, maxYres = getFramebufferRes(scene, ppi)
		gl.glutInitWindowSize(maxXres, maxYres)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")

		-- Build a hashmap of bodies to ids (for labeling blueprints)
		var body2id = [HashMap(&Body, uint)].stackAlloc()
		for i=0,scene.bodies.size do
			body2id:put(scene.bodies(i), i)
		end

		-- Build a hashmap of contact face --> contact
		-- (for rendering contact diagrams)
		var face2contact = [HashMap(&Face(4), &RectRectContact)].stackAlloc()
		for i=0,scene.connections.size do
			var contact = [&RectRectContact](scene.connections(i))
			face2contact:put(contact.face1, contact)
			face2contact:put(contact.face2, contact)
		end

		-- For each body, render:
		--  * An image describing the block cut-out
		--  * Images (1 or 2) describing block contacts
		var img = RGBImage.stackAlloc()
		img:resize(maxXres, maxYres)
		-- Skip ground at index 0
		for i=1,scene.bodies.size do
			var body = scene.bodies(i)
			renderBodyCutout(body, &body2id, &img, ppi, directory, i)
			renderBodyContacts(body, &body2id, &face2contact, &img, ppi, directory, i)
		end
		m.destruct(img)

		m.destruct(body2id)
		m.destruct(face2contact)
	end
end

return
{
	saveBlueprints = saveBlueprints
}







