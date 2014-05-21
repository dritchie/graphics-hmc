terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local inheritance = terralib.require("inheritance")
local templatize = terralib.require("templatize")
local Vector = terralib.require("vector")
local s3dCore = terralib.require("s3dCore")
local s3dRendering = terralib.require("s3dRendering")
local s3dConnections = terralib.require("s3dConnections")

local C = terralib.includecstring [[
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
]]


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)
local rendering = s3dRendering(pcomp)
util.importAll(rendering)
local connections = s3dConnections(pcomp)
util.importAll(connections)



----- ADDITIONAL USEFUL OPERATIONS ON RECTANGULAR FACES

local coreFace = Face
core.Face = templatize(function(nverts)
	local FaceT = coreFace(nverts)
	if nverts == 4 then
		
		-- Stitch this face to another face so that they are aligned and coplanar
		--    and the centroid of self is at point
		terra FaceT:weld(face: &FaceT, point: Vec3, validCheck: bool) : {}
			-- Transforms to apply:
			--  * Move self:centroid to origin
			--  * Rotate so -self:normal faces face:normal (coplanarity)
			--  (* Assume that tangent directions align...)
			--  * Move origin to point
			var mat = Mat4.translate(point) * Mat4.face(-self:normal(), face:normal()) * Mat4.translate(-self:centroid())
			self:transform(&mat)
			if validCheck then
				util.assert(RectRectContact.isValidContact(self, face),
					"Cannot stitch non-aligned rectangular faces\n")
			end
		end

		terra FaceT:weld(face: &FaceT, xcoord: real, ycoord: real, validCheck: bool, relCoords: bool) : {}
			var xedge = face:vertex(1) - face:vertex(0)
			var yedge = face:vertex(2) - face:vertex(1)
			if not relCoords then
				xedge:normalize()
				yedge:normalize()
			end
			var point = face:vertex(0) + xcoord*xedge + ycoord*yedge
			self:weld(face, point, validCheck)
		end

		terra FaceT:weld(face: &FaceT, xcoord: real, ycoord: real, validCheck: bool) : {}
			self:weld(face, xcoord, ycoord, validCheck, true)
		end

	end
	return FaceT
end)
Face = core.Face




local QuadFace = Face(4)


----- HEXAHEDRA


-- A convex hexahedron with quadrilateral faces
local struct QuadHex
{
	faces: QuadFace[6]
}
QuadHex.ParentClass = ConvexPrimitiveShape(8)
inheritance.dynamicExtend(QuadHex.ParentClass, QuadHex)

-- Identifying vertices / faces
QuadHex.fFront = 0
QuadHex.fBack = 1
QuadHex.fTop = 2
QuadHex.fBot = 3
QuadHex.fLeft = 4
QuadHex.fRight = 5
QuadHex.vFrontBotLeft = 0
QuadHex.vFrontBotRight = 1
QuadHex.vFrontTopRight = 2
QuadHex.vFrontTopLeft = 3
QuadHex.vBackBotLeft = 4
QuadHex.vBackBotRight = 5
QuadHex.vBackTopRight = 6
QuadHex.vBackTopLeft = 7

-- Face accessors for client code
QuadHex.methods.frontFace = macro(function(self)
	return `&self.faces[ [QuadHex.fFront] ]
end)
QuadHex.methods.backFace = macro(function(self)
	return `&self.faces[ [QuadHex.fBack] ]
end)
QuadHex.methods.topFace = macro(function(self)
	return `&self.faces[ [QuadHex.fTop] ]
end)
QuadHex.methods.botFace = macro(function(self)
	return `&self.faces[ [QuadHex.fBot] ]
end)
QuadHex.methods.leftFace = macro(function(self)
	return `&self.faces[ [QuadHex.fLeft] ]
end)
QuadHex.methods.rightFace = macro(function(self)
	return `&self.faces[ [QuadHex.fRight] ]
end)

-- Edge accessors (edges always point in positive axis direction)
QuadHex.methods.frontLeftEdge = macro(function(self)
	return `self.verts[ [QuadHex.vFrontTopLeft] ] - self.verts[ [QuadHex.vFrontBotLeft] ]
end)
QuadHex.methods.frontRightEdge = macro(function(self)
	return `self.verts[ [QuadHex.vFrontTopRight] ] - self.verts[ [QuadHex.vFrontBotRight] ]
end)
QuadHex.methods.backLeftEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackTopLeft] ] - self.verts[ [QuadHex.vBackBotLeft] ]
end)
QuadHex.methods.backRightEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackTopRight] ] - self.verts[ [QuadHex.vBackBotRight] ]
end)
QuadHex.methods.topFrontEdge = macro(function(self)
	return `self.verts[ [QuadHex.vFrontTopRight] ] - self.verts[ [QuadHex.vFrontTopLeft] ]
end)
QuadHex.methods.topBackEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackTopRight] ] - self.verts[ [QuadHex.vBackTopLeft] ]
end)
QuadHex.methods.topLeftEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackTopLeft] ] - self.verts[ [QuadHex.vFrontTopLeft] ]
end)
QuadHex.methods.topRightEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackTopRight] ] - self.verts[ [QuadHex.vFrontTopRight] ]
end)
QuadHex.methods.botFrontEdge = macro(function(self)
	return `self.verts[ [QuadHex.vFrontBotRight] ] - self.verts[ [QuadHex.vFrontBotLeft] ]
end)
QuadHex.methods.botBackEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackBotRight] ] - self.verts[ [QuadHex.vBackBotLeft] ]
end)
QuadHex.methods.botLeftEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackBotLeft] ] - self.verts[ [QuadHex.vFrontBotLeft] ]
end)
QuadHex.methods.botRightEdge = macro(function(self)
	return `self.verts[ [QuadHex.vBackBotRight] ] - self.verts[ [QuadHex.vFrontBotRight] ]
end)


terra QuadHex:__construct() : {}
	[QuadHex.ParentClass].__construct(self)

	-- We initial to a unit cube at the origin in a z-up coordinate system.

	self.faces[ [QuadHex.fFront] ] = QuadFace.stackAlloc(self, [QuadHex.vFrontBotLeft], [QuadHex.vFrontBotRight], [QuadHex.vFrontTopRight], [QuadHex.vFrontTopLeft])
	self.faces[ [QuadHex.fBack] ] = QuadFace.stackAlloc(self, [QuadHex.vBackBotRight], [QuadHex.vBackBotLeft], [QuadHex.vBackTopLeft], [QuadHex.vBackTopRight])
	self.faces[ [QuadHex.fTop] ] = QuadFace.stackAlloc(self, [QuadHex.vFrontTopLeft], [QuadHex.vFrontTopRight], [QuadHex.vBackTopRight], [QuadHex.vBackTopLeft])
	self.faces[ [QuadHex.fBot] ] = QuadFace.stackAlloc(self, [QuadHex.vFrontBotRight], [QuadHex.vFrontBotLeft], [QuadHex.vBackBotLeft], [QuadHex.vBackBotRight])
	self.faces[ [QuadHex.fLeft] ] = QuadFace.stackAlloc(self, [QuadHex.vBackBotLeft], [QuadHex.vFrontBotLeft], [QuadHex.vFrontTopLeft], [QuadHex.vBackTopLeft])
	self.faces[ [QuadHex.fRight] ] = QuadFace.stackAlloc(self, [QuadHex.vFrontBotRight], [QuadHex.vBackBotRight], [QuadHex.vBackTopRight], [QuadHex.vFrontTopRight])

	self.verts[ [QuadHex.vFrontBotLeft] ] = Vec3.stackAlloc(-0.5, -0.5, -0.5)
	self.verts[ [QuadHex.vFrontBotRight] ] = Vec3.stackAlloc(0.5, -0.5, -0.5)
	self.verts[ [QuadHex.vFrontTopRight] ] = Vec3.stackAlloc(0.5, -0.5, 0.5)
	self.verts[ [QuadHex.vFrontTopLeft] ] = Vec3.stackAlloc(-0.5, -0.5, 0.5)
	self.verts[ [QuadHex.vBackBotLeft] ] = Vec3.stackAlloc(-0.5, 0.5, -0.5)
	self.verts[ [QuadHex.vBackBotRight] ] = Vec3.stackAlloc(0.5, 0.5, -0.5)
	self.verts[ [QuadHex.vBackTopRight] ] = Vec3.stackAlloc(0.5, 0.5, 0.5)
	self.verts[ [QuadHex.vBackTopLeft] ] = Vec3.stackAlloc(-0.5, 0.5, 0.5)
end

terra QuadHex:__construct(fbl: Vec3, fbr: Vec3, ftl: Vec3, ftr: Vec3, bbl: Vec3, bbr: Vec3, btl: Vec3, btr: Vec3) : {}
	self:__construct()

	self.verts[ [QuadHex.vFrontBotLeft] ] = fbl
	self.verts[ [QuadHex.vFrontBotRight] ] = fbr
	self.verts[ [QuadHex.vFrontTopRight] ] = ftr
	self.verts[ [QuadHex.vFrontTopLeft] ] = ftl
	self.verts[ [QuadHex.vBackBotLeft] ] = bbl
	self.verts[ [QuadHex.vBackBotRight] ] = bbr
	self.verts[ [QuadHex.vBackTopRight] ] = btr
	self.verts[ [QuadHex.vBackTopLeft] ] = btl
end

terra QuadHex:makeBox(center: Vec3, width: real, depth: real, height: real)
	var w2 = 0.5*width
	var d2 = 0.5*depth
	var h2 = 0.5*height
	self.verts[ [QuadHex.vFrontBotLeft] ] = center + Vec3.stackAlloc(-w2, -d2, -h2)
	self.verts[ [QuadHex.vFrontBotRight] ] = center + Vec3.stackAlloc(w2, -d2, -h2)
	self.verts[ [QuadHex.vFrontTopRight] ] = center + Vec3.stackAlloc(w2, -d2, h2)
	self.verts[ [QuadHex.vFrontTopLeft] ] = center + Vec3.stackAlloc(-w2, -d2, h2)
	self.verts[ [QuadHex.vBackBotLeft] ] = center + Vec3.stackAlloc(-w2, d2, -h2)
	self.verts[ [QuadHex.vBackBotRight] ] = center + Vec3.stackAlloc(w2, d2, -h2)
	self.verts[ [QuadHex.vBackTopRight] ] = center + Vec3.stackAlloc(w2, d2, h2)
	self.verts[ [QuadHex.vBackTopLeft] ] = center + Vec3.stackAlloc(-w2, d2, h2)
end

terra QuadHex:getQuadFaces(outvec: &Vector(&QuadFace)) : {}
	outvec:push(self:botFace())
	outvec:push(self:topFace())
	outvec:push(self:leftFace())
	outvec:push(self:rightFace())
	outvec:push(self:frontFace())
	outvec:push(self:backFace())
end
inheritance.virtual(QuadHex, "getQuadFaces")

-- Volume of a convex hexahedron is just the sum of the volumes
--    of the six pyramids formed by connecting each face to the centroid
-- Face areas are just one half the norm of the cross product of the diagonals
terra QuadHex:volume() : real
	var c = self:centroid()
	return [(function()
		local oneThird = 1.0/3
		local exp = `real(0.0)
		for i=0,5 do
			local volq = quote
				var f = &self.faces[i]
				var baseArea = 0.5*(f:vertex(2) - f:vertex(0)):cross(f:vertex(3) - f:vertex(1)):norm()
				var fpoint = f:vertex(0)
				var finnormal = -f:normal()
				var pyrheight = (c - fpoint):dot(finnormal)
			in
				oneThird * baseArea * pyrheight
			end
			exp = `[exp] + [volq]
		end
		return exp
	end)()]
end
inheritance.virtual(QuadHex, "volume")

if real == double then
	terra QuadHex:render(settings: &RenderSettings) : {}
		self.faces[0]:render(settings)
		self.faces[1]:render(settings)
		self.faces[2]:render(settings)
		self.faces[3]:render(settings)
		self.faces[4]:render(settings)
		self.faces[5]:render(settings)
	end
	inheritance.virtual(QuadHex, "render")
end

-- Various methods to check if the shape is a planar extrusion along different
--    axes

local planarExtrudeThresh = 1e-7

terra QuadHex:isFrontBackPlanarExtrusion()
	var disp = self.faces[ [QuadHex.fBack] ]:centroid() - self.faces[ [QuadHex.fFront] ]:centroid()
	var err = real(0.0)
	err = err + (self.verts[ [QuadHex.vBackBotLeft] ] - (self.verts[ [QuadHex.vFrontBotLeft] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vBackBotRight] ] - (self.verts[ [QuadHex.vFrontBotRight] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vBackTopRight] ] - (self.verts[ [QuadHex.vFrontTopRight] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vBackTopLeft] ] - (self.verts[ [QuadHex.vFrontTopLeft] ] + disp)):norm()
	return err < planarExtrudeThresh
end

terra QuadHex:isLeftRightPlanarExtrusion()
	var disp = self.faces[ [QuadHex.fRight] ]:centroid() - self.faces[ [QuadHex.fLeft] ]:centroid()
	var err = real(0.0)
	err = err + (self.verts[ [QuadHex.vBackBotRight] ] - (self.verts[ [QuadHex.vBackBotLeft] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vFrontBotRight] ] - (self.verts[ [QuadHex.vFrontBotLeft] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vFrontTopRight] ] - (self.verts[ [QuadHex.vFrontTopLeft] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vBackTopRight] ] - (self.verts[ [QuadHex.vBackTopLeft] ] + disp)):norm()
	return err < planarExtrudeThresh
end

terra QuadHex:isBotTopPlanarExtrusion()
	var disp = self.faces[ [QuadHex.fTop] ]:centroid() - self.faces[ [QuadHex.fBot] ]:centroid()
	var err = real(0.0)
	err = err + (self.verts[ [QuadHex.vBackTopLeft] ] - (self.verts[ [QuadHex.vBackBotLeft] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vBackTopRight] ] - (self.verts[ [QuadHex.vBackBotRight] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vFrontTopRight] ] - (self.verts[ [QuadHex.vFrontBotRight] ] + disp)):norm()
	err = err + (self.verts[ [QuadHex.vFrontTopLeft] ] - (self.verts[ [QuadHex.vFrontBotLeft] ] + disp)):norm()
	return err < planarExtrudeThresh
end

-- Assumes self is stacked on top of a another QuadHex.
-- Rotates self about its normal so that its bottom face is aligned with
--    the top face of the QuadHex that it is stacked on.
-- Assumes the two faces are rectangular, so it only aligns along one axis
terra QuadHex:alignStacked(box: &QuadHex)
	var botCenter = self:botFace():centroid()
	var myAxis = self:botFrontEdge(); myAxis:normalize()
	var otherAxis = box:topFrontEdge(); otherAxis:normalize()
	-- C.printf("myAxis: "); myAxis:print(); C.printf("  |  otherAxis: "); otherAxis:print(); C.printf("\n")
	var xform = Mat4.translate(botCenter) *
				Mat4.face(myAxis, otherAxis) *
				Mat4.translate(-botCenter)
	self:transform(&xform)
end

-- Transform self such that its bottom face sits on the top face of box,
--    and the centroid of its bottom face is located at point.
terra QuadHex:stack(box: &QuadHex, point: Vec3) : {}
	var myBotFace = self:botFace()
	var targetTopFace = box:topFace()
	var xform = Mat4.translate(point) *
				Mat4.face(-myBotFace:normal(), targetTopFace:normal()) *
				Mat4.translate(-myBotFace:centroid())
	self:transform(&xform)
end

-- Transform self such that its bottom face sits on the top face of box,
--    and the centroid of its bottom face is located at the (xcoord, ycoord)
--    in the local coordinate system of box's top face
-- If relativeCoords is true, then xcoord,ycoord are interpreted as percentages along
--    their respective edges. Otherwise, they are interpreted as absolute physical unites
terra QuadHex:stack(box: &QuadHex, xcoord: real, ycoord: real, relativeCoords: bool) : {}
	var origin = box.verts[ [QuadHex.vFrontTopLeft] ]
	var xedge = box.verts[ [QuadHex.vFrontTopRight] ] - origin
	var yedge = box.verts[ [QuadHex.vBackTopLeft] ] - origin
	if not relativeCoords then
		xedge:normalize()
		yedge:normalize()
	end
	var point = origin + xcoord*xedge + ycoord*yedge
	self:stack(box, point)
end

-- Transform self such that its bottom face sits on the top face of box,
--    and the centroid of its bottom face is located somewhere such that
--    there is at least 'margin' overlap between the two contacting faces.
-- Only valid between coplanar, aligned, rectangular faces. If checkValidity
--    is true, we check this condition before proceeding.
terra QuadHex:stackRandom(box: &QuadHex, margin: real, checkValidity: bool, initialToZero: bool) : {}
	-- Setup by getting us stacked at the centroid, and check validity if requested.
	var topFaceCentroid = box:topFace():centroid()
	self:stack(box, topFaceCentroid)
	if checkValidity then
		util.assert(RectRectContact.isValidContact(box:topFace(), self:botFace()))
	end

	-- Figure out which edge pairs correspond between the two faces so we can
	--    compute uniform variable bounds
	var myorigin = self.verts[ [QuadHex.vFrontBotLeft] ]
	var myxedge = self.verts[ [QuadHex.vFrontBotRight] ] - myorigin
	var myyedge = self.verts[ [QuadHex.vBackBotLeft] ] - myorigin
	var boxorigin = box.verts[ [QuadHex.vFrontTopLeft] ]
	var boxxedge = box.verts[ [QuadHex.vFrontTopRight] ] - boxorigin
	var boxyedge = box.verts[ [QuadHex.vBackTopLeft] ] - boxorigin
	var myxnorm = myxedge:norm()
	var myynorm = myyedge:norm()
	var boxxnorm = boxxedge:norm()
	var boxynorm = boxyedge:norm()
	myxedge = myxedge / myxnorm
	myyedge = myyedge / myynorm
	boxxedge = boxxedge / boxxnorm
	boxyedge = boxyedge / boxynorm
	if myxedge:dot(boxxedge) < myxedge:dot(boxyedge) then
		util.swap(myxedge, myyedge)
		util.swap(myxnorm, myynorm)
	end

	-- Compute bounds, draw uniform variables, do stacking
	var oneSidedXrange = 0.5*boxxnorm + 0.5*myxnorm - margin
	var oneSidedYrange = 0.5*boxynorm + 0.5*myynorm - margin
	var xperturb : real
	var yperturb : real
	if initialToZero then
		xperturb = boundedUniform(-oneSidedXrange, oneSidedXrange, {initialVal=0.0})
		yperturb = boundedUniform(-oneSidedYrange, oneSidedYrange, {initialVal=0.0})
	else
		xperturb = boundedUniform(-oneSidedXrange, oneSidedXrange)
		yperturb = boundedUniform(-oneSidedYrange, oneSidedYrange)
	end
	var disp = xperturb*boxxedge + yperturb*boxyedge
	var mat = Mat4.translate(disp)
	self:transform(&mat)
end
terra QuadHex:stackRandom(box: &QuadHex, margin: real, checkValidity: bool) : {}
	self:stackRandom(box, margin, checkValidity, false)
end
QuadHex.methods.stackRandom = pmethod(QuadHex.methods.stackRandom)



terra QuadHex:stackRandomX(box: &QuadHex, margin: real, checkValidity: bool, initialToZero: bool) : {}
	-- Setup by getting us stacked at the centroid, and check validity if requested.
	var topFaceCentroid = box:topFace():centroid()
	self:stack(box, topFaceCentroid)
	if checkValidity then
		util.assert(RectRectContact.isValidContact(box:topFace(), self:botFace()))
	end

	-- Figure out which edge pairs correspond between the two faces so we can
	--    compute uniform variable bounds
	var myorigin = self.verts[ [QuadHex.vFrontBotLeft] ]
	var myxedge = self.verts[ [QuadHex.vFrontBotRight] ] - myorigin
	var myyedge = self.verts[ [QuadHex.vBackBotLeft] ] - myorigin
	var boxorigin = box.verts[ [QuadHex.vFrontTopLeft] ]
	var boxxedge = box.verts[ [QuadHex.vFrontTopRight] ] - boxorigin
	var boxyedge = box.verts[ [QuadHex.vBackTopLeft] ] - boxorigin
	var myxnorm = myxedge:norm()
	var myynorm = myyedge:norm()
	var boxxnorm = boxxedge:norm()
	var boxynorm = boxyedge:norm()
	myxedge = myxedge / myxnorm
	myyedge = myyedge / myynorm
	boxxedge = boxxedge / boxxnorm
	boxyedge = boxyedge / boxynorm
	if myxedge:dot(boxxedge) < myxedge:dot(boxyedge) then
		util.swap(myxedge, myyedge)
		util.swap(myxnorm, myynorm)
	end

	-- Compute bounds, draw uniform variables, do stacking
	var oneSidedXrange = 0.5*boxxnorm + 0.5*myxnorm - margin
	var xperturb : real
	if initialToZero then
		xperturb = boundedUniform(-oneSidedXrange, oneSidedXrange, {initialVal=0.0})
	else
		xperturb = boundedUniform(-oneSidedXrange, oneSidedXrange)
	end
	var disp = xperturb*boxxedge
	var mat = Mat4.translate(disp)
	self:transform(&mat)
end
terra QuadHex:stackRandomX(box: &QuadHex, margin: real, checkValidity: bool) : {}
	self:stackRandomX(box, margin, checkValidity, false)
end
QuadHex.methods.stackRandomX = pmethod(QuadHex.methods.stackRandomX)



terra QuadHex:stackRandomY(box: &QuadHex, margin: real, checkValidity: bool, initialToZero: bool) : {}
	-- Setup by getting us stacked at the centroid, and check validity if requested.
	var topFaceCentroid = box:topFace():centroid()
	self:stack(box, topFaceCentroid)
	if checkValidity then
		util.assert(RectRectContact.isValidContact(box:topFace(), self:botFace()))
	end

	-- Figure out which edge pairs correspond between the two faces so we can
	--    compute uniform variable bounds
	var myorigin = self.verts[ [QuadHex.vFrontBotLeft] ]
	var myxedge = self.verts[ [QuadHex.vFrontBotRight] ] - myorigin
	var myyedge = self.verts[ [QuadHex.vBackBotLeft] ] - myorigin
	var boxorigin = box.verts[ [QuadHex.vFrontTopLeft] ]
	var boxxedge = box.verts[ [QuadHex.vFrontTopRight] ] - boxorigin
	var boxyedge = box.verts[ [QuadHex.vBackTopLeft] ] - boxorigin
	var myxnorm = myxedge:norm()
	var myynorm = myyedge:norm()
	var boxxnorm = boxxedge:norm()
	var boxynorm = boxyedge:norm()
	myxedge = myxedge / myxnorm
	myyedge = myyedge / myynorm
	boxxedge = boxxedge / boxxnorm
	boxyedge = boxyedge / boxynorm
	if myxedge:dot(boxxedge) < myxedge:dot(boxyedge) then
		util.swap(myxedge, myyedge)
		util.swap(myxnorm, myynorm)
	end

	-- Compute bounds, draw uniform variables, do stacking
	var oneSidedYrange = 0.5*boxynorm + 0.5*myynorm - margin
	var yperturb : real
	if initialToZero then
		yperturb = boundedUniform(-oneSidedYrange, oneSidedYrange, {initialVal=0.0})
	else
		yperturb = boundedUniform(-oneSidedYrange, oneSidedYrange)
	end
	var disp = yperturb*boxyedge
	var mat = Mat4.translate(disp)
	self:transform(&mat)
end
terra QuadHex:stackRandomY(box: &QuadHex, margin: real, checkValidity: bool) : {}
	self:stackRandomY(box, margin, checkValidity, false)
end
QuadHex.methods.stackRandomY = pmethod(QuadHex.methods.stackRandomY)


---- The following methods assume that self is still in its original object space
---- (i.e. we just called makeBox and nothing else)

-- Shear along the local x axis, leaving the bottom face unchanged
terra QuadHex:shearX(angle: real)
	-- Translate bottom face to the origin, then shear Z onto X by tan(angle)
	var tana = ad.math.tan(angle)
	var botCentroid = self:botFace():centroid()
	var mat = Mat4.translate(botCentroid) * Mat4.shearZontoX(tana) * Mat4.translate(-botCentroid)
	self:topFace():transform(&mat)
end

-- Shear along the local y axis, leaving the bottom face unchanged
terra QuadHex:shearY(angle: real)
	-- Translate bottom face to the origin, then shear Z onto Y by tan(angle)
	var tana = ad.math.tan(angle)
	var botCentroid = self:botFace():centroid()
	var mat = Mat4.translate(botCentroid) * Mat4.shearZontoY(tana) * Mat4.translate(-botCentroid)
	self:topFace():transform(&mat)
end


--  These functions do what I ask of them, but they unfortunately do not compose
--  nicely. Namely, when we use shear to do the top-face tilting, it's impossible to shear in two
--  different directions and still end up with the top face being a rectangle. It'll always end up
--  as a non-rectangular parallelogram

-- Shear the top face along the local x axis, leaving the bottom face unchanged
terra QuadHex:topShearX(angle: real)
	util.assert(angle >= [-math.pi/2] and angle <= [math.pi/2],
		"Cannot shear top by an absolute magnitude of more than 90 degrees.\n")
	var tiltAmt = ad.math.tan(angle)

	-- -- Target tilt amount is tan(angle), but we might not be able to do that much
	-- -- We're limited by how far we can shear the top face before the shape starts to invert
	-- -- The following bounds are conservative, but correct.
	-- var frontLeftEdgeLen = self:frontLeftEdge():norm()
	-- var frontRightEdgeLen = self:frontRightEdge():norm()
	-- var backLeftEdgeLen = self:backLeftEdge():norm()
	-- var backRightEdgeLen = self:backRightEdge():norm()

	-- var invScale = 1.0/ad.math.fmax(0.5*self:topFrontEdge():norm(), 0.5*self:topBackEdge():norm())

	-- var tiltSign = 1.0; if tiltAmt < 0 then tiltSign = -1.0 end
	-- tiltAmt = ad.math.fabs(tiltAmt)
	-- tiltAmt = ad.math.fmin(tiltAmt, frontLeftEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, frontRightEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, backLeftEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, backRightEdgeLen*invScale)
	-- tiltAmt = tiltSign*tiltAmt

	-- Translate top face centroid to origin, then shear X onto Z
	var topCentroid = self:topFace():centroid()
	var mat = Mat4.translate(topCentroid) * Mat4.shearXontoZ(tiltAmt) * Mat4.translate(-topCentroid)
	self:topFace():transform(&mat)
end

-- Tilt the top face along the local y axis, leaving the bottom face unchanged
terra QuadHex:topShearY(angle: real)
	util.assert(angle >= [-math.pi/2] and angle <= [math.pi/2],
		"Cannot shear top by an absolute magnitude of more than 90 degrees.\n")
	var tiltAmt = ad.math.tan(angle)

	-- -- Target tilt amount is tan(angle), but we might not be able to do that much
	-- -- We're limited by how far we can shear the top face before the shape starts to invert
	-- -- The following bounds are conservative, but correct.
	-- var frontLeftEdgeLen = self:frontLeftEdge():norm()
	-- var frontRightEdgeLen = self:frontRightEdge():norm()
	-- var backLeftEdgeLen = self:backLeftEdge():norm()
	-- var backRightEdgeLen = self:backRightEdge():norm()

	-- var invScale = 1.0/ad.math.fmax(0.5*self:topLeftEdge():norm(), 0.5*self:topRightEdge():norm())

	-- var tiltSign = 1.0; if tiltAmt < 0 then tiltSign = -1.0 end
	-- tiltAmt = ad.math.fabs(tiltAmt)
	-- tiltAmt = ad.math.fmin(tiltAmt, frontLeftEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, frontRightEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, backLeftEdgeLen*invScale)
	-- tiltAmt = ad.math.fmin(tiltAmt, backRightEdgeLen*invScale)
	-- tiltAmt = tiltSign*tiltAmt

	-- Translate top face centroid to origin, then shear Y onto Z
	var topCentroid = self:topFace():centroid()
	var mat = Mat4.translate(topCentroid) * Mat4.shearYontoZ(tiltAmt) * Mat4.translate(-topCentroid)
	self:topFace():transform(&mat)
end


-- These functions also do what I ask, but have another composability problem: while successive tilts
-- will preserve the rectangularity of the top face, the other faces become non-planar. Basically, I've
-- learned that it's impossible to get everything I want here (unfortunately)

-- Rotate the top face about the y axis (causing a tilt along x)
terra QuadHex:tiltX(angle: real)
	var tana = ad.math.tan(angle)
	var topCentroid = self:topFace():centroid()
	var mat = Mat4.translate(topCentroid) * Mat4.rotateY(tana) * Mat4.translate(-topCentroid)
	self:topFace():transform(&mat)
end

-- Rotate the top face about the x axis (causing a tilt along y)
terra QuadHex:tiltY(angle: real)
	var tana = ad.math.tan(angle)
	var topCentroid = self:topFace():centroid()
	var mat = Mat4.translate(topCentroid) * Mat4.rotateX(tana) * Mat4.translate(-topCentroid)
	self:topFace():transform(&mat)
end

terra QuadHex:assertFacePlanarity()
	[(function()
		local t = {}
		for i=0,5 do
			table.insert(t, quote
				util.assert(self.faces[i]:isPlanar(), "QuadHex had non-planar face %d\n", i)
			end)
		end
		return t
	end)()]
end

terra QuadHex:saveContentsToFile(file: &C.FILE)
	-- C.fprintf(file, "%g %g %g   %g %g %g   %g %g %g   %g %g %g   %g %g %g   %g %g %g   %g %g %g   %g %g %g\n",
		C.fprintf(file, "%1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f   %1.10f %1.10f %1.10f\n",
		self.verts[0](0), self.verts[0](1), self.verts[0](2),
		self.verts[1](0), self.verts[1](1), self.verts[1](2),
		self.verts[2](0), self.verts[2](1), self.verts[2](2),
		self.verts[3](0), self.verts[3](1), self.verts[3](2),
		self.verts[4](0), self.verts[4](1), self.verts[4](2),
		self.verts[5](0), self.verts[5](1), self.verts[5](2),
		self.verts[6](0), self.verts[6](1), self.verts[6](2),
		self.verts[7](0), self.verts[7](1), self.verts[7](2))
end

terra QuadHex:saveToFile(file: &C.FILE) : {}
	C.fprintf(file, "QuadHex\n")
	self:saveContentsToFile(file)
end
inheritance.virtual(QuadHex, "saveToFile")

terra QuadHex:loadFromFile(file: &C.FILE)
	var buf : int8[1024]
	C.fgets(buf, 1023, file)
	self.verts[0](0) = C.atof(C.strtok(buf, " "))
	self.verts[0](1) = C.atof(C.strtok(nil, " "))
	self.verts[0](2) = C.atof(C.strtok(nil, " "))
	self.verts[1](0) = C.atof(C.strtok(nil, " "))
	self.verts[1](1) = C.atof(C.strtok(nil, " "))
	self.verts[1](2) = C.atof(C.strtok(nil, " "))
	self.verts[2](0) = C.atof(C.strtok(nil, " "))
	self.verts[2](1) = C.atof(C.strtok(nil, " "))
	self.verts[2](2) = C.atof(C.strtok(nil, " "))
	self.verts[3](0) = C.atof(C.strtok(nil, " "))
	self.verts[3](1) = C.atof(C.strtok(nil, " "))
	self.verts[3](2) = C.atof(C.strtok(nil, " "))
	self.verts[4](0) = C.atof(C.strtok(nil, " "))
	self.verts[4](1) = C.atof(C.strtok(nil, " "))
	self.verts[4](2) = C.atof(C.strtok(nil, " "))
	self.verts[5](0) = C.atof(C.strtok(nil, " "))
	self.verts[5](1) = C.atof(C.strtok(nil, " "))
	self.verts[5](2) = C.atof(C.strtok(nil, " "))
	self.verts[6](0) = C.atof(C.strtok(nil, " "))
	self.verts[6](1) = C.atof(C.strtok(nil, " "))
	self.verts[6](2) = C.atof(C.strtok(nil, " "))
	self.verts[7](0) = C.atof(C.strtok(nil, " "))
	self.verts[7](1) = C.atof(C.strtok(nil, " "))
	self.verts[7](2) = C.atof(C.strtok(nil, " "))
end

Shape.addLoader("QuadHex", terra(file: &C.FILE) : &Shape
	var qhex = QuadHex.heapAlloc()
	qhex:loadFromFile(file)
	return qhex
end)

m.addConstructors(QuadHex)



-- A cuboid
-- Some things are simpler/cheaper for cuboids, so we have a special class
-- NOTE: This is only a Box by convention. It is totally possible to modify the
--    underlying vertices so that it's no longer a Box. Don't do that, stupid.
local struct Box {}
inheritance.dynamicExtend(QuadHex, Box)

terra Box:volume() : real
	-- Multiply the length of three mututally orthogonal edges
	return (self.verts[ [QuadHex.vFrontTopLeft] ] - self.verts[ [QuadHex.vFrontBotLeft] ]):norm() *
		   (self.verts[ [QuadHex.vFrontBotRight] ] - self.verts[ [QuadHex.vFrontBotLeft] ]):norm() *
		   (self.verts[ [QuadHex.vBackBotLeft] ] - self.verts[ [QuadHex.vFrontBotLeft] ]):norm()
end
inheritance.virtual(Box, "volume")

terra Box:saveToFile(file: &C.FILE) : {}
	C.fprintf(file, "Box\n")
	self:saveContentsToFile(file)
end
inheritance.virtual(Box, "saveToFile")

Shape.addLoader("Box", terra(file: &C.FILE) : &Shape
	var box = Box.heapAlloc()
	box:loadFromFile(file)
	return box
end)

m.addConstructors(Box)





return
{
	QuadHex = QuadHex,
	Box = Box
}

end)








