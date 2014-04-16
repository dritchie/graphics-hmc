terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local inheritance = terralib.require("inheritance")
local Vec = terralib.require("linalg").Vec
local BBox = terralib.require("bbox")
local Vector = terralib.require("vector")
local s3dCore = terralib.require("s3dCore")

local C = terralib.includecstring [[
#include "stdio.h"
]]

return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)




----- CONNECTIONS


-- A ContactPoint generates a compressive normal force and
--    two friction-bounded tangent forces between two bodies at
--    a single point.
local struct ContactPoint
{
	body1: &Body,
	body2: &Body,
	point: Vec3,
	normal: Vec3,	-- assumed outward facing from body1
	tangent1: Vec3,
	tangent2: Vec3

}
inheritance.dynamicExtend(Connection, ContactPoint)

terra ContactPoint:__construct(body1: &Body, body2: &Body, point: Vec3, normal: Vec3, tangent1: Vec3, tangent2: Vec3)
	self.body1 = body1
	self.body2 = body2
	self.point = point
	self.normal = normal
	self.tangent1 = tangent1
	self.tangent2 = tangent2
end

terra ContactPoint:applyForcesImpl() : {}
	var friction = self.body1.friction * self.body2.friction
	var f : Force
	-- Only generate forces if one or more bodies is active
	if self.body1.active or self.body2.active then
		f = Force.lowerBoundedRandomLineForce(self.point, self.normal, 1e-14)
		var frictionBound = friction*f.force:norm()
		var tf1 = Force.boundedRandomLineForce(self.point, self.tangent1, -frictionBound, frictionBound)
		var tf2 = Force.boundedRandomLineForce(self.point, self.tangent2, -frictionBound, frictionBound)
		f:combineWith(&tf1)
		f:combineWith(&tf2)
	end
	if self.body2.active then
		self.body2:applyForce(&f)
	end
	if self.body1.active then
		f.force = -f.force
		self.body1:applyForce(&f)
	end
end
inheritance.virtual(ContactPoint, "applyForcesImpl")

m.addConstructors(ContactPoint)





local validThresh = 1e-16
local RectContactFace = Face(4)

-- A RectRectContact joins two bodies at two coincident faces.
-- Both faces must be:
--    * Rectangular
--    * Co-planar
--    * Aligned (both edge directions parallel)
-- Internally, the contact is represented as four ContactPoints--
--    one for each vertex of the contact polygon.
local struct RectRectContact
{
	contactPoints: ContactPoint[4]
}
inheritance.dynamicExtend(Connection, RectRectContact)


RectRectContact.methods.isValidContact = terra(face1: &RectContactFace, face2: &RectContactFace)
	-- Check rectangularity
	if not face1:isRectangular() or not face2:isRectangular() then return false end
	-- Check alignment
	var e1 = face1:vertex(1) - face1:vertex(0); e1:normalize()
	var e2a = face2:vertex(1) - face2:vertex(0); e2a:normalize()
	var e2b = face2:vertex(2) - face2:vertex(1); e2b:normalize()
	var aligned = ad.math.fabs(e1:dot(e2a)) < validThresh or
		   		  ad.math.fabs(e1:dot(e2b)) < validThresh
	if not aligned then return false end
	-- Check coplanarity
	var n = e2a:cross(e2b)
	return face1:vertex(0):inPlane(face2:vertex(0), n)
end

local Vec2 = Vec(real, 2)
local fwdXform = macro(function(point, origin, xaxis, yaxis)
	return quote
		var v = point - origin
	in
		Vec2.stackAlloc(v:dot(xaxis), v:dot(yaxis))
	end
end)
local invXform = macro(function(point, origin, xaxis, yaxis)
	return `origin + point(0)*xaxis + point(1)*yaxis
end)
-- Assumes that the faces pass the validity check
RectRectContact.methods.contactPoints = terra(face1: &RectContactFace, face2: &RectContactFace)
	-- Pick point to be origin, pick two axes
	var origin = face1:vertex(0)
	var xaxis = face1:vertex(1) - face1:vertex(0); xaxis:normalize()
	var yaxis = face1:vertex(3) - face1:vertex(0); yaxis:normalize()

	-- Compute AABBs for each face in this coordinate system
	var f1bbox = [BBox(Vec2)].stackAlloc()
	f1bbox:expand(fwdXform(face1:vertex(0), origin, xaxis, yaxis))
	f1bbox:expand(fwdXform(face1:vertex(1), origin, xaxis, yaxis))
	f1bbox:expand(fwdXform(face1:vertex(2), origin, xaxis, yaxis))
	f1bbox:expand(fwdXform(face1:vertex(3), origin, xaxis, yaxis))
	var f2bbox = [BBox(Vec2)].stackAlloc()
	f2bbox:expand(fwdXform(face2:vertex(0), origin, xaxis, yaxis))
	f2bbox:expand(fwdXform(face2:vertex(1), origin, xaxis, yaxis))
	f2bbox:expand(fwdXform(face2:vertex(2), origin, xaxis, yaxis))
	f2bbox:expand(fwdXform(face2:vertex(3), origin, xaxis, yaxis))

	-- Intersect BBoxs
	f1bbox:intersectWith(&f2bbox)

	-- Transform 2D points back into original 3D space
	return invXform(Vec2.stackAlloc(f1bbox.mins(0), f1bbox.mins(1)), origin, xaxis, yaxis),
		   invXform(Vec2.stackAlloc(f1bbox.maxs(0), f1bbox.mins(1)), origin, xaxis, yaxis),
		   invXform(Vec2.stackAlloc(f1bbox.maxs(0), f1bbox.maxs(1)), origin, xaxis, yaxis),
		   invXform(Vec2.stackAlloc(f1bbox.mins(0), f1bbox.maxs(1)), origin, xaxis, yaxis)

end

-- Caller can forgo valid check if the input faces are guaranteed to be valid
-- (i.e. by construction)
terra RectRectContact:__construct(body1: &Body, body2: &Body, face1: &RectContactFace, face2: &RectContactFace, validCheck: bool)
	if validCheck then
		util.assert(RectRectContact.isValidContact(face1, face2),
			"Can only create Contact between two aligned, rectangular faces\n")
	end

	-- Compute contact polygon, create 4 contact points
	var cp1, cp2, cp3, cp4 = RectRectContact.contactPoints(face1, face2)
	var n = face1:normal()
	var t1 = face1:vertex(1) - face1:vertex(0); t1:normalize()
	var t2 = face1:vertex(3) - face1:vertex(0); t2:normalize()
	self.contactPoints[0] = ContactPoint.stackAlloc(body1, body2, cp1, n, t1, t2)
	self.contactPoints[1] = ContactPoint.stackAlloc(body1, body2, cp2, n, t1, t2)
	self.contactPoints[2] = ContactPoint.stackAlloc(body1, body2, cp3, n, t1, t2)
	self.contactPoints[3] = ContactPoint.stackAlloc(body1, body2, cp4, n, t1, t2)
end

terra RectRectContact:applyForcesImpl() : {}
	self.contactPoints[0]:applyForcesImpl()
	self.contactPoints[1]:applyForcesImpl()
	self.contactPoints[2]:applyForcesImpl()
	self.contactPoints[3]:applyForcesImpl()
end
inheritance.virtual(RectRectContact, "applyForcesImpl")

m.addConstructors(RectRectContact)


-- TODO: PolyRectContact (i.e. one face is rectangular and the other is not,
--    but its oriented bbox must be contained within the rectangular face.)

return
{
	RectRectContact = RectRectContact
}

end)





