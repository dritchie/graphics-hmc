terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local inheritance = terralib.require("inheritance")
local Vec = terralib.require("linalg").Vec
local BBox = terralib.require("bbox")
local Vector = terralib.require("vector")
local lpsolve = terralib.require("lpsolve")
local s3dCore = terralib.require("s3dCore")

local C = terralib.includecstring [[
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
]]

local Vec3d = Vec(double, 3)


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

	-- Important: add this connection to each body's internal list
	body1:addConnection(self)
	body2:addConnection(self)
end

terra ContactPoint:__destruct() : {}
	-- Can do nothing, but we need to implement it since
	--    the base class destructor is pure virtual.
end
inheritance.virtual(ContactPoint, "__destruct")

ContactPoint.methods.fsign = macro(function(self, body)
	return quote
		var sign = 1.0
		if body == self.body1 then sign = -1.0 end
	in
		sign
	end
end)

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
	var fforce = f.force
	if self.body2.active then
		f.force = self:fsign(self.body2)*fforce
		self.body2:applyForce(&f)
	end
	if self.body1.active then
		f.force = self:fsign(self.body1)*fforce
		self.body1:applyForce(&f)
	end
end
inheritance.virtual(ContactPoint, "applyForcesImpl")

-- LP stability stuff
if real == double then

	terra ContactPoint:numLPVars() : int
		-- Normal force and two tangent forces
		return 3
	end
	inheritance.virtual(ContactPoint, "numLPVars")

	ContactPoint.methods.normalForceID = macro(function(self)
		return `self.firstLPVarID
	end)
	ContactPoint.methods.tangentForce1ID = macro(function(self)
		return `self.firstLPVarID + 1
	end)
	ContactPoint.methods.tangentForce2ID = macro(function(self)
		return `self.firstLPVarID + 2
	end)

	terra ContactPoint:addStabilityLPConstraints(lp: &lpsolve._lprec) : {}
		-- Variables default to a lower bound of zero, so we need to
		--    explicitly make the tangent forces unbounded
		lpsolve.set_unbounded(lp, self:tangentForce1ID())
		lpsolve.set_unbounded(lp, self:tangentForce2ID())

		-- Now introduce the friction bound constraints
		-- (one lower bound, one upper bound for each tangent force)
		var friction = self.body1.friction * self.body2.friction
		var indices : int[2]
		var coeffs : double[2]
		-- Bounds for tangent force 1
		indices[0] = self:tangentForce1ID()
		indices[1] = self:normalForceID()
		coeffs[0] = 1.0
		coeffs[1] = friction
		lpsolve.add_constraintex(lp, 2, &coeffs[0], &indices[0], lpsolve.GE, 0.0)
		indices[0] = self:tangentForce1ID()
		indices[1] = self:normalForceID()
		coeffs[0] = 1.0
		coeffs[1] = -friction
		lpsolve.add_constraintex(lp, 2, &coeffs[0], &indices[0], lpsolve.LE, 0.0)
		-- Bounds for tangent force 2
		indices[0] = self:tangentForce2ID()
		indices[1] = self:normalForceID()
		coeffs[0] = 1.0
		coeffs[1] = friction
		lpsolve.add_constraintex(lp, 2, &coeffs[0], &indices[0], lpsolve.GE, 0.0)
		indices[0] = self:tangentForce2ID()
		indices[1] = self:normalForceID()
		coeffs[0] = 1.0
		coeffs[1] = -friction
		lpsolve.add_constraintex(lp, 2, &coeffs[0], &indices[0], lpsolve.LE, 0.0)
	end
	inheritance.virtual(ContactPoint, "addStabilityLPConstraints")

	terra ContactPoint:forceCoeffsForBody(body: &Body, indices: &Vector(int), coeffs: &Vector(Vec3d)) : {}
		indices:push(self:normalForceID())
		indices:push(self:tangentForce1ID())
		indices:push(self:tangentForce2ID())
		coeffs:push(self:fsign(body) * self.normal)
		coeffs:push(self:fsign(body) * self.tangent1)
		coeffs:push(self:fsign(body) * self.tangent2)
	end
	inheritance.virtual(ContactPoint, "forceCoeffsForBody")

	terra ContactPoint:torqueCoeffsForBody(body: &Body, indices: &Vector(int), coeffs: &Vector(Vec3d)) : {}
		indices:push(self:normalForceID())
		indices:push(self:tangentForce1ID())
		indices:push(self:tangentForce2ID())
		var v = self.point - body:centerOfMass()
		coeffs:push(v:cross(self:fsign(body) * self.normal))
		coeffs:push(v:cross(self:fsign(body) * self.tangent1))
		coeffs:push(v:cross(self:fsign(body) * self.tangent2))
	end
	inheritance.virtual(ContactPoint, "torqueCoeffsForBody")

end

m.addConstructors(ContactPoint)





local alignThresh = 1e-7
local RectContactFace = Face(4)

-- A RectRectContact joins two bodies at two coincident faces.
-- Both faces must be:
--    * Parallelograms
--    * Co-planar
--    * Aligned (both edge directions parallel)
-- Internally, the contact is represented as four ContactPoints--
--    one for each vertex of the contact polygon.
local struct RectRectContact
{
	contactPoints: (&ContactPoint)[4],
	face1: &RectContactFace,
	face2: &RectContactFace
}
inheritance.dynamicExtend(Connection, RectRectContact)


RectRectContact.methods.isValidContact = terra(face1: &RectContactFace, face2: &RectContactFace)
	-- Check parallelogram-ness
	if not face1:isParallelogram() or not face2:isParallelogram() then return false end
	-- Check alignment
	var e1 = face1:vertex(1) - face1:vertex(0); e1:normalize()
	var e2a = face2:vertex(1) - face2:vertex(0); e2a:normalize()
	var e2b = face2:vertex(2) - face2:vertex(1); e2b:normalize()
	var aligned = ad.math.fabs(e1:dot(e2a)) < alignThresh or
		   		  ad.math.fabs(e1:dot(e2b)) < alignThresh
	-- C.printf("alignment: %g   |   %g\n", e1:dot(e2a), e1:dot(e2b))
	if not aligned then return false end
	-- Check coplanarity
	var n = e2a:cross(e2b)
	-- var e = face1:vertex(0) - face2:vertex(0); e:normalize()
	-- var d = e:dot(n); C.printf("plane eqn: %g\n", d)
	if not face1:vertex(0):inPlane(face2:vertex(0), n) then return false end
	return true
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
			"Can only create RectRectContact between two coplanar, aligned, parallelogram faces\n")
	end

	-- Compute contact polygon, create 4 contact points
	var cp1, cp2, cp3, cp4 = RectRectContact.contactPoints(face1, face2)
	var n = face1:normal()
	var t1 = face1:vertex(1) - face1:vertex(0); t1:normalize()
	var t2 = face1:vertex(3) - face1:vertex(0); t2:normalize()
	self.contactPoints[0] = ContactPoint.heapAlloc(body1, body2, cp1, n, t1, t2)
	self.contactPoints[1] = ContactPoint.heapAlloc(body1, body2, cp2, n, t1, t2)
	self.contactPoints[2] = ContactPoint.heapAlloc(body1, body2, cp3, n, t1, t2)
	self.contactPoints[3] = ContactPoint.heapAlloc(body1, body2, cp4, n, t1, t2)

	self.face1 = face1
	self.face2 = face2
end

terra RectRectContact:recalculate() : {}
	var face1 = self.face1
	var face2 = self.face2

	var cp1, cp2, cp3, cp4 = RectRectContact.contactPoints(face1, face2)
	var n = face1:normal()
	var t1 = face1:vertex(1) - face1:vertex(0); t1:normalize()
	var t2 = face1:vertex(3) - face1:vertex(0); t2:normalize()

	self.contactPoints[0].point = cp1; self.contactPoints[0].normal = n; self.contactPoints[0].tangent1 = t1; self.contactPoints[0].tangent2 = t2
	self.contactPoints[1].point = cp2; self.contactPoints[1].normal = n; self.contactPoints[1].tangent1 = t1; self.contactPoints[1].tangent2 = t2
	self.contactPoints[2].point = cp3; self.contactPoints[2].normal = n; self.contactPoints[2].tangent1 = t1; self.contactPoints[2].tangent2 = t2
	self.contactPoints[3].point = cp4; self.contactPoints[3].normal = n; self.contactPoints[3].tangent1 = t1; self.contactPoints[3].tangent2 = t2

end
inheritance.virtual(RectRectContact, "recalculate")

terra RectRectContact:__destruct() : {}
	m.delete(self.contactPoints[0])
	m.delete(self.contactPoints[1])
	m.delete(self.contactPoints[2])
	m.delete(self.contactPoints[3])
end
inheritance.virtual(RectRectContact, "__destruct")

terra RectRectContact:applyForcesImpl() : {}
	self.contactPoints[0]:applyForcesImpl()
	self.contactPoints[1]:applyForcesImpl()
	self.contactPoints[2]:applyForcesImpl()
	self.contactPoints[3]:applyForcesImpl()
end
inheritance.virtual(RectRectContact, "applyForcesImpl")


local terra faceToIndex(body: &Body, face: &RectContactFace)
	var faces = [Vector(&RectContactFace)].stackAlloc()
	body.shape:getQuadFaces(&faces)
	var index = -1
	for i=0,faces.size do
		if faces(i) == face then
			index = i
			break
		end
	end
	m.destruct(faces)
	return index
end
terra RectRectContact:saveToFile(file: &C.FILE) : {}
	C.fprintf(file, "RectRectContact\n")
	var body1 = self.contactPoints[0].body1
	var body2 = self.contactPoints[0].body2
	C.fprintf(file, "%d %d   %d %d\n",
		body1.index, faceToIndex(body1, self.face1),
		body2.index, faceToIndex(body2, self.face2))
end
inheritance.virtual(RectRectContact, "saveToFile")

Connection.addLoader("RectRectContact", terra(file: &C.FILE, bodies: &Vector(&Body)) : &Connection
	var buf : int8[1024]
	C.fgets(buf, 1023, file)
	var index1 = C.atoi(C.strtok(buf, " "))
	var faceIndex1 = C.atoi(C.strtok(nil, " "))
	var index2 = C.atoi(C.strtok(nil, " "))
	var faceIndex2 = C.atoi(C.strtok(nil, " "))
	var body1 = bodies(index1)
	var body2 = bodies(index2)
	var faces = [Vector(&RectContactFace)].stackAlloc()
	body1.shape:getQuadFaces(&faces)
	var face1 = faces(faceIndex1)
	faces:clear()
	body2.shape:getQuadFaces(&faces)
	var face2 = faces(faceIndex2)
	m.destruct(faces)
	return RectRectContact.heapAlloc(body1, body2, face1, face2, false)
end)

-- LP stability stuff
if real == double then

	terra RectRectContact:setFirstLPVarID(id: int): {}
		var nextid = id
		self.contactPoints[0]:setFirstLPVarID(nextid)
		nextid = nextid + self.contactPoints[0]:numLPVars()
		self.contactPoints[1]:setFirstLPVarID(nextid)
		nextid = nextid + self.contactPoints[1]:numLPVars()
		self.contactPoints[2]:setFirstLPVarID(nextid)
		nextid = nextid + self.contactPoints[2]:numLPVars()
		self.contactPoints[3]:setFirstLPVarID(nextid)
	end
	inheritance.virtual(RectRectContact, "setFirstLPVarID")

	terra RectRectContact:numLPVars() : int
		return self.contactPoints[0]:numLPVars() +
			   self.contactPoints[1]:numLPVars() +
			   self.contactPoints[2]:numLPVars() +
			   self.contactPoints[3]:numLPVars() 
	end
	inheritance.virtual(RectRectContact, "numLPVars")

	terra RectRectContact:addStabilityLPConstraints(lp: &lpsolve._lprec) : {}
		self.contactPoints[0]:addStabilityLPConstraints(lp)
		self.contactPoints[1]:addStabilityLPConstraints(lp)
		self.contactPoints[2]:addStabilityLPConstraints(lp)
		self.contactPoints[3]:addStabilityLPConstraints(lp)
	end
	inheritance.virtual(RectRectContact, "addStabilityLPConstraints")

	terra RectRectContact:forceCoeffsForBody(body: &Body, indices: &Vector(int), coeffs: &Vector(Vec3d)) : {}
		self.contactPoints[0]:forceCoeffsForBody(body, indices, coeffs)
		self.contactPoints[1]:forceCoeffsForBody(body, indices, coeffs)
		self.contactPoints[2]:forceCoeffsForBody(body, indices, coeffs)
		self.contactPoints[3]:forceCoeffsForBody(body, indices, coeffs)
	end
	inheritance.virtual(RectRectContact, "forceCoeffsForBody")

	terra RectRectContact:torqueCoeffsForBody(body: &Body, indices: &Vector(int), coeffs: &Vector(Vec3d)) : {}
		self.contactPoints[0]:torqueCoeffsForBody(body, indices, coeffs)
		self.contactPoints[1]:torqueCoeffsForBody(body, indices, coeffs)
		self.contactPoints[2]:torqueCoeffsForBody(body, indices, coeffs)
		self.contactPoints[3]:torqueCoeffsForBody(body, indices, coeffs)
	end
	inheritance.virtual(RectRectContact, "torqueCoeffsForBody")

end

m.addConstructors(RectRectContact)



-- TODO: PolyRectContact (i.e. one face is rectangular and the other is not,
--    but its oriented bbox must be contained within the rectangular face.)

return
{
	ContactPoint = ContactPoint,
	RectRectContact = RectRectContact
}

end)





