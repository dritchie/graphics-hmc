terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local inheritance = terralib.require("inheritance")
local s3dCore = terralib.require("s3dCore")


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
		f = Force.lowerBoundedRandomLineForce(self.point, self.normal, 1e-15)
		var frictionBound = friction*f:norm()
		var tf1 = Force.boundedRandomLineForce(self.point, self.tangent1, -frictionBound, frictionBound)
		var tf2 = Force.boundedRandomLineForce(self.point, self.tangent2, -frictionBound, frictionBound)
		f:combineWith(tf1)
		f:combineWith(tf2)
	end
	if self.body1.active then
		self.body1:applyForce(f)
	end
	if self.body2.active then
		f.force = -f.force
		self.body2:applyForce(f)
	end
end
inheritance.virtual(ContactPoint, "applyForcesImpl")

m.addConstructors(ContactPoint)



-- A Contact joins two bodies at two coincident faces.
-- For ease of parameterization / ensuring fixed-dimensionality
--    and continuous-ness during inference, the two faces must be:
--    * Rectangular
--    * Co-planar
--    * Aligned (both edge directions parallel)
-- Internally, the Contact is represented as four ContactPoints--
--    one for each vertex of the contact polygon.
local ContactFace = Face(4)
local struct Contact
{
	contactPoints: ContactPoint[4]
}
inheritance.dynamicExtend(Connection, Contact)

local validThresh = 1e-16
Contact.methods.isValidContact = terra(face1: &ContactFace, face2: &ContactFace)
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

-- Caller can forgo valid check if the input faces are guaranteed to be valid
-- (e.g. by construction)
terra Contact:__construct(body1: &Body, body2: &Body, face1: &ContactFace, face2: &ContactFace, validCheck: bool) : {}
	if validCheck then
		util.assert(Contact.isValidContact(face1, face2),
			"Can only create Contact between two aligned, rectangular faces\n")
	end
	-- TODO: Compute contact polygon, create 4 contact points
end

-- Do the valid check by default, though
terra Contact:__construct(body1: &Body, body2: &Body, face1: &ContactFace, face2: &ContactFace) : {}
	self:__construct(body1, body2, face1, face2, true)
end

terra Contact:applyForcesImpl() : {}
	self.contactPoints[0]:applyForcesImpl()
	self.contactPoints[1]:applyForcesImpl()
	self.contactPoints[2]:applyForcesImpl()
	self.contactPoints[3]:applyForcesImpl()
end
inheritance.virtual(Contact, "applyForcesImpl")

m.addConstructors(Contact)

return
{
	Contact = Contact
}

end)