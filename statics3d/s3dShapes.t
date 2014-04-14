terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local ad = terralib.require("ad")
local inheritance = terralib.require("inheritance")
local s3dCore = terralib.require("s3dCore")
local s3dRendering = terralib.require("s3dRendering")


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)
local rendering = s3dRendering(pcomp)
util.importAll(rendering)




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
	return `self.faces[QuadHex.fFront]
end)
QuadHex.methods.backFace = macro(function(self)
	return `self.faces[QuadHex.fBack]
end)
QuadHex.methods.topFace = macro(function(self)
	return `self.faces[QuadHex.fTop]
end)
QuadHex.methods.botFace = macro(function(self)
	return `self.faces[QuadHex.fBot]
end)
QuadHex.methods.leftFace = macro(function(self)
	return `self.faces[QuadHex.fLeft]
end)
QuadHex.methods.rightFace = macro(function(self)
	return `self.faces[QuadHex.fRight]
end)

terra QuadHex:__construct() : {}
	[QuadHex.ParentClass].__construct(self)

	-- We default to a unit cube at the origin in a z-up coordinate system.

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

-- TODO: Switch this to use proper transformation matrices, once I write those?
-- (Or even eliminate it entirely?)
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

terra QuadHex:render(settings: &RenderSettings) : {}
	self.faces[0]:render(settings)
	self.faces[1]:render(settings)
	self.faces[2]:render(settings)
	self.faces[3]:render(settings)
	self.faces[4]:render(settings)
	self.faces[5]:render(settings)
end
inheritance.virtual(QuadHex, "render")

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

m.addConstructors(Box)


return
{
	QuadHex = QuadHex,
	Box = Box
}

end)








