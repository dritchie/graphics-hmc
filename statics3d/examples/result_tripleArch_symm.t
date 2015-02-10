require("prob")


local s3dLib = require("s3dLib")
local m = require("mem")
local util = require("util")
local ad = require("ad")
local templatize = require("templatize")
local AutoPtr = require("autopointer")
local Vector = require("Vector")
local Vec = require("linalg").Vec
local Vec3d = Vec(double, 3)
local glutils = require("glutils")
util.importEntries(glutils, "Camera", "Light")


local C = terralib.includecstring [[
#include "stdio.h"
]]


local mm = macro(function(x)
	return `0.001*x
end)
local radians = macro(function(deg)
	return `deg*[math.pi]/180.0
end)
local lerp = macro(function(a,b,t)
	return `(1.0-t)*a + t*b
end)


------------------------------------------------------------------------------------------


local BezCurve = templatize(function(VecT)
	local real = VecT.RealType
	local struct BezCurveT
	{
		points: VecT[2],
		tangents: VecT[2]
	}

	terra BezCurveT:__construct(sp: VecT, ep: VecT, st: VecT, et: VecT) : {}
		self.points[0] = sp
		self.points[1] = ep
		self.tangents[0] = st
		self.tangents[1] = et
	end

	terra BezCurveT:__construct(ep: VecT, st: VecT, et: VecT) : {}
		self:__construct(VecT.stackAlloc(0.0), ep, st, et)
	end

	terra BezCurveT:__construct(st: VecT, et: VecT) : {}
		self:__construct(VecT.stackAlloc(0.0), st, et)
	end

	terra BezCurveT:__construct() : {}
		self:__construct(VecT.stackAlloc(0.0), VecT.stackAlloc(0.0))
	end

	terra BezCurveT:eval(t: real)
		var p0 = self.points[0]
		var p3 = self.points[1]
		var p1 = p0 + self.tangents[0]
		var p2 = p3 + self.tangents[1]
		var p01 = lerp(p0, p1, t)
		var p12 = lerp(p1, p2, t)
		var p23 = lerp(p2, p3, t)
		var p012 = lerp(p01, p12, t)
		var p123 = lerp(p12, p23, t)
		var point = lerp(p012, p123, t)
		var tangent = p123 - p012
		return point, tangent
	end

	if VecT.Dimension == 2 then
		terra BezCurveT:normal(t: real)
			var point, tang = self:eval(t)
			tang:normalize()
			util.swap(tang(0), tang(1))
			tang(0) = -tang(0)
			return tang
		end
	end
	if VecT.Dimension == 3 then
		terra BezCurveT:normal(t: real)
			var point, tang = self:eval(t)
			tang:normalize()
			util.swap(tang(0), tang(2))
			tang(0) = -tang(0)
			return tang
		end
	end

	terra BezCurveT:print()
		C.printf("points: ")
		self.points[0]:print()
		C.printf(", ")
		self.points[1]:print()
		C.printf(" | tangents: ")
		self.tangents[0]:print()
		C.printf(", ")
		self.tangents[1]:print()
	end

	terra BezCurveT:println()
		self:print()
		C.printf("\n")
	end

	m.addConstructors(BezCurveT)
return BezCurveT
end)


local Vec2d = Vec(double, 2)
local BezCurve2d = BezCurve(Vec2d)


local struct CurveConnection
{
	from: &BezCurve2d,			-- Which curve?
	fromIndex: int,
	whereFrom: uint,			-- Which endpoint on the curve?
	to: &BezCurve2d,			-- Which curve?
	toIndex: int,				-- (Can be -1 if ground)
	connectToEnd: bool,			-- Should we connect to the end of the curve?
	union {
		toEndpoint: uint,		-- Which endpoint to connect to?
		whereTo: double			-- How far along the curve to connect?
	}		
}


local struct CurveNetwork
{
	ground: &BezCurve2d,
	curves: Vector(&BezCurve2d),
	connections: Vector(CurveConnection),
	curve2conn: Vector(Vector(uint)),
	nblocks: Vector(uint)
}

terra CurveNetwork:__construct(groundHeight: double, groundWidth: double)
	-- Ground is just a straight line bezier curve
	self.ground = BezCurve2d.heapAlloc(Vec2d.stackAlloc(-0.5*groundWidth, groundHeight),
									   Vec2d.stackAlloc(0.5*groundWidth, groundHeight),
									   Vec2d.stackAlloc((1.0/3)*groundWidth, 0.0),
									   Vec2d.stackAlloc(-(1.0/3)*groundWidth, 0.0))
	m.init(self.curves)
	m.init(self.connections)
	m.init(self.curve2conn)
	m.init(self.nblocks)
end

terra CurveNetwork:__destruct()
	m.delete(self.ground)
	self.curves:clearAndDelete()
	m.destruct(self.curves)
	m.destruct(self.connections)
	m.destruct(self.curve2conn)
	m.destruct(self.nblocks)
end

terra CurveNetwork:addCurve(bc: &BezCurve2d, nblocks: uint)
	self.curves:push(bc)
	self.curve2conn:resize(self.curves.size)
	self.nblocks:push(nblocks)
end

terra CurveNetwork:addConnection(cc: CurveConnection)
	-- Connections must be 'acyclic' in the sense that all connections must go from a curve of index i
	--    to a curve of index j < i (includes ground at index -1). Correct construction of block stacks
	--    depends on this.
	util.assert(cc.toIndex < cc.fromIndex,
		"Attempt to connect curve %d to curve %d; can only connect with curves of index < %d\n",
		cc.fromIndex, cc.toIndex, cc.fromIndex)
	self.connections:push(cc)
	self.curve2conn(cc.fromIndex):push(self.connections.size-1)
end

terra CurveNetwork:addConnectionToCurve(fromIndex: int, whereFrom: uint, toIndex: int, whereTo: double)
	var conn : CurveConnection
	conn.from = self.curves(fromIndex)
	conn.fromIndex = fromIndex
	conn.whereFrom = whereFrom
	conn.to = self.curves(toIndex)
	conn.toIndex = toIndex
	conn.connectToEnd = false
	conn.whereTo = whereTo
	self:addConnection(conn)
end

terra CurveNetwork:addConnectionToCurveEndpoint(fromIndex: int, whereFrom: uint, toIndex: int, whichEndpoint: uint)
	var conn : CurveConnection
	conn.from = self.curves(fromIndex)
	conn.fromIndex = fromIndex
	conn.whereFrom = whereFrom
	conn.to = self.curves(toIndex)
	conn.toIndex = toIndex
	conn.connectToEnd = true
	conn.toEndpoint = whichEndpoint
	self:addConnection(conn)
end

terra CurveNetwork:addConnectionToGround(fromIndex: int, whereFrom: uint, whereTo: double)
	var conn : CurveConnection
	conn.from = self.curves(fromIndex)
	conn.fromIndex = fromIndex
	conn.whereFrom = whereFrom
	conn.to = self.ground
	conn.toIndex = -1
	conn.whereTo = whereTo
	conn.connectToEnd = false
	self:addConnection(conn)
end

terra CurveNetwork:fixEndpoints(i: uint) : {}
	var curve = self.curves(i)
	var conns = self.curve2conn:getPointer(i)
	for j=0,conns.size do
		var connIndex = conns(j)
		var conn = self.connections(connIndex)
		if conn.connectToEnd then
			curve.points[conn.whereFrom] = conn.to.points[conn.toEndpoint]
		else
			var point, tangent = conn.to:eval(conn.whereTo)
			curve.points[conn.whereFrom] = point
		end
	end
end

terra CurveNetwork:fixEndpoints() : {}
	for i=0,self.curves.size do
		self:fixEndpoints(i)
	end
end

m.addConstructors(CurveNetwork)


------------------------------------------------------------------------------------------

-- Global instance of a curve network that all inference will use
local curvenet = global(CurveNetwork)
local terra initGlobalCurveNet()
	var groundWidth = mm(1000.0)
	curvenet = CurveNetwork.stackAlloc(0.0, groundWidth)

	-- -- Ensuring that the ground interpolates as linear interpolation
	-- var p0, t0 = curvenet.ground:eval(0.0)
	-- var pq1, tq1 = curvenet.ground:eval(0.25)
	-- var phalf, thalf = curvenet.ground:eval(0.5)
	-- var pq3, tq3 = curvenet.ground:eval(0.75)
	-- var p1, t1 = curvenet.ground:eval(1.0)
	-- C.printf("ground(0): %g  (should be %g)\n", p0(0), -0.5*groundWidth)
	-- C.printf("ground(0.25): %g  (should be %g)\n", pq1(0), -0.25*groundWidth)
	-- C.printf("ground(0.5): %g  (should be %g)\n", phalf(0), 0.0)
	-- C.printf("ground(0.75): %g  (should be %g)\n", pq3(0), 0.25*groundWidth)
	-- C.printf("ground(1): %g  (should be %g)\n", p1(0), 0.5*groundWidth)

	-- TRIPLE ARCH

	-- 0: A small arch on the ground
	curvenet:addCurve(BezCurve2d.heapAlloc(Vec2d.stackAlloc(0.0, mm(300.0)),
										  Vec2d.stackAlloc(0.0, mm(300.0))),
					  9)
	curvenet:addConnectionToGround(0, 0, 0.18)
	curvenet:addConnectionToGround(0, 1, 0.43)

	-- 1: Another small arch on the ground
	curvenet:addCurve(BezCurve2d.heapAlloc(Vec2d.stackAlloc(0.0, mm(300.0)),
										  Vec2d.stackAlloc(0.0, mm(300.0))),
					  9)
	curvenet:addConnectionToGround(1, 0, 0.57)
	curvenet:addConnectionToGround(1, 1, 0.82)

	-- 2: A small arch sitting on top of the first two
	curvenet:addCurve(BezCurve2d.heapAlloc(Vec2d.stackAlloc(0.0, mm(300.0)),
										  Vec2d.stackAlloc(0.0, mm(300.0))),
					  11)
	curvenet:addConnectionToCurve(2, 0, 0, 0.5)
	curvenet:addConnectionToCurve(2, 1, 1, 0.5)


	curvenet:fixEndpoints()
end
initGlobalCurveNet()


------------------------------------------------------------------------------------------

return probcomp(function()
	local s3d = s3dLib()
	util.importAll(s3d)

	local gravityConst = `9.8
	local upVector = global(Vec3d)
	upVector:getpointer():__construct(0.0, 0.0, 1.0)

	local frelTol = 0.01
	local trelTol = 0.01


	local lowerBarrier = macro(function(val, lowerBound, shape)
		return quote
			if val < lowerBound then factor([-math.huge])
			else factor(-1.0/(shape*(val-lowerBound))) end
		end
	end)


	-- Treat 2d points as lying in the xz plane centered at y = 0
	local v2tov3 = macro(function(vec)
		return `Vec3.stackAlloc(vec(0), 0.0, vec(1))
	end)
	-- Y component is zero, so this is a quick-n-dirty way to find a perpendicular
	local perp = macro(function(vec)
		return quote
			var pv = vec
			util.swap(pv(0), pv(2))
			pv(0) = -pv(0)
		in
			pv
		end
	end)

	local BezCurve3 = BezCurve(Vec3)

	local terra bc2tobc3(bc2: &BezCurve2d)
		return BezCurve3.stackAlloc(v2tov3(bc2.points[0]), v2tov3(bc2.points[1]),
									v2tov3(bc2.tangents[0]), v2tov3(bc2.tangents[1]))
	end

	local struct BlockCurve
	{
		blocks: Vector(&Body)
	}

	terra BlockCurve:__construct(bc: &BezCurve3, nblocks: uint, minDim: real, maxDim: real, maxHeightChange: real, margin: real, maxAng: real, bodyGen: {&Shape}->{&Body}) : {}
		m.init(self.blocks)
		-- bc:println()
		-- Make blocks of varying width and depth
		var avgDim = 0.5*(minDim + maxDim)
		var yaxis = Vec3.stackAlloc(0.0, 1.0, 0.0)
		for i=0,nblocks do
			var tlo = double(i)/nblocks
			var thi = double(i+1)/nblocks
			var p3lo, t3lo = bc:eval(tlo)
			var p3hi, t3hi = bc:eval(thi)
			var nlo = perp(t3lo); nlo:normalize()
			var nhi = perp(t3hi); nhi:normalize()
			var w = boundedUniform(minDim, maxDim, {initialVal=avgDim})
			var d = boundedUniform(minDim, maxDim, {initialVal=avgDim})
			var widthAxisLo = 0.5*w*nlo
			var widthAxisHi = 0.5*w*nhi
			var depthAxis = 0.5*d*yaxis
			-- Final block shape assembly
			var fbl = p3lo - widthAxisLo - depthAxis
			var fbr = p3lo - widthAxisLo + depthAxis
			var bbl = p3lo + widthAxisLo - depthAxis
			var bbr = p3lo + widthAxisLo + depthAxis
			var ftl = p3hi - widthAxisHi - depthAxis
			var ftr = p3hi - widthAxisHi + depthAxis
			var btl = p3hi + widthAxisHi - depthAxis
			var btr = p3hi + widthAxisHi + depthAxis
			var shape = QuadHex.heapAlloc(fbl, fbr, ftl, ftr, bbl, bbr, btl, btr)
			var body = bodyGen(shape)
			self.blocks:push(body)
		end
		-- Perturb contact points, heights, and shear angles
		var mat : Mat4
		for i=0,nblocks do
			var currBlock = self.blocks(i)
			var currShape = [&QuadHex](currBlock.shape)
			if i > 0 then
				-- contact points
				var prevBlock = self.blocks(i-1)
				var prevShape = [&QuadHex](prevBlock.shape)
				currShape:stackRandomY(prevShape, margin, false, true)
			end
			-- heights
			var axis = currShape:topFace():centroid() - currShape:botFace():centroid()
			var change = boundedUniform(-maxHeightChange, maxHeightChange, {initialVal=0.0})
			mat = Mat4.translate(change*axis)
			currShape:topFace():transform(&mat)
			-- shear angles
			currShape:shearX(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			currShape:topShearX(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			if i < nblocks-1 then	
				-- weld next block back on to maintain connectivity
				var nextBlock = self.blocks(i+1)
				var nextShape = [&QuadHex](nextBlock.shape)
				nextShape:botFace():weld(currShape:topFace(), currShape:topFace():centroid(), false)
			end
			-- Try to prevent this block from inverting (having negative volume)
			var slendev = mm(0.5)
			lowerBarrier(currShape:volume(), 0.0, 1.0/(slendev*slendev*slendev))
		end
	end
	BlockCurve.methods.__construct = pmethod(BlockCurve.methods.__construct)

	terra BlockCurve:__destruct()
		m.destruct(self.blocks)
	end

	BlockCurve.methods.botBlock = macro(function(self)
		return `self.blocks(0)
	end)
	BlockCurve.methods.botFace = macro(function(self)
		return `[&QuadHex](self.blocks(0).shape):botFace()
	end)

	BlockCurve.methods.topBlock = macro(function(self)
		return `self.blocks(self.blocks.size-1)
	end)
	BlockCurve.methods.topFace = macro(function(self)
		return `[&QuadHex](self.blocks(self.blocks.size-1).shape):topFace()
	end)

	terra BlockCurve:addToScene(scene: &Scene)
		for i=0,self.blocks.size do
			scene.bodies:push(self.blocks(i))
		end
		-- Block-to-block contacts
		for i=0,self.blocks.size-1 do
			var currBlock = self.blocks(i)
			var nextBlock = self.blocks(i+1)
			var currShape = [&QuadHex](currBlock.shape)
			var nextShape = [&QuadHex](nextBlock.shape)
			scene.connections:push(RectRectContact.heapAlloc(currBlock, nextBlock, currShape:topFace(), nextShape:botFace(), false))
		end
	end

	m.addConstructors(BlockCurve)



	local struct BlockCurveConnection
	{
		from: &Body,				
		whereFrom: &Face(4),		
		to: &Body,				
		whereTo: &Face(4),		
		whereToPos: Vec3
	}



	local struct BlockCurveNetwork
	{
		ground: &Body,
		blockCurves: Vector(&BlockCurve),
		connections: Vector(BlockCurveConnection)
	}

	terra BlockCurveNetwork:__construct(cnet: &CurveNetwork, groundDepth: real, groundThickness: real,
										minDim: real, maxDim: real, maxHeightChange: real, margin: real, maxAng: real, bodyGen: {&Shape}->{&Body})
		-- Init ground
		var groundShape = Box.heapAlloc(); groundShape:makeBox(v2tov3(0.5*(cnet.ground.points[0] + cnet.ground.points[1])),
															   (cnet.ground.points[1] - cnet.ground.points[0]):norm(),
															   groundDepth, groundThickness)
		self.ground = bodyGen(groundShape)
		self.ground.active = false

		-- Init block curves and connections
		self.blockCurves = [Vector(&BlockCurve)].stackAlloc(cnet.curves.size)
		self.connections = [Vector(BlockCurveConnection)].stackAlloc()
		var connsPerCurve = [Vector(BlockCurveConnection)].stackAlloc()
		for i=0,cnet.curves.size do
			var bc3 = bc2tobc3(cnet.curves(i))

			-- First, fill out the 'to' part of any connections this curve is involved with. We need to do this before
			--    generating the curve itself so that we can update the curve endpoints/tangents to reflect the random variation in
			--    the connecting blocks.
			var nconns = cnet.curve2conn(i).size 
			connsPerCurve:resize(nconns)
			for j=0,nconns do
				var conn = cnet.connections:getPointer(cnet.curve2conn(i)(j))
				var blockConn = connsPerCurve:getPointer(j)
				var connPointPoint2, connPointTangent2 = conn.to:eval(conn.whereTo)
				var connPointPoint3 = v2tov3(connPointPoint2)
				-- If we're connecting to ground, things are pretty straightforward
				if conn.toIndex == -1 then
					blockConn.to = self.ground
					blockConn.whereTo = groundShape:topFace()
					blockConn.whereToPos = groundShape:topFace():projectToPlane(connPointPoint3)
				-- Otherwise, it's a little more involved
				else
					var toBlockCurve = self.blockCurves(conn.toIndex)
					-- Check if we're connecting to an endpoint, b/c that's easy
					if conn.connectToEnd then
						-- Which face we connect to depends on which endpoint
						if conn.toEndpoint == 0 then
							blockConn.to = toBlockCurve:botBlock()
							blockConn.whereTo = toBlockCurve:botFace()
						else
							blockConn.to = toBlockCurve:topBlock()
							blockConn.whereTo = toBlockCurve:topFace()
						end
						-- We just connect in the middle of the face, for endpoint connections
						blockConn.whereToPos = blockConn.whereTo:centroid()
					else
						-- We first need to figure out which block in the 'to' curve we're connecting to.
						var n = cnet.nblocks(conn.toIndex)
						var whichBlock = int(conn.whereTo*n)
						blockConn.to = toBlockCurve.blocks(whichBlock)
						-- We're either connecting to this block's front or back face.
						--    To determine which, we check which way our endpoint tangent is facing
						var normal = conn.to:normal(conn.whereTo)
						var endTang = conn.from.tangents[conn.whereFrom]; endTang:normalize()
						if normal:dot(endTang) > 0.0 then
							blockConn.whereTo = [&QuadHex](blockConn.to.shape):backFace()
						else
							blockConn.whereTo = [&QuadHex](blockConn.to.shape):frontFace()
						end
						-- Take the initial configuration 2d connection point, project it onto the line
						--    between the endpoints of this curve segment, then figure out how far along
						--    that line it is. Use that value to interpolate along the actual 3D connecting
						--    face.
						var tlo = double(whichBlock)/n
						var thi = double(whichBlock+1)/n
						var loPoint2, loTangent2 = conn.to:eval(tlo)
						var hiPoint2, hiTangent2 = conn.to:eval(thi)
						var projPoint2 = connPointPoint2:projectToLineSeg(loPoint2, hiPoint2)
						var linearT = projPoint2:inverseLerp(loPoint2, hiPoint2)
						blockConn.whereToPos = blockConn.whereTo:interp(0.5, linearT)
					end
				end
				-- Update the 'from' curve endpoint/tangent accordingly.
				bc3.points[conn.whereFrom] = blockConn.whereToPos
				bc3.tangents[conn.whereFrom] = blockConn.whereTo:normal() * bc3.tangents[conn.whereFrom]:norm()
			end

			-- Create the blocks using the updated curve
			var fromBlockCurve = BlockCurve.heapAlloc(&bc3, cnet.nblocks(i), minDim, maxDim, maxHeightChange, margin, maxAng, bodyGen)
			self.blockCurves(i) = fromBlockCurve

			-- Finally, fill out the 'from' part of any connections this curve is involved with.
			for j=0,nconns do
				var conn = cnet.connections:getPointer(cnet.curve2conn(i)(j))
				var blockConn = connsPerCurve:getPointer(j)
				-- Figure out if we're connecting the top or bottom face of this block curve based on the
				--    curve endpoint
				if conn.whereFrom == 0 then
					-- bottom
					blockConn.from = fromBlockCurve:botBlock()
					blockConn.whereFrom = fromBlockCurve:botFace()
				else
					-- top
					blockConn.from = fromBlockCurve:topBlock()
					blockConn.whereFrom = fromBlockCurve:topFace()
				end
				-- Weld the connecting face into place
				blockConn.whereFrom:weld(blockConn.whereTo, blockConn.whereToPos, false)
				-- Add to self.connections
				self.connections:push(@blockConn)
			end
		end
		m.destruct(connsPerCurve)
	end
	BlockCurveNetwork.methods.__construct = pmethod(BlockCurveNetwork.methods.__construct)

	terra BlockCurveNetwork:__destruct()
		self.blockCurves:clearAndDelete()
		m.destruct(self.blockCurves)
		m.destruct(self.connections)
	end

	terra BlockCurveNetwork:addToScene(scene: &Scene)
		-- Add ground
		scene.bodies:push(self.ground)
		-- Add all the block curves
		for i=0,self.blockCurves.size do
			self.blockCurves(i):addToScene(scene)
		end
		-- Add all all inter-curve connections
		for i=0,self.connections.size do
			var conn = self.connections:getPointer(i)
			scene.connections:push(RectRectContact.heapAlloc(conn.from, conn.to, conn.whereFrom, conn.whereTo, false))
		end
	end

	m.addConstructors(BlockCurveNetwork)



	-- Computes a factor scoring how symmetric bc1 is with bc2
	local reflectYZ = macro(function(point, xcenter)
		return `Vec3.stackAlloc(-(point(0)-xcenter)+xcenter, point(1), point(2))
	end)
	local archBilateralSymmetry = factorfn(terra(bc1: &BlockCurve, bc2: &BlockCurve, xcenter: real, softness: real)
		util.assert(bc1.blocks.size == bc2.blocks.size, "archBilateralSymmetry - two arches must have same number of blocks\n")
		var n = bc1.blocks.size
		var fac = real(0.0)
		for i=0,bc1.blocks.size do
			var bc1shape = [&QuadHex](bc1.blocks(i).shape)
			var bc2shape = [&QuadHex](bc2.blocks(n-i-1).shape)

			var err = real(0.0)
			err = err + (reflectYZ(bc1shape:frontBotLeftVert(), xcenter) - bc2shape:frontTopLeftVert()):norm()
			err = err + (reflectYZ(bc1shape:frontBotRightVert(), xcenter) - bc2shape:frontTopRightVert()):norm()
			err = err + (reflectYZ(bc1shape:frontTopRightVert(), xcenter) - bc2shape:frontBotRightVert()):norm()
			err = err + (reflectYZ(bc1shape:frontTopLeftVert(), xcenter) - bc2shape:frontBotLeftVert()):norm()
			err = err + (reflectYZ(bc1shape:backBotLeftVert(), xcenter) - bc2shape:backTopLeftVert()):norm()
			err = err + (reflectYZ(bc1shape:backBotRightVert(), xcenter) - bc2shape:backTopRightVert()):norm()
			err = err + (reflectYZ(bc1shape:backTopRightVert(), xcenter) - bc2shape:backBotRightVert()):norm()
			err = err + (reflectYZ(bc1shape:backTopLeftVert(), xcenter) - bc2shape:backBotLeftVert()):norm()

			fac = fac + softeq(err, 0.0, 8.0*softness)
		end
		return fac
	end)



	-- Parameters
	local groundDepth = `mm(1000.0)
	local groundThickness = `mm(10.0)
	local minDim = `mm(20.0)
	local maxDim = `mm(60.0)
	local maxHeightChange = `0.3
	local margin = `mm(10.0)
	local maxAng = `radians(30.0)





	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		-- var camdist = mm(350.0)
		-- camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
		-- camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(120.0))
		var camdist = mm(1000.0)
		camera.eye = Vec3d.stackAlloc(mm(0.0), -camdist, mm(250.0))
		camera.target = Vec3d.stackAlloc(mm(0.0), 0.0, mm(250.0))
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		light.dir = Vec3d.stackAlloc(1.0, -0.7, 1.0)
		renderScene:addLight(light)

		-- Set up stuff in the scene --
		var bcn = BlockCurveNetwork.stackAlloc(&curvenet, groundDepth, groundThickness,
											   minDim, maxDim, maxHeightChange, margin, maxAng, Body.oak)
		bcn:addToScene(&renderScene.scene)

		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)

		-- Symmetry
		var xmid = 0.0
		for i=0,curvenet.curves.size do
			xmid = xmid + curvenet.curves(i).points[0](0)
			xmid = xmid + curvenet.curves(i).points[1](0)
		end
		xmid = xmid / double(2*curvenet.curves.size)
		var symmSoftness = mm(2.0)
		archBilateralSymmetry(bcn.blockCurves(0), bcn.blockCurves(1), xmid, symmSoftness)
		archBilateralSymmetry(bcn.blockCurves(2), bcn.blockCurves(2), xmid, symmSoftness)

		m.destruct(bcn)

		return renderScene
	end
end)








