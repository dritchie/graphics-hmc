terralib.require("prob")


local s3dLib = terralib.require("s3dLib")
local m = terralib.require("mem")
local util = terralib.require("util")
local ad = terralib.require("ad")
local AutoPtr = terralib.require("autopointer")
local Vector = terralib.require("Vector")
local Vec = terralib.require("linalg").Vec
local Vec3d = Vec(double, 3)
local glutils = terralib.require("glutils")
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


------------------------------------------------------------------------------------------



local Vec2d = Vec(double, 2)

local struct BezCurve
{
	points: Vec2d[2],
	tangents: Vec2d[2]
}

terra BezCurve:__construct(sp: Vec2d, ep: Vec2d, st: Vec2d, et: Vec2d) : {}
	self.points[0] = sp
	self.points[1] = ep
	self.tangents[0] = st
	self.tangents[1] = et
end

terra BezCurve:__construct(ep: Vec2d, st: Vec2d, et: Vec2d) : {}
	self:__construct(Vec2d.stackAlloc(0.0, 0.0), ep, st, et)
end

terra BezCurve:__construct(st: Vec2d, et: Vec2d) : {}
	self:__construct(Vec2d.stackAlloc(0.0, 0.0), st, et)
end

terra BezCurve:__construct() : {}
	self:__construct(Vec2d.stackAlloc(0.0, 0.0), Vec2d.stackAlloc(0.0, 0.0))
end

terra BezCurve:eval(t: double)
	var t2 = t*t
	var t3 = t*t2
	var oneMinusT = 1.0 - t
	var oneMinusT2 = oneMinusT*oneMinusT
	var oneMinusT3 = oneMinusT*oneMinusT2
	var p0 = self.points[0]
	var p3 = self.points[1]
	var p1 = p0 + self.tangents[0]
	var p2 = p3 + self.tangents[1]
	var point = oneMinusT3*p0 + 3.0*oneMinusT2*t*p1 + 3.0*oneMinusT*t2*p2 + t3*p3
	var tangent = 3.0*oneMinusT2*(p1 - p0) + 6.0*oneMinusT*t*(p2 - p1) + 3.0*t2*(p3 - p2)
	return point, tangent
end

terra BezCurve:normal(t: double)
	var point, tang = self:eval(t)
	tang:normalize()
	util.swap(tang(0), tang(1))
	tang(0) = -tang(0)
	return tang
end

m.addConstructors(BezCurve)


local struct CurveConnection
{
	from: &BezCurve,			-- Which curve?
	fromIndex: int,
	whereFrom: uint,			-- Which endpoint on the curve?
	to: &BezCurve,				-- Which curve?
	toIndex: int,				-- (Can be -1 if ground)
	whereTo: double				-- How far along the curve?
}



local struct CurveNetwork
{
	ground: &BezCurve,
	curves: Vector(&BezCurve),
	connections: Vector(CurveConnection),
	curve2conn: Vector(Vector(uint))
}

terra CurveNetwork:__construct(groundHeight: double, groundWidth: double)
	-- Ground is just a straight line bezier curve
	self.ground = BezCurve.heapAlloc(Vec2d.stackAlloc(-0.5*groundWidth, groundHeight),
									 Vec2d.stackAlloc(0.5*groundWidth, groundHeight),
									 Vec2d.stackAlloc(1.0, 0.0),
									 Vec2d.stackAlloc(-1.0, 0.0))
	m.init(self.curves)
	m.init(self.connections)
	m.init(self.curve2conn)
end

terra CurveNetwork:__destruct()
	m.delete(self.ground)
	self.curves:clearAndDelete()
	m.destruct(self.curves)
	m.destruct(self.connections)
	m.destruct(self.curve2conn)
end

terra CurveNetwork:addCurve(bc: BezCurve)
	self.curves:push(bc)
	self.curve2conn:resize(self.curve2conn.size)
end

terra CurveNetwork:addConnection(cc: CurveConnection)
	self.connections:push(cc)
	self.curve2conn(cc.to):push(self.connections.size-1)
	self.curve2conn(cc.from):push(self.connections.size-1)
end

terra CurveNetwork:fixEndpoints()
	for i=0,self.curves.size do
		var curve = self.curves(i)
		var conns = self.curve2conn:getPointer(i)
		for j=0,conns.size do
			var connIndex = conns(j)
			var conn = self.connections(connIndex)
			if curve == conn.from then
				var point, tangent = conn.to:eval(conn.whereTo)
				curve.points[conn.whereFrom] = point
			end
		end
	end
end

m.addConstructors(CurveNetwork
	)

------------------------------------------------------------------------------------------

-- Global instance of a curve network that all inference will use
local curvenet = global(CurveNetwork)
local terra initGlobalCurveNet()
	curvenet = CurveNetwork.stackAlloc(0.0, mm(1000.0))
	-- TODO: add other curves 'n stuff
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


	local struct BlockCurve
	{
		blocks: Vector(&Body)
	}

	terra BlockCurve:__construct(bc: &BezCurve, nblocks: uint, minDim: real, maxDim: real, maxHeightChange: real, margin: real, maxAng: real, bodyGen: {&Shape}->{&Body}) : {}
		m.init(self.blocks)
		-- Make blocks of varying width and depth
		var yaxis = Vec3.stackAlloc(0.0, 1.0, 0.0)
		for i=0,nblocks do
			var tlo = double(i)/nblocks
			var thi = double(i+1)/nblocks
			var p2lo, t2lo = bc:eval(tlo)
			var p3lo, t3lo = v2tov3(p2lo), v2tov3(t2lo)
			var p2hi, t2hi = bc:eval(thi)
			var p3hi, t3hi = v2tov3(p2hi), v2tov3(t2hi)
			var nlo = perp(t3lo); nlo:normalize()
			var nhi = perp(t3hi); nhi:normalize()
			var w = boundedUniform(minDim, maxDim)
			var d = boundedUniform(minDim, maxDim)
			var widthAxisLo = w*nlo
			var widthAxisHi = w*nhi
			var depthAxis = d*yaxis
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
				currShape:stackRandomX(prevShape, margin, false, true)
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
		whereToX: real,			
		whereToY: real
	}



	local struct BlockCurveNetwork
	{
		ground: &Body,
		blockCurves: Vector(&BlockCurve),
		connections: Vector(BlockCurveConnection)
	}

	terra BlockCurveNetwork:__construct(cnet: &CurveNetwork, nblocks: &Vector(uint), groundDepth: real, groundThickness: real,
										minDim: real, maxDim: real, maxHeightChange: real, margin: real, maxAng: real, bodyGen: {&Shape}->{&Body})
		-- Init ground
		var groundShape = Box.heapAlloc(); groundShape:makeBox(v2tov3(0.5*(cnet.ground.points[0] + cnet.ground.points[1])),
															   (cnet.ground.points[1] - cnet.ground.points[0]):norm(),
															   groundDepth, groundThickness)
		self.ground = bodyGen(groundShape)
		self.ground.active = false

		-- Init block curves
		self.blockCurves = [Vector(&BlockCurve)].stackAlloc(cnet.curves.size)
		cnet:fixEndpoints()
		for i=0,cnet.curves.size do
			self.blockCurves(i) = BlockCurve.heapAlloc(cnet.curves(i), nblocks(i), minDim, maxDim, maxHeightChange, margin, maxAng, bodyGen)
		end

		-- Init connections
		-- Weld block curve faces correctly.
		self.connections = [Vector(BlockCurveConnection)].stackAlloc(cnet.connections.size)
		for i=0,cnet.connections.size do
			var conn = cnet.connections:getPointer(i)
			var blockConn = self.connections:getPointer(i)
			var fromBlockCurve = self.blockCurves(conn.fromIndex)
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
			-- If we're connecting to ground, things are pretty straightforward
			if conn.toIndex == -1 then
				blockConn.to = self.ground
				blockConn.whereTo = groundShape:topFace()
				blockConn.whereToX = conn.whereTo
				blockConn.whereToY = 0.5
			-- Otherwise, it's a little more involved
			else
				-- We first need to figure out which block in the 'to' curve we're connecting to.
				var toBlockCurve = self.blockCurves(conn.toIndex)
				var n = nblocks(conn.toIndex)
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
				-- Now just figure out where on the block we're connecting to
				var tlo = double(whichBlock)/n
				var thi = double(whichBlock+1)/n
				blockConn.whereToX = 0.5
				blockConn.whereToY = (conn.whereTo - tlo) / (thi - tlo)

			end
			-- Finally, weld the connecting face into place
			blockConn.whereFrom:weld(blockConn.whereTo, blockConn.whereToX, blockConn.whereToY, false)
		end
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





	-- Parameters
	local groundDepth = `mm(1000.0)
	local groundThickness = `mm(10.0)
	local minDim = `mm(39.0)
	local maxDim = `mm(41.0)
	local maxHeightChange = `0.01
	local margin = `mm(18.0)
	local maxAng = `radians(1.0)




	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		-- var camdist = mm(350.0)
		-- camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
		-- camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(120.0))
		var camdist = mm(750.0)
		camera.eye = Vec3d.stackAlloc(0.5*camdist, -camdist, 0.5*camdist)
		camera.target = Vec3d.stackAlloc(mm(100.0), 0.0, mm(120.0))
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		renderScene:addLight(light)

		-- Set up stuff in the scene --
		var bcn = BlockCurveNetwork.stackAlloc(&curvenet, nil, groundDepth, groundThickness,
											   minDim, maxDim, maxHeightChange, margin, maxAng, Body.oak)
		bcn:addToScene(&renderScene.scene)
		m.destruct(bcn)

		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)


		return renderScene
	end
end)








