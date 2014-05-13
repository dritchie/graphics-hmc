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


return probcomp(function()
	local s3d = s3dLib()
	util.importAll(s3d)

	local gravityConst = `9.8
	local upVector = global(Vec3d)
	upVector:getpointer():__construct(0.0, 0.0, 1.0)

	local frelTol = 0.01
	local trelTol = 0.01

	local mm = macro(function(x)
		return `0.001*x
	end)
	local radians = macro(function(deg)
		return `deg*[math.pi]/180.0
	end)

	-- Parameters
	local minDim = `mm(20.0)
	local maxDim = `mm(80.0)
	local maxHeightVarPercent = `0.3
	local maxAng = `radians(30.0)
	local margin = `mm(3.0)

	local Vec2 = Vec(real, 2)

	local struct Stack
	{
		blocks: Vector(&Body),
		botBody: &Body,
		topBody: &Body 
	}

	terra Stack:__construct(botBody: &Body, botPoint: Vec3, topBody: &Body, topPoint: Vec3, numBlocks: uint, bodyGen: {&Shape}->{&Body}) : {}
		m.init(self.blocks)
		self.botBody = botBody
		self.topBody = topBody
		var botShape = [&QuadHex](botBody.shape)
		var topShape = [&QuadHex](topBody.shape)
		var shapes = [Vector(&QuadHex)].stackAlloc()

		-- Generate a bunch of blocks of random size (but uniform height) from bot to top
		var totalHeight = topPoint(2) - botPoint(2)
		var heightPerBlock = totalHeight / double(numBlocks)
		var zerovec = Vec3.stackAlloc(0.0, 0.0, 0.0)
		for i=0,numBlocks do
			var w = boundedUniform(minDim, maxDim)
			var d = boundedUniform(minDim, maxDim)
			var h = heightPerBlock
			var blockShape = QuadHex.heapAlloc(); blockShape:makeBox(zerovec, w, d, h)
			shapes:push(blockShape)
		end

		-- Stack bottom block
		shapes(0):stack(botShape, botPoint)
		-- shapes(0):botFace():weld(botShape:topFace(), botPoint, false)

		-- stackRandom the blocks on top of each other
		var coords = [Vector(Vec2)].stackAlloc(numBlocks)
		for i=1,numBlocks do
			var prevShape = shapes(i-1)
			var currShape = shapes(i)
			-- currShape:stackRandom(prevShape, margin, false, true)
			-- currShape:stack(prevShape, 0.5, 0.5, true)
			currShape:stackRandomX(prevShape, margin, false, true)
			coords(i) = prevShape:topFace():coords(currShape:botFace():centroid(), false)
		end

		-- Weld top block
		shapes:back():topFace():weld(topShape:botFace(), topPoint, false)

		-- Apply random height variations and shears, re-weld as we go
		var mat : Mat4
		for i=0,numBlocks-1 do
			var shape = shapes(i)
			var heightaxis = shape:frontLeftEdge()
			mat = Mat4.translate(boundedUniform(-maxHeightVarPercent, maxHeightVarPercent) * heightaxis)
			shape:topFace():transform(&mat)
			-- if i % 2 == 0 then
				shape:topShearX(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			-- else
			-- 	shape:topShearY(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			-- end
			shape:shearX(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			-- shape:shearY(boundedUniform(-maxAng, maxAng, {initialVal=0.0}))
			var nextShape = shapes(i+1)
			var nextCoords = coords(i+1)
			nextShape:botFace():weld(shape:topFace(), nextCoords(0), nextCoords(1), false, false)
		end

		m.destruct(coords)

		-- Convert shapes to bodies
		for i=0,shapes.size do
			-- C.printf("%d\n", i)
			-- shapes(i):assertFacePlanarity()
			self.blocks:push(bodyGen(shapes(i)))
		end
		m.destruct(shapes)
	end

	terra Stack:__construct(botBody: &Body, bx: real, by: real, topBody: &Body, tx: real, ty: real, numBlocks: uint, bodyGen: {&Shape}->{&Body}) : {}
		var botShape = [&QuadHex](botBody.shape)
		var topShape = [&QuadHex](topBody.shape)
		var topPoint = topShape:botFace():interp(tx, ty)
		var botPoint = botShape:topFace():interp(bx, by)
		self:__construct(botBody, botPoint, topBody, topPoint, numBlocks, bodyGen)
	end

	Stack.methods.__construct = pmethod(Stack.methods.__construct)

	terra Stack:__destruct()
		m.destruct(self.blocks)
	end

	terra Stack:transform(mat: &Mat4)
		for i=0,self.blocks.size do
			self.blocks(i).shape:transform(mat)
		end
	end

	terra Stack:addToScene(scene: &Scene)
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
		-- Top and bottom contacts
		var firstBlock = self.blocks(0)
		var lastBlock = self.blocks:back()
		var firstShape = [&QuadHex](firstBlock.shape)
		var lastShape = [&QuadHex](lastBlock.shape)
		var botShape = [&QuadHex](self.botBody.shape)
		var topShape = [&QuadHex](self.topBody.shape)
		scene.connections:push(RectRectContact.heapAlloc(firstBlock, self.botBody, firstShape:botFace(), botShape:topFace(), false))
		scene.connections:push(RectRectContact.heapAlloc(lastBlock, self.topBody, lastShape:topFace(), topShape:botFace(), false))
	end

	m.addConstructors(Stack)



	local struct Arch
	{
		botBody1: &Body,
		botBody2: &Body,
		stack1: Stack,
		stack2: Stack,
		topBody: &Body
	}

	terra Arch:__construct(bodyGen: {&Shape}->{&Body},
						   topCenter: Vec3, topw: real, topd: real, toph: real,
						   bb1: &Body, nblocks1: uint, lox1: real, loy1: real, hix1: real, hiy1: real,
						   bb2: &Body, nblocks2: uint, lox2: real, loy2: real, hix2: real, hiy2: real)
		-- Create the top bar
		var topShape = Box.heapAlloc(); topShape:makeBox(topCenter, topw, topd, toph)
		var rotAngle = boundedUniform(-maxAng, maxAng, {initialVal=0.0})
		var rotmat = Mat4.rotate(Vec3.stackAlloc(0.0, 1.0, 0.0), rotAngle, topShape:centroid())
		topShape:transform(&rotmat)
		self.topBody = bodyGen(topShape)

		-- Stack 1
		self.botBody1 = bb1
		self.stack1 = Stack.stackAlloc(self.botBody1, lox1, loy1, self.topBody, hix1, hiy1, nblocks1, bodyGen)

		-- Stack 2
		self.botBody2 = bb2
		self.stack2 = Stack.stackAlloc(self.botBody2, lox2, loy2, self.topBody, hix2, hiy2, nblocks2, bodyGen)
	end
	Arch.methods.__construct = pmethod(Arch.methods.__construct)

	terra Arch:__destruct()
		m.destruct(self.stack1)
		m.destruct(self.stack2)
	end

	terra Arch:addToScene(scene: &Scene)
		scene.bodies:push(self.topBody)
		self.stack1:addToScene(scene)
		self.stack2:addToScene(scene)
	end

	m.addConstructors(Arch)



	return terra()
		-- Set up scene
		var scene = Scene.stackAlloc(gravityConst, upVector)
		var camera = Camera.stackAlloc()
		-- var camdist = mm(350.0)
		-- camera.eye = Vec3d.stackAlloc(camdist, -camdist, camdist)
		-- camera.target = Vec3d.stackAlloc(0.0, 0.0, mm(120.0))
		var camdist = mm(900.0)
		camera.eye = Vec3d.stackAlloc(0.5*camdist, -camdist, 0.5*camdist)
		camera.target = Vec3d.stackAlloc(mm(130.0), 0.0, mm(160.0))
		camera.up = upVector
		camera.znear = 0.01
		camera.zfar = 10.0
		var renderScene = AutoPtr.wrap(RenderableScene.heapAlloc(scene, camera))
		var light = Light.stackAlloc()
		renderScene:addLight(light)

		-- Set up stuff in the scene --

		-- Ground
		var groundShape = Box.heapAlloc(); groundShape:makeBox(Vec3.stackAlloc(0.0, 0.0, -mm(35.0)), mm(1300.0), mm(1300.0), mm(10.0))
		var groundBody = Body.oak(groundShape); groundBody.active = false
		renderScene.scene.bodies:push(groundBody)

		----- Stacks 'n stuff -----

		var loContactx1 = boundedUniform(0.3, 0.4, {initialVal=0.35})
		var hiContactx1 = boundedUniform(0.75, 0.95, {initialVal=0.9})
		var loContactx2 = boundedUniform(0.6, 0.7, {initialVal=0.65})
		var hiContactx2 = boundedUniform(0.05, 0.25, {initialVal=0.1})
		var arch1 = Arch.stackAlloc(Body.oak,
									Vec3.stackAlloc(0.0, 0.0, mm(200.0)), mm(300.0), mm(100.0), mm(10.0),
									groundBody, 4, loContactx1, 0.5, hiContactx1, 0.5,
									groundBody, 4, loContactx2, 0.5, hiContactx2, 0.5)
		arch1:addToScene(&renderScene.scene)

		loContactx1 = boundedUniform(0.4, 0.6, {initialVal=0.5})
		hiContactx1 = boundedUniform(0.75, 0.95, {initialVal=0.9})
		loContactx2 = boundedUniform(0.83, 0.9, {initialVal=0.85})
		hiContactx2 = boundedUniform(0.05, 0.25, {initialVal=0.1})
		var arch2 = Arch.stackAlloc(Body.oak,
									Vec3.stackAlloc(mm(150.0), 0.0, mm(300.0)), mm(350.0), mm(100.0), mm(10.0),
									arch1.topBody, 2, loContactx1, 0.5, hiContactx1, 0.5,
									groundBody, 6, loContactx2, 0.5, hiContactx2, 0.5)
		arch2:addToScene(&renderScene.scene)

		m.destruct(arch1)
		m.destruct(arch2)


		-- Stablity
		renderScene.scene:encourageStability(frelTol, trelTol)


		return renderScene
	end
end)








