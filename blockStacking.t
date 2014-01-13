
terralib.require("prob")

local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local rand =terralib.require("prob.random")
local gl = terralib.require("gl")
local image = terralib.require("image")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local Vec2d = Vec(double, 2)
local Color3d = Vec(double, 3)
local RGBImage = image.Image(uint8, 3)


----------------------------------

local Block = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct BlockT
	{
		centerOfMass: Vec2,
		length: real,
		height: real,
		halfLength: real,
		halfHeight: real
	}
	terra BlockT:__construct(com: Vec2, length: real, height: real) : {}
		self.centerOfMass = com
		self.length = length
		self.height = height
		self.halfLength = 0.5*length
		self.halfHeight = 0.5*height
	end
	terra BlockT:__construct() : {}
		self:__construct(Vec2.stackAlloc(0.0, 0.0), 0.0, 0.0)
	end
	terra BlockT:top() return self.centerOfMass(1) + self.halfHeight end
	terra BlockT:bottom() return self.centerOfMass(1) - self.halfHeight end
	terra BlockT:left() return self.centerOfMass(0) - self.halfLength end
	terra BlockT:right() return self.centerOfMass(0) + self.halfLength end
	terra BlockT:mass(density: real) return self.length * self.height * density end
	terra BlockT:weight(density: real, gravMag: real)
		return self:mass(density) * gravMag
	end
	m.addConstructors(BlockT)
	return BlockT
end)
local BlockD = Block(double)

local Structure = templatize(function(real)
	local BlockT = Block(real)
	local struct StructureT
	{
		width: real,
		height: real,
		blocks: Vector(BlockT)
	}
	terra StructureT:__construct(w: real, h: real) : {}
		self.width = w
		self.height = h
		m.init(self.blocks)
	end
	terra StructureT:__construct() : {}
		self:__construct(0.0, 0.0)
	end
	terra StructureT:__destruct()
		m.destruct(self.blocks)
	end
	terra StructureT:__copy(other: &StructureT)
		self.width = other.width
		self.height = other.height
		self.blocks = m.copy(other.blocks)
	end
	m.addConstructors(StructureT)
	return StructureT
end)
local StructureD = Structure(double)

----------------------------------


local function stackingModel()
	local roomWidth = `100.0
	local roomHeight = `100.0
	local groundHeight = `2.0
	local bottomBlockCenterX = `roomWidth/2.0
	local blockMinLength = `10.0
	local blockMaxLength = `40.0
	local blockMinHeight = `5.0
	local blockMaxHeight = `10.0
	local blockDensity = `0.1
	local numBlocks = 6
	local gravityConstant = `9.8

	local Vec2 = Vec(real, 2)
	local BlockT = Block(real)
	local StructureT = Structure(real)

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)

	local boundedUniform = macro(function(lo, hi)
		return `uniform(lo, hi, {structural=false, lowerBound=lo, upperBound=hi})
	end)

	-- A force is defined by its vector direction/magnitude and the point on
	--    which it acts
	local struct Force { vec: Vec2, point: Vec2 }

	local genForce = pfn(terra(block: &BlockT, point: Vec2, dir: Vec2)
		-- Variance for the force magnitude is proportional the force of gravity
		--    on this block
		-- TODO: Does this heuristic work? Or does it make things weird?
		-- var variance = block:weight(blockDensity, gravityConstant)
		var variance = 100.0
		var forceMag = gaussian(0.0, variance, {structural=false, lowerBound=0.0})
		return Force { forceMag * dir, point }
	end)

	local terra torque(force: Force, centerOfRotation: Vec2)
		var d = force.point - centerOfRotation
		return force.vec(0)*d(1) - force.vec(1)*d(0)
	end

	local stabilityConstraint = pfn(terra(structure: &StructureT)
		var up = Vec2.stackAlloc(0.0, 1.0)
		var down = -up
		var forces = [Vector(Force)].stackAlloc()
		var centersOfRotation = [Vector(Vec2)].stackAlloc()

		-- Enforce stability for all blocks except the bottom block
		--    (which is stable due to resting on the ground), and the
		--    top block (which is stable by construction)
		for i=1,structure.blocks.size-1 do
			forces:clear()
			centersOfRotation:clear()
			var block = structure.blocks:getPointer(i)

			-- Generate upward forces due to the block below us
			var belowBlock = structure.blocks:getPointer(i-1)
			var upForceY = block:bottom()
			var upForceX1 = ad.math.fmax(block:left(), belowBlock:left())
			var upForceX2 = ad.math.fmin(block:right(), belowBlock:right())
			var upPos1 = Vec2.stackAlloc(upForceX1, upForceY)
			var upPos2 = Vec2.stackAlloc(upForceX2, upForceY)
			forces:push(genForce(block, upPos1, up))
			forces:push(genForce(block, upPos2, up))

			-- Generate downward forces due to the block above us
			var aboveBlock = structure.blocks:getPointer(i+1)
			var downForceY = block:top()
			var downForceX1 = ad.math.fmax(block:left(), aboveBlock:left())
			var downForceX2 = ad.math.fmin(block:right(), aboveBlock:right())
			var downPos1 = Vec2.stackAlloc(downForceX1, downForceY)
			var downPos2 = Vec2.stackAlloc(downForceX2, downForceY)
			forces:push(genForce(block, downPos1, down))
			forces:push(genForce(block, downPos2, down))

			-- C.printf("----------------\n")
			-- C.printf("%g, %g, %g, %g\n",
			-- 	ad.val(forces(0).vec(1)), ad.val(forces(1).vec(1)), ad.val(forces(2).vec(1)), ad.val(forces(3).vec(1)))
			
			-- Account for gravity
			forces:push(Force { block:weight(blockDensity, gravityConstant)*down,
								block.centerOfMass })

			-- Determine centers of rotation about which to calculate torque:
			--    We have four unknown forces.
			--    This requires force balance eqn. + 3 torque balance eqns.
			--    In additional to c.o.m., we'll use the two bottom contacts
			centersOfRotation:push(block.centerOfMass)
			centersOfRotation:push(upPos1)
			centersOfRotation:push(upPos2)

			-- Enforce force balance
			var totalForce = real(0.0)
			for i=0,forces.size do
				-- Forces are all vertical, so we just sum up the y component
				totalForce = totalForce + forces(i).vec(1)
			end
			-- C.printf("totalForce: %g\n", ad.val(totalForce))
			factor(softEq(totalForce, 0.0, 1.0))

			-- Enforce torque balance
			for c=0,centersOfRotation.size do
				var totalTorque = real(0.0)
				for i=0,forces.size do
					var t = torque(forces(i), centersOfRotation(c))
					totalTorque = totalTorque + t
				end
				factor(softEq(totalTorque, 0.0, 1.0))
			end
		end

		m.destruct(forces)
		m.destruct(centersOfRotation)
	end)

	local rescale = macro(function(x, lo, hi)
		return `lo + (hi - lo) * x
	end)

	return terra()
		var structure = StructureT.stackAlloc(roomWidth, roomHeight)

		-- Generate the bottom block
		var botBlockLength = boundedUniform(blockMinLength, blockMaxLength)
		var botBlockHeight = boundedUniform(blockMinHeight, blockMaxHeight)
		var botBlockCenter = Vec2.stackAlloc(bottomBlockCenterX, groundHeight + botBlockHeight/2.0)
		structure.blocks:push(BlockT.stackAlloc(botBlockCenter, botBlockLength, botBlockHeight))
		
		-- Generate the stack of blocks above it
		for i=1,numBlocks do
			var length = boundedUniform(blockMinLength, blockMaxLength)
			var height = boundedUniform(blockMinHeight, blockMaxHeight)
			-- This block must be 'locally stable' (the center of mass lies atop the
			--    the block beneath this one)
			var supportBlock = structure.blocks:getPointer(i-1)
			var y = supportBlock:top() + 0.5*height
			var xlo = supportBlock:left()
			var xhi = supportBlock:right()
			var x = rescale(boundedUniform(0.0, 1.0), xlo, xhi)
			var com = Vec2.stackAlloc(x, y)
			structure.blocks:push(BlockT.stackAlloc(com, length, height))
		end

		-- Enforce stability
		stabilityConstraint(&structure)

		return structure
	end
end


----------------------------------

local terra drawBlock(block: &BlockD, color: Color3d)
	var top = block:top()
	var bot = block:bottom()
	var left = block:left()
	var right = block:right()
	-- Draw polygon
	gl.glColor3d(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_QUADS())
	gl.glVertex2d(left, bot)
	gl.glVertex2d(left, top)
	gl.glVertex2d(right, top)
	gl.glVertex2d(right, bot)
	gl.glEnd()
	-- Outline the edges in black
	gl.glLineWidth(2.0)
	gl.glColor3d(0.0, 0.0, 0.0)
	gl.glBegin(gl.mGL_LINE_LOOP())
	gl.glVertex2d(left, bot)
	gl.glVertex2d(left, top)
	gl.glVertex2d(right, top)
	gl.glVertex2d(right, bot)
	gl.glEnd()
end

local terra drawStructure(structure: &StructureD, blockColor: Color3d)
	gl.glMatrixMode(gl.mGL_PROJECTION())
	gl.glLoadIdentity()
	gl.gluOrtho2D(0, structure.width, 0, structure.height)
	gl.glMatrixMode(gl.mGL_MODELVIEW())
	gl.glLoadIdentity()
	for i=0,structure.blocks.size do
		drawBlock(structure.blocks:getPointer(i), blockColor)
	end
end

local function renderSamples(samples, moviename, imageWidth, imageHeight)
	local moviefilename = string.format("renders/%s.mp4", moviename)
	local movieframebasename = string.format("renders/%s", moviename) .. "_%06d.png"
	local movieframewildcard = string.format("renders/%s", moviename) .. "_*.png"
	io.write("Rendering video...")
	io.flush()
	local numsamps = samples.size
	local frameSkip = math.ceil(numsamps / 1000.0)
	local terra renderFrames()
		var argc = 0
		gl.glutInit(&argc, nil)
		gl.glutInitWindowSize(imageWidth, imageHeight)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageHeight)
		var im = RGBImage.stackAlloc(imageWidth, imageHeight)
		var framename: int8[1024]
		var framenumber = 0
		var blockColor = Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			var structure = &samples(i).value
			drawStructure(structure, blockColor)
			gl.glFlush()
			gl.glReadPixels(0, 0, imageWidth, imageHeight,
				gl.mGL_BGR(), gl.mGL_UNSIGNED_BYTE(), im.data)
			[RGBImage.save()](&im, image.Format.PNG, framename)
		end
	end
	renderFrames()
	util.wait(string.format("ffmpeg -threads 0 -y -r 30 -i %s -c:v libx264 -r 30 -pix_fmt yuv420p %s 2>&1", movieframebasename, moviefilename))
	util.wait(string.format("rm -f %s", movieframewildcard))
	print("done.")
end

----------------------------------
local numsamps = 2000
-- local numsamps = 1000000
local verbose = true
local temp = 1.0
local imageWidth = 500
local imageHeight = 500
local kernel = HMC({numSteps=1000, verbosity=0})
-- local kernel = RandomWalk()
local scheduleFn = macro(function(iter, currTrace)
	return quote
		currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
	return [mcmc(stackingModel, kernel, {numsamps=numsamps, verbose=verbose})]
	-- return [forwardSample(stackingModel, numsamps)]
end
local samples = m.gc(doInference())
moviename = arg[1] or "movie"
renderSamples(samples, moviename, imageWidth, imageHeight)




