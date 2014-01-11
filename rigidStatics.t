
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

local Bar = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local struct BarT
	{
		bot: Vec2,
		top: Vec2,
		width: real
	}
	terra BarT:centerOfMass()
		return 0.5*(self.bot + self.top)
	end
	return BarT
end)
local BarD = Bar(double)

local Structure = templatize(function(real)
	local BarT = Bar(real)
	local struct StructureT
	{
		width: real,
		height: real,
		bars: Vector(BarT)
	}
	terra StructureT:__construct(w: real, h: real) : {}
		self.width = w
		self.height = h
		m.init(self.bars)
	end
	terra StructureT:__construct() : {}
		self:__construct(0.0, 0.0)
	end
	terra StructureT:__destruct()
		m.destruct(self.bars)
	end
	terra StructureT:__copy(other: &StructureT)
		self.width = other.width
		self.height = other.height
		self.bars = m.copy(other.bars)
	end
	m.addConstructors(StructureT)
	return StructureT
end)
local StructureD = Structure(double)

----------------------------------


local function staticsModel()
	local width = `100.0
	local height = `50.0
	local base = `5.0
	local supportHeight = `15.0
	local barWidth = `5.0
	local horizBarCenterPos = `width/2.0
	local numSupports = 2
	local gravityConstant = `-9.8
	local barDensity = 0.1

	local Vec2 = Vec(real, 2)
	local BarT = Bar(real)
	local StructureT = Structure(real)

	local softEq = macro(function(x, target, softness)
		return `[rand.gaussian_logprob(real)](x, target, softness)
	end)

	local terra torque(force: Vec2, pointOfAction: Vec2, centerOfRotation: Vec2)
		var d = pointOfAction - centerOfRotation
		return force(0)*d(1) - force(1)*d(0)
	end

	-- Currently, everything assumes that there is one horizontal bar, followed
	--    by some number of vertical bars supporting it.
	-- Model assumes frictionless point contacts.

	local stabilityConstraint = pfn(terra(structure: &StructureT)
		var up = Vec2.stackAlloc(0.0, 1.0)
		var horizBar = structure.bars:getPointer(0)
		
		var forces = [Vector(Vec2)].stackAlloc()
		var pointsOfAction = [Vector(Vec2)].stackAlloc()
		var force2point = [Vector(uint)].stackAlloc()

		-- C.printf("----------------\n")
		for i=0,numSupports do
			var vertBar = structure.bars:getPointer(i+1)
			var pointOfAction = vertBar.top + (horizBar.width/2.0)*up
			pointsOfAction:push(pointOfAction)
			-- Two internal forces for each support: the force exerted by the bar on the support,
			--    and the force exerted by the support on the bar
			-- The first force has to act downward, and the second has to act upward

			var downForce = gaussian(0.0, 100.0, {structural=false}) * up
			var upForce = gaussian(0.0, 100.0, {structural=false}) * up
			-- var downForce = gaussian(0.0, 100.0, {structural=false, upperBound=0.0}) * up
			-- var upForce = gaussian(0.0, 100.0, {structural=false, lowerBound=0.0}) * up

			-- C.printf("f: (%g, %g)\n", ad.val(upForce(0)), ad.val(upForce(1)))
			-- C.printf("f: (%g, %g)\n", ad.val(downForce(0)), ad.val(downForce(1)))

			-- forces:push(downForce)
			-- force2point:push(pointsOfAction.size-1)
			forces:push(upForce)
			force2point:push(pointsOfAction.size-1)
		end

		-- The only external force is gravity
		var hbarLength = (horizBar.top - horizBar.bot):norm()
		var gravForce = gravityConstant * hbarLength * up
		forces:push(gravForce)
		-- Gravity acts on the center of mass of the horizontal bar
		pointsOfAction:push(horizBar:centerOfMass())
		force2point:push(pointsOfAction.size-1)

		-- Constraint: The sum of all forces should be zero.
		var fsum = Vec2.stackAlloc(0.0, 0.0)
		for i=0,forces.size do
			fsum = fsum + forces(i)
		end
		factor(softEq(fsum(0), 0.0, 1.0))
		factor(softEq(fsum(1), 0.0, 1.0))
		-- C.printf("fsum:norm(): %g          \n", ad.val(fsum:norm()))

		-- Constraint: For any given point of action, the sum of all force moments about that
		--    point should be zero.
		var totaltsum = real(0.0)
		for i=0,pointsOfAction.size do
			var tsum = real(0.0)
			var c = pointsOfAction(i)
			for j=0,forces.size do
				var f = forces(j)
				var p = pointsOfAction(force2point(j))
				var t = torque(f, p, c)
				-- if not (t == t) then
				-- 	util.fatalError("f: (%f, %f), p: (%f, %f), c: (%f, %f)\n",
				-- 		ad.val(f(0)), ad.val(f(1)), ad.val(p(0)), ad.val(p(1)), ad.val(c(0)), ad.val(c(1)))
				-- end
				tsum = tsum + t
			end
			factor(softEq(tsum, 0.0, 10.0))
			totaltsum = totaltsum + tsum
		end
		-- C.printf("total torque: %g           \n", ad.val(totaltsum))

		m.destruct(forces)
		m.destruct(pointsOfAction)
		m.destruct(force2point)
	end)

	return terra()
		var structure = StructureT.stackAlloc(width, height)

		-- Gen horizontal bar
		-- var horizBarLength = gaussian(60.0, 10.0, {structural=false})
		var horizBarLength = 60.0
		var hbarLeft = horizBarCenterPos - horizBarLength/2.0
		var hbarRight = horizBarCenterPos + horizBarLength/2.0
		var hbarHeight = base + supportHeight + barWidth/2.0
		structure.bars:push(BarT{Vec2.stackAlloc(hbarLeft, hbarHeight), Vec2.stackAlloc(hbarRight, hbarHeight), barWidth})
		
		-- Gen supporting bars
		var supportBot = base
		var supportTop = base + supportHeight
		var lo = hbarLeft+barWidth/2.0
		var hi = hbarRight-barWidth/2.0
		for i=0,numSupports do
			var supportHoriz : real
			supportHoriz = uniform(lo, hi, {structural=false, lowerBound=lo, upperBound=hi})
			-- supportHoriz = 0.5*(lo + hi)
			-- if i == 0 then supportHoriz = lo else supportHoriz = hi end
			structure.bars:push(BarT{Vec2.stackAlloc(supportHoriz, supportBot), Vec2.stackAlloc(supportHoriz, supportTop), barWidth})
		end

		-- Enforce stability
		stabilityConstraint(&structure)

		return structure
	end
end


----------------------------------

local terra drawBar(bar: &BarD, color: Color3d)
	var dir = bar.top - bar.bot
	dir:normalize()
	var w = bar.width / 2.0
	var perp = Vec2d.stackAlloc(dir(1), dir(0))
	var p0 = bar.bot - w*perp
	var p1 = bar.bot + w*perp
	var p2 = bar.top + w*perp
	var p3 = bar.top - w*perp
	gl.glColor3d(color(0), color(1), color(2))
	gl.glBegin(gl.mGL_QUADS())
	gl.glVertex2d(p0(0), p0(1))
	gl.glVertex2d(p1(0), p1(1))
	gl.glVertex2d(p2(0), p2(1))
	gl.glVertex2d(p3(0), p3(1))
	gl.glEnd()
end

local terra drawStructure(structure: &StructureD, barColor: Color3d)
	gl.glMatrixMode(gl.mGL_PROJECTION())
	gl.glLoadIdentity()
	gl.gluOrtho2D(0, structure.width, 0, structure.height)
	gl.glMatrixMode(gl.mGL_MODELVIEW())
	gl.glLoadIdentity()
	for i=0,structure.bars.size do
		drawBar(structure.bars:getPointer(i), barColor)
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
		var barColor = Color3d.stackAlloc(31/255.0, 119/255.0, 180/255.0)
		for i=0,numsamps,frameSkip do
			C.sprintf(framename, movieframebasename, framenumber)
			framenumber = framenumber + 1
			gl.glClearColor(1.0, 1.0, 1.0, 1.0)
			gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
			var structure = &samples(i).value
			drawStructure(structure, barColor)
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
local numsamps = 1000
local verbose = true
local temp = 1.0
local imageWidth = 500
local imageHeight = 250
local kernel = HMC({numSteps=1000, verbosity=0})
local scheduleFn = macro(function(iter, currTrace)
	return quote
		currTrace.temperature = temp
	end
end)
kernel = Schedule(kernel, scheduleFn)
local terra doInference()
	return [mcmc(staticsModel, kernel, {numsamps=numsamps, verbose=verbose})]
	-- return [forwardSample(staticsModel, numsamps)]
end
local samples = m.gc(doInference())
moviename = arg[1] or "movie"
renderSamples(samples, moviename, imageWidth, imageHeight)




