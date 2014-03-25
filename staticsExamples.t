terralib.require("prob")

local m = terralib.require("mem")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local Vec = terralib.require("linalg").Vec
local staticsUtils = terralib.require("staticsUtils")

local C = terralib.includecstring [[
#include <stdio.h>
]]

local function genExamples(gravityConstant, Connections)

	local Vec2 = Vec(real, 2)
	local BeamT = staticsUtils.Beam(real)
	local Ground = staticsUtils.Ground(real)
	local RigidSceneT = staticsUtils.RigidScene(real)

	-----------------------------------------

	local lerp = macro(function(lo, hi, t)
		return `(1.0-t)*lo + t*hi
	end)

	local polar2rect = macro(function(r, theta)
		return `Vec2.stackAlloc(r*ad.math.cos(theta), r*ad.math.sin(theta))
	end)

	local boundedUniform = macro(function(lo, hi, opts)
		local valquote = nil
		if opts then
			local OpsType = opts:gettype()
			local struct NewOpsType {}
			for _,e in ipairs(OpsType.entries) do
				table.insert(NewOpsType.entries, {field=e.field, type=e.type})
			end
			table.insert(NewOpsType.entries, {field="lowerBound", type=lo:gettype()})
			table.insert(NewOpsType.entries, {field="upperBound", type=hi:gettype()})
			table.insert(NewOpsType.entries, {field="structural", type=bool})
			valquote = quote
				var newopts = NewOpsType(opts)
				newopts.structural = false
				newopts.lowerBound = lo
				newopts.upperBound = hi
			in
				uniform(lo, hi, newopts)
			end
		else
			valquote = `uniform(lo, hi, {structural=false, lowerBound=lo, upperBound=hi})
		end
		return valquote
	end)

	-----------------------------------------

	local Examples = {}

	-- =============== OLD EXAMPLES WITH PHYSICALLY-INACCURATE UNITS ========================

	-- This is just a fudge so that the existing examples I made have the same behavior as before
	local oldBeamDepth = `1.0
	local oldBeamDensity = `0.1

	-- Define a simple scene with a beam rotating on a hinge
		--    attached to the ground. There's also a cable connecting
		--    the beam to the ground. The cable is inactive; it applies
		--    forces, but we don't solve for its equilibrium.
	Examples.simpleBeamHingeCableScene = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)

		var hingeY = groundHeight
		var hinge = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth*0.5, hingeY))
		
		var beamBot = Vec2.stackAlloc(hinge.location(0), hingeY)
		var beamLen = 20.0
		-- var beamAngle = [math.pi/4]
		var beamAngle = boundedUniform(0.0, [math.pi])
		var beamTop = beamBot + polar2rect(beamLen, beamAngle)
		var beamWidth = 3.0
		var beam = BeamT.heapAlloc(beamBot, beamTop, beamWidth, oldBeamDepth)
		beam.density = oldBeamDensity
		hinge:addObj(beam)

		var groundPinLoc = Vec2.stackAlloc(sceneWidth*0.2, groundHeight)
		var beamPinLoc = beamTop
		var cableWidth = 0.5
		var cable = Connections.Cable.heapAlloc(groundPinLoc, beamPinLoc, ground, beam, cableWidth)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam)
		scene.objects:push(cable:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge)
		connections:push(cable)

		return scene, connections
	end)

	-- Slightly more complicated example: two hinged beams connected at the top
	--    by a cable.
	Examples.twoBeamsConnectedByCableScene = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
		var beamLength = 20.0
		var beamWidth = 3.0
		var beam1angle = boundedUniform(0.0, [math.pi])
		var beam2angle = boundedUniform(0.0, [math.pi])
		-- var beam1angle = [3*math.pi/4]
		-- var beam2angle = [math.pi/4]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth, oldBeamDepth)
		beam1.density = oldBeamDensity
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth, oldBeamDepth)
		beam2.density = oldBeamDensity
		hinge1:addObj(beam1)
		hinge2:addObj(beam2)

		var cable = Connections.Cable.heapAlloc(beam1:endpoint(1), beam2:endpoint(1), beam1, beam2, 0.5)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(cable:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(cable)

		return scene, connections
	end)

	-- Slighty wacky "bridge" with one link
	Examples.singleLinkWackyBridge = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 50.0
		var sceneHeight = 50.0
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(sceneWidth/3.0, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(2.0*sceneWidth/3.0, groundHeight))
		var beamLength = 20.0
		var beamWidth = 3.0
		var beam1angle = boundedUniform(0.0, [math.pi])
		var beam2angle = boundedUniform(0.0, [math.pi])
		-- var beam1angle = [0.6*math.pi]
		-- var beam2angle = [0.1*math.pi]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth, oldBeamDepth)
		beam1.density = oldBeamDensity
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth, oldBeamDepth)
		beam2.density = oldBeamDensity
		hinge1:addObj(beam1)
		hinge2:addObj(beam2)
		var platformHeight = boundedUniform(10.0, 30.0)
		-- var platformHeight = 10.0
		var platformWidth = 3.0
		var platformLength = 10.0
		var platform = BeamT.heapAlloc(Vec2.stackAlloc(sceneWidth*0.5 - platformLength*0.5, platformHeight + platformWidth*0.5),
									   Vec2.stackAlloc(sceneWidth*0.5 + platformLength*0.5, platformHeight + platformWidth*0.5),
									   platformWidth, oldBeamDepth)
		platform.density = oldBeamDensity
		var cableWidth = 0.5
		var cable1 = Connections.Cable.heapAlloc(beam1:endpoint(1), platform:endpoint(0), beam1, platform, cableWidth)
		var cable2 = Connections.Cable.heapAlloc(beam2:endpoint(1), platform:endpoint(1), beam2, platform, cableWidth)

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		scene.objects:push(ground)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		scene.objects:push(platform)
		scene.objects:push(cable1:createProxy())
		scene.objects:push(cable2:createProxy())
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		connections:push(hinge1)
		connections:push(hinge2)
		connections:push(cable1)
		connections:push(cable2)

		return scene, connections
	end)

	-- Multi-link wacky bridge with rotatable links
	Examples.multiLinkWackyBridge = pfn(terra(numLinks: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Set up end support beams
		var hinge1 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(0.2*sceneWidth, groundHeight))
		var hinge2 = Connections.Hinge.heapAlloc(Vec2.stackAlloc(0.8*sceneWidth, groundHeight))
		var beamLength = 40.0
		var beamWidth = 4.0
		var beam1angle = boundedUniform(0.0, [math.pi])
		var beam2angle = boundedUniform(0.0, [math.pi])
		-- var beam1angle = [math.pi/2]
		-- var beam2angle = [math.pi/2]
		var beam1 = BeamT.heapAlloc(hinge1.location, hinge1.location + polar2rect(beamLength, beam1angle), beamWidth, oldBeamDepth)
		beam1.density = oldBeamDensity
		var beam2 = BeamT.heapAlloc(hinge2.location, hinge2.location + polar2rect(beamLength, beam2angle), beamWidth, oldBeamDepth)
		beam2.density = oldBeamDensity
		hinge1:addObj(beam1)
		hinge2:addObj(beam2)
		scene.objects:push(beam1)
		scene.objects:push(beam2)
		connections:push(hinge1)
		connections:push(hinge2)

		-- Create platforms
		var platforms = [Vector(&BeamT)].stackAlloc()
		var platxmin = beam1:endpoint(0)(0)
		var platxmax = beam2:endpoint(0)(0)
		var platxrange = platxmax - platxmin
		for i=0,numLinks do
			var t = ((i+1)/double(numLinks+1))
			var centerx = platxmin + t*platxrange
			var centery = boundedUniform(10.0, 40.0)
			var center = Vec2.stackAlloc(centerx, centery)
			-- var rot = 0.0
			-- var rot = gaussian(0.0, [math.pi/20], {structural=false})
			var rot = boundedUniform([-math.pi/6], [math.pi/6])
			var width = 1.5
			var length = 4.0
			var longAxis = polar2rect(length, rot)
			var platform = BeamT.heapAlloc(center - longAxis, center+longAxis, width, oldBeamDepth)
			platform.density = oldBeamDensity
			platforms:push(platform)
			scene.objects:push(platform)
		end

		-- Link everything together with cables
		var startBeams = [Vector(&BeamT)].stackAlloc()
		var startEndpoints = [Vector(uint)].stackAlloc()
		var endBeams = [Vector(&BeamT)].stackAlloc()
		var endEndpoints = [Vector(uint)].stackAlloc()
		startBeams:push(beam1)
		startEndpoints:push(1)
		for i=0,platforms.size do
			endBeams:push(platforms(i))
			endEndpoints:push(0)
			startBeams:push(platforms(i))
			startEndpoints:push(1)
		end
		endBeams:push(beam2)
		endEndpoints:push(1)
		m.destruct(platforms)
		for i=0,startBeams.size do
			var width = 0.4
			var cable = Connections.Cable.heapAlloc(startBeams(i):endpoint(startEndpoints(i)),
										 			endBeams(i):endpoint(endEndpoints(i)),
										 			startBeams(i), endBeams(i), width)
			scene.objects:push(cable:createProxy())
			connections:push(cable)
		end
		m.destruct(startBeams)
		m.destruct(startEndpoints)
		m.destruct(endBeams)
		m.destruct(endEndpoints)

		return scene, connections
	end)

	-- Multi-link bridge with rotatable links and sliding support columns
	Examples.multiLinkSlidingBridge = pfn(terra(numLinks: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0

		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Set up end support beams
		var beamSepWidth = uniform(0.5*sceneWidth, 0.99*sceneWidth, {structural=false, lowerBound=0.5*sceneWidth, upperBound=0.99*sceneWidth})
		var beam1x = 0.5*sceneWidth - 0.5*beamSepWidth
		var beam2x = 0.5*sceneWidth + 0.5*beamSepWidth
		var beam1bot = Vec2.stackAlloc(beam1x, groundHeight)
		var beam2bot = Vec2.stackAlloc(beam2x, groundHeight)
		var beamLength = 40.0
		var beamWidth = 4.0
		var beam1 = BeamT.heapAlloc(beam1bot, beam1bot + Vec2.stackAlloc(0.0, beamLength), beamWidth, oldBeamDepth)
		beam1.density = oldBeamDensity
		beam1:disable()
		var beam2 = BeamT.heapAlloc(beam2bot, beam2bot + Vec2.stackAlloc(0.0, beamLength), beamWidth, oldBeamDepth)
		beam2.density = oldBeamDensity
		beam2:disable()
		scene.objects:push(beam1)
		scene.objects:push(beam2)

		-- Create platforms
		var platforms = [Vector(&BeamT)].stackAlloc()
		var platxmin = beam1:endpoint(0)(0)
		var platxmax = beam2:endpoint(0)(0)
		var platxrange = platxmax - platxmin
		for i=0,numLinks do
			var t = ((i+1)/double(numLinks+1))
			var centerx = platxmin + t*platxrange
			var centery = boundedUniform(5.0, 40.0)
			var center = Vec2.stackAlloc(centerx, centery)
			-- var rot = 0.0
			-- var rot = gaussian(0.0, [math.pi/20], {structural=false})
			var rot = boundedUniform([-math.pi/4], [math.pi/4])
			var width = 1.5
			var length = 4.0
			var longAxis = polar2rect(length, rot)
			var platform = BeamT.heapAlloc(center - longAxis, center+longAxis, width, oldBeamDepth)
			platform.density = oldBeamDensity
			platforms:push(platform)
			scene.objects:push(platform)
		end

		-- Link everything together with cables
		var startBeams = [Vector(&BeamT)].stackAlloc()
		var startEndpoints = [Vector(uint)].stackAlloc()
		var endBeams = [Vector(&BeamT)].stackAlloc()
		var endEndpoints = [Vector(uint)].stackAlloc()
		startBeams:push(beam1)
		startEndpoints:push(1)
		for i=0,platforms.size do
			endBeams:push(platforms(i))
			endEndpoints:push(0)
			startBeams:push(platforms(i))
			startEndpoints:push(1)
		end
		endBeams:push(beam2)
		endEndpoints:push(1)
		m.destruct(platforms)
		for i=0,startBeams.size do
			var width = 0.4
			var cable = Connections.Cable.heapAlloc(startBeams(i):endpoint(startEndpoints(i)),
										 			endBeams(i):endpoint(endEndpoints(i)),
										 			startBeams(i), endBeams(i), width)
			scene.objects:push(cable:createProxy())
			connections:push(cable)
		end
		m.destruct(startBeams)
		m.destruct(startEndpoints)
		m.destruct(endBeams)
		m.destruct(endEndpoints)

		return scene, connections
	end)

	-- Single-spar cable-stayed bridge
	local genCableSpan = macro(function()
		return quote
			var spanBegin = boundedUniform(0.1, 0.4)
			var spanEnd = boundedUniform(0.6, 0.9)
		in
			spanBegin, spanEnd
		end
	end)
	Examples.cableStayedBridge = pfn(terra(numCablePairs: uint)
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Platform
		var platformWidth = boundedUniform(0.5*sceneWidth, 0.99*sceneWidth)
		var platformHeight = boundedUniform(0.1*sceneWidth, 0.3*sceneWidth)
		var platformLeft = Vec2.stackAlloc(0.5*sceneWidth - 0.5*platformWidth, platformHeight)
		var platformRight = Vec2.stackAlloc(0.5*sceneWidth + 0.5*platformWidth, platformHeight)
		var platformCenter = 0.5*(platformLeft + platformRight)
		var platformThickness = 4.0
		var platform = BeamT.heapAlloc(platformLeft, platformRight, platformThickness, oldBeamDepth)
		platform.density = oldBeamDensity
		scene.objects:push(platform)

		-- Support beams
		var beamWidth = 4.0
		var beamBotY = groundHeight
		var beamTopY = platformHeight - 0.5*platformThickness
		var beam1xlo = platformLeft(0) + 0.5*beamWidth
		var beam1xhi = platformCenter(0) - 0.5*beamWidth
		-- var beam1x = boundedUniform(beam1xlo, beam1xhi)
		var beam1x = beam1xlo
		var beam2xlo = platformCenter(0) + 0.5*beamWidth
		var beam2xhi = platformRight(0) - 0.5*beamWidth
		-- var beam2x = boundedUniform(beam2xlo, beam2xhi)
		var beam2x = beam2xhi
		var beam1Bot = Vec2.stackAlloc(beam1x, beamBotY)
		var beam1Top = Vec2.stackAlloc(beam1x, beamTopY)
		var beam2Bot = Vec2.stackAlloc(beam2x, beamBotY)
		var beam2Top = Vec2.stackAlloc(beam2x, beamTopY)
		var beam1 = BeamT.heapAlloc(beam1Bot, beam1Top, beamWidth, oldBeamDepth)
		beam1.density = oldBeamDensity
		var beam2 = BeamT.heapAlloc(beam2Bot, beam2Top, beamWidth, oldBeamDepth)
		beam2.density = oldBeamDensity
		scene.objects:push(beam1)
		scene.objects:push(beam2)

		-- Spar
		var sparWidth = 4.0
		var sparLength = boundedUniform(0.5*platformWidth, platformWidth)
		var sparBotParam = boundedUniform(0.1, 0.9)
		var sparLowerBound = Vec2.stackAlloc(platformLeft(0) + 0.5*sparWidth, platformLeft(1))
		var sparUpperBound = Vec2.stackAlloc(platformRight(0) - 0.5*sparWidth, platformLeft(1))
		var sparBot = lerp(sparLowerBound, sparUpperBound, sparBotParam)
		var sparBotLeft = Vec2.stackAlloc(sparBot(0)-0.5*sparWidth, sparBot(1))
		var sparBotRight = Vec2.stackAlloc(sparBot(0)+0.5*sparWidth, sparBot(1))
		var sparAngle = boundedUniform(0.0, [math.pi])
		var sparTop = sparBot + polar2rect(sparLength, sparAngle)
		var spar = BeamT.heapAlloc(sparBot, sparTop, sparWidth, oldBeamDepth)
		spar.density = oldBeamDensity		
		scene.objects:push(spar)

		-- Support beams are stacked on the ground
		connections:push(Connections.Stacking.heapAlloc(ground, beam1))
		connections:push(Connections.Stacking.heapAlloc(ground, beam2))

		-- Support beams are welded to platform
		connections:push(Connections.Weld.heapAlloc(platform, beam1, 2, 3))
		connections:push(Connections.Weld.heapAlloc(platform, beam2, 2, 3))

		-- Spar is connected to platform by a hinge
		var hinge = Connections.Hinge.heapAlloc(sparBot)
		hinge:addObj(spar)
		connections:push(hinge)

		-- Spar is connected to platform by cables
		var cableWidth = 0.4
		var sparAxis = sparTop - sparBot; sparAxis:normalize()
		var sparPerp = staticsUtils.perp(sparAxis)
		var platformLeftVec = platformLeft - platformCenter; platformLeftVec:normalize()
		var platformRightVec = platformRight - platformCenter; platformRightVec:normalize()
		if sparPerp:dot(platformLeftVec) > sparPerp:dot(platformRightVec) then
			sparPerp = -sparPerp    -- Guarantee perp points toward right edge of scene
		end
		var sparSpanBegin, sparSpanEnd = genCableSpan()
		var platLeftSpanBegin, platLeftSpanEnd = genCableSpan()
		var platRightSpanBegin, platRightSpanEnd = genCableSpan()
		for i=0,numCablePairs do
			var t = (i + 0.5)/numCablePairs
			var sparT = lerp(sparSpanBegin, sparSpanEnd, t)
			var platLeftT = lerp(platLeftSpanBegin, platLeftSpanEnd, t)
			var platRightT = lerp(platRightSpanBegin, platRightSpanEnd, t)
			var sparCenterPoint = lerp(sparBot, sparTop, sparT)
			var sparLeftPoint = sparCenterPoint - 0.5*sparWidth*sparPerp
			var sparRightPoint = sparCenterPoint + 0.5*sparWidth*sparPerp
			var platLeftPoint = lerp(sparBotLeft, platformLeft, platLeftT)
			platLeftPoint(1) = platLeftPoint(1) + 0.5*platformThickness
			var platRightPoint = lerp(sparBotRight, platformRight, platRightT)
			platRightPoint(1) = platRightPoint(1) + 0.5*platformThickness
			var cableLeft = Connections.Cable.heapAlloc(platLeftPoint, sparLeftPoint, platform, spar, cableWidth)
			var cableRight = Connections.Cable.heapAlloc(platRightPoint, sparRightPoint, platform, spar, cableWidth)
			scene.objects:push(cableLeft:createProxy())
			scene.objects:push(cableRight:createProxy())
			connections:push(cableLeft)
			connections:push(cableRight)
		end

		return scene, connections
	end)

	-- A (simple?) hanging structure
	-- It looks kind of like this:
	--             [=]
	--              |
	--          [=======]
	--          /       \
	--      [=====]   [=====]
	--      |     \   /     |
	--      |    [=====]    |
	--      |     /   \     |
	--      [=====]   [=====]
	--         \         /
	--          [=======]
	Examples.hangingStructure = pfn(terra()
		var sceneWidth = 100.0
		var sceneHeight = 100.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var beamThickness = 4.0

		-- Anchor for everything to hang from
		var anchorCenter = Vec2.stackAlloc(0.5*sceneWidth, 0.95*sceneHeight)
		var anchorSize = beamThickness
		var anchor = BeamT.heapAlloc(anchorCenter, anchorSize, anchorSize, 0.0, oldBeamDepth)
		anchor.density = oldBeamDensity
		anchor:disable()
		scene.objects:push(anchor)

		-- Generate top and bottom beams
		var topBeamCenter = Vec2.stackAlloc(0.5*sceneWidth, 0.8*sceneHeight)
		var topBeamLen = boundedUniform(0.1*sceneWidth, 0.5*sceneWidth)
		-- var topBeamLen = 0.5*sceneWidth
		var topBeam = BeamT.heapAlloc(topBeamCenter, topBeamLen, beamThickness, 0.0, oldBeamDepth)
		topBeam.density = oldBeamDensity
		scene.objects:push(topBeam)
		var botBeamCenter = Vec2.stackAlloc(0.5*sceneWidth, topBeamCenter(1) - boundedUniform(0.4*sceneHeight, 0.8*sceneHeight))
		-- var botBeamCenter = Vec2.stackAlloc(0.5*sceneWidth, topBeamCenter(1) - 0.6*sceneHeight)
		var botBeam = BeamT.heapAlloc(botBeamCenter, topBeamLen, beamThickness, 0.0, oldBeamDepth)
		botBeam.density = oldBeamDensity
		scene.objects:push(botBeam)

		-- The middle beam should be somewhere between the top and bottom beams
		var midBeamCenterT = boundedUniform(0.2, 0.8)
		-- var midBeamCenterT = 0.5
		var midBeamCenter = lerp(botBeamCenter, topBeamCenter, midBeamCenterT)
		var midBeamLen = boundedUniform(0.5*topBeamLen, topBeamLen)
		-- var midBeamLen = 0.5*topBeamLen
		var midBeam = BeamT.heapAlloc(midBeamCenter, midBeamLen, beamThickness, 0.0, oldBeamDepth)
		midBeam.density = oldBeamDensity
		scene.objects:push(midBeam)

		-- The side beams are sandwiched between the top/mid/bottom beams.
		-- They can move side to side a bit (though they must stay horizontally symmetric).
		-- They can also rotate a bit.

		-- Side beams between the mid and top beam
		var sideBeams1HeightT = boundedUniform(0.2, 0.8)
		-- var sideBeams1HeightT = 0.5
		var sideBeams1Height = lerp(midBeamCenter(1), topBeamCenter(1), sideBeams1HeightT)
		var sideBeams1Len = boundedUniform(0.5*topBeamLen, topBeamLen)
		-- var sideBeams1Len = 0.5*topBeamLen
		-- var sideBeams1LeftCenterX = boundedUniform(0.5*sideBeams1Len, midBeam.endpoints[0](0)-0.5*sideBeams1Len)
		var sideBeams1LeftCenterX = boundedUniform(0.5*sideBeams1Len, midBeamCenter(0)-0.5*sideBeams1Len)
		-- var sideBeams1LeftCenterX = 0.5*(0.5*sideBeams1Len + (midBeam.endpoints[0](0) - 0.5*sideBeams1Len))
		var sideBeams1LeftCenter = Vec2.stackAlloc(sideBeams1LeftCenterX, sideBeams1Height)
		var sideBeams1RightCenterX = (midBeamCenter(0) - sideBeams1LeftCenterX) + midBeamCenter(0)
		var sideBeams1RightCenter = Vec2.stackAlloc(sideBeams1RightCenterX, sideBeams1Height)
		var sideBeams1LeftAngle = boundedUniform([-math.pi/6], [math.pi/6])
		-- var sideBeams1LeftAngle = 0.0
		var sideBeams1RightAngle = -sideBeams1LeftAngle
		var sideBeams1Left = BeamT.heapAlloc(sideBeams1LeftCenter, sideBeams1Len, beamThickness, sideBeams1LeftAngle, oldBeamDepth)
		sideBeams1Left.density = oldBeamDensity
		var sideBeams1Right = BeamT.heapAlloc(sideBeams1RightCenter, sideBeams1Len, beamThickness, sideBeams1RightAngle, oldBeamDepth)
		sideBeams1Right.density = oldBeamDensity
		scene.objects:push(sideBeams1Left)
		scene.objects:push(sideBeams1Right)

		-- Side beams between the bottom and mid beam
		var sideBeams2HeightT = boundedUniform(0.2, 0.8)
		-- var sideBeams2HeightT = 0.5
		var sideBeams2Height = lerp(botBeamCenter(1), midBeamCenter(1), sideBeams2HeightT)
		var sideBeams2Len = boundedUniform(0.5*topBeamLen, topBeamLen)
		-- var sideBeams2Len = 0.5*topBeamLen
		-- var sideBeams2LeftCenterX = boundedUniform(0.5*sideBeams2Len, midBeam.endpoints[0](0)-0.5*sideBeams2Len)
		var sideBeams2LeftCenterX = boundedUniform(0.5*sideBeams2Len, midBeamCenter(0)-0.5*sideBeams2Len)
		-- var sideBeams2LeftCenterX = 0.5*(0.5*sideBeams2Len + (midBeam.endpoints[0](0)-0.5*sideBeams2Len))
		var sideBeams2LeftCenter = Vec2.stackAlloc(sideBeams2LeftCenterX, sideBeams2Height)
		var sideBeams2RightCenterX = (midBeamCenter(0) - sideBeams2LeftCenterX) + midBeamCenter(0)
		var sideBeams2RightCenter = Vec2.stackAlloc(sideBeams2RightCenterX, sideBeams2Height)
		var sideBeams2LeftAngle = boundedUniform([-math.pi/6], [math.pi/6])
		-- var sideBeams2LeftAngle = 0.0
		var sideBeams2RightAngle = -sideBeams2LeftAngle
		var sideBeams2Left = BeamT.heapAlloc(sideBeams2LeftCenter, sideBeams2Len, beamThickness, sideBeams2LeftAngle, oldBeamDepth)
		sideBeams2Left.density = oldBeamDensity
		var sideBeams2Right = BeamT.heapAlloc(sideBeams2RightCenter, sideBeams2Len, beamThickness, sideBeams2RightAngle, oldBeamDepth)
		sideBeams2Right.density = oldBeamDensity
		scene.objects:push(sideBeams2Left)
		scene.objects:push(sideBeams2Right)

		-- Wire things up with cables
		var cableWidth = 0.4
		var anchorTopCable = Connections.Cable.heapAlloc(anchorCenter, topBeamCenter, anchor, topBeam, cableWidth)
		var topSideLeftCable = Connections.Cable.heapAlloc(topBeam:endpoint(0), sideBeams1LeftCenter, topBeam, sideBeams1Left, cableWidth)
		var topSideRightCable = Connections.Cable.heapAlloc(topBeam:endpoint(1), sideBeams1RightCenter, topBeam, sideBeams1Right, cableWidth)
		var sideLeftsCable = Connections.Cable.heapAlloc(sideBeams1Left:endpoint(0), sideBeams2Left:endpoint(0), sideBeams1Left, sideBeams2Left, cableWidth)
		var sideRightsCable = Connections.Cable.heapAlloc(sideBeams1Right:endpoint(1), sideBeams2Right:endpoint(1), sideBeams1Right, sideBeams2Right, cableWidth)
		var sideLeftMidCable = Connections.Cable.heapAlloc(sideBeams1Left:endpoint(1), midBeam:endpoint(0), sideBeams1Left, midBeam, cableWidth)
		var sideRightMidCable = Connections.Cable.heapAlloc(sideBeams1Right:endpoint(0), midBeam:endpoint(1), sideBeams1Right, midBeam, cableWidth)
		var midSideLeftCable = Connections.Cable.heapAlloc(midBeam:endpoint(0), sideBeams2Left:endpoint(1), midBeam, sideBeams2Left, cableWidth)
		var midSideRightCable = Connections.Cable.heapAlloc(midBeam:endpoint(1), sideBeams2Right:endpoint(0), midBeam, sideBeams2Right, cableWidth)
		var sideLeftBotCable = Connections.Cable.heapAlloc(sideBeams2LeftCenter, botBeam:endpoint(0), sideBeams2Left, botBeam, cableWidth)
		var sideRightBotCable = Connections.Cable.heapAlloc(sideBeams2RightCenter, botBeam:endpoint(1), sideBeams2Right, botBeam, cableWidth)
		scene.objects:push(anchorTopCable:createProxy())
		scene.objects:push(topSideLeftCable:createProxy())
		scene.objects:push(topSideRightCable:createProxy())
		scene.objects:push(sideLeftsCable:createProxy())
		scene.objects:push(sideRightsCable:createProxy())
		scene.objects:push(sideLeftMidCable:createProxy())
		scene.objects:push(sideRightMidCable:createProxy())
		scene.objects:push(midSideLeftCable:createProxy())
		scene.objects:push(midSideRightCable:createProxy())
		scene.objects:push(sideLeftBotCable:createProxy())
		scene.objects:push(sideRightBotCable:createProxy())
		connections:push(anchorTopCable)
		connections:push(topSideLeftCable)
		connections:push(topSideRightCable)
		connections:push(sideLeftsCable)
		connections:push(sideRightsCable)
		connections:push(sideLeftMidCable)
		connections:push(sideRightMidCable)
		connections:push(midSideLeftCable)
		connections:push(midSideRightCable)
		connections:push(sideLeftBotCable)
		connections:push(sideRightBotCable)

		return scene, connections
	end)

	Examples.simpleHangingStructure = pfn(terra()
		var sceneWidth = 100.0
		var sceneHeight = 100.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()

		var beamThickness = 4.0
		var cableThickness = 0.4

		-- Anchors
		var anchor1center = Vec2.stackAlloc(0.25*sceneWidth, 0.95*sceneHeight)
		var anchor1 = BeamT.heapAlloc(anchor1center, beamThickness, beamThickness, 0.0, oldBeamDepth)
		anchor1.density = oldBeamDensity
		anchor1:disable()
		var anchor2center = Vec2.stackAlloc(0.75*sceneWidth, 0.95*sceneHeight)
		var anchor2 = BeamT.heapAlloc(anchor2center, beamThickness, beamThickness, 0.0, oldBeamDepth)
		anchor2.density = oldBeamDensity
		anchor2:disable()
		scene.objects:push(anchor1)
		scene.objects:push(anchor2)

		-- Bar
		var barLength = anchor2center(0) - anchor1center(0)
		var barRot = boundedUniform([-math.pi/4.0], [math.pi/4.0])
		-- var barRot = gaussian(0.0, [math.pi/4.0], {structural=false, initialVal=0.0})
		-- var barDisplace = boundedUniform(0.2*sceneHeight, 0.6*sceneHeight)
		var barDisplace = 0.5*sceneHeight
		var barCenter = 0.5*(anchor1center + anchor2center)
		barCenter(1) = barCenter(1) - barDisplace
		var bar = BeamT.heapAlloc(barCenter, barLength, beamThickness, barRot, oldBeamDepth)
		bar.density = oldBeamDensity
		scene.objects:push(bar)

		-- Cables
		var cable1 = Connections.Cable.heapAlloc(anchor1center, bar:endpoint(0), anchor1, bar, cableThickness)
		var cable2 = Connections.Cable.heapAlloc(anchor2center, bar:endpoint(1), anchor2, bar, cableThickness)
		scene.objects:push(cable1:createProxy())
		scene.objects:push(cable2:createProxy())
		connections:push(cable1)
		connections:push(cable2)

		return scene, connections
	end)

	Examples.simpleContactTest = pfn(terra()
		var groundHeight = 2.0
		var sceneWidth = 100.0
		var sceneHeight = 100.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		var beamThickness = 4.0

		-- Objects
		-- var platformHeight = 0.5*sceneHeight
		-- var platformLength = 0.75*sceneWidth
		var platformHeight = boundedUniform(0.1*sceneHeight, 0.5*sceneHeight)
		var platformLength = boundedUniform(0.4*sceneWidth, 0.75*sceneWidth)
		var platform = BeamT.heapAlloc(Vec2.stackAlloc(0.5*sceneWidth, groundHeight + platformHeight),
									   platformLength, beamThickness, 0.0, oldBeamDepth)
		platform.density = oldBeamDensity
		var support1 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(0)(0) + 0.5*beamThickness, groundHeight),
									   platform:endpoint(0) + 0.5*Vec2.stackAlloc(beamThickness, -beamThickness),
									   beamThickness, oldBeamDepth)
		support1.density = oldBeamDensity
		var support2 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(1)(0) - 0.5*beamThickness, groundHeight),
									   platform:endpoint(1) - 0.5*Vec2.stackAlloc(beamThickness, beamThickness),
									   beamThickness, oldBeamDepth)
		support2.density = oldBeamDensity
		scene.objects:push(platform)
		scene.objects:push(support1)
		scene.objects:push(support2)

		-- Connections
		var gs1a, gs1b = Connections.FrictionalContact.makeBeamContacts(support1, ground, 0, 1)
		var gs2a, gs2b = Connections.FrictionalContact.makeBeamContacts(support2, ground, 0, 1)
		var ps1a, ps1b = Connections.FrictionalContact.makeBeamContacts(support1, platform, 2, 3)
		var ps2a, ps2b = Connections.FrictionalContact.makeBeamContacts(support2, platform, 2, 3)
		connections:push(gs1a); connections:push(gs1b)
		connections:push(gs2a); connections:push(gs2b)
		connections:push(ps1a); connections:push(ps1b)
		connections:push(ps2a); connections:push(ps2b)

		return scene, connections
	end)

	-- =============== NEW, PHYSICALLY-ACCURATE EXAMPLES ========================

	local mm = macro(function(x)
		return `0.001*x
	end)

	local radians = macro(function(x)
		return `x*[math.pi]/180.0
	end)

	Examples.simpleNailTest = pfn(terra()
		var groundHeight = mm(5.0)
		var sceneWidth = mm(250.0)
		var sceneHeight = mm(250.0)
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Approx. measurements for a gauge 14-1/2 box nail
		var nailDiameter = mm(2.0)
		var nailLength = mm(32.0)

		-- We'd like to have:
		--  * penetrationDepth > 10*nailDiameter
		--  * penetrationDepth ~= 2*sideMemberThickness --> sideMemberThickness ~= 1/3 * nailLength
		-- So
		--  * penetrationDepth > 20mm
		--  * sideMemberThickness ~= 10mm

		-- We'll use beams with square cross sections; that is, width == depth
		var beamThickness = mm(10.0)
		var halfThickness = 0.5*beamThickness

		-- Objects
		var platformHeight = 0.3*sceneHeight
		var platformLength = 0.75*sceneWidth
		var platform = BeamT.heapAlloc(Vec2.stackAlloc(0.5*sceneWidth, groundHeight + platformHeight),
									   platformLength, beamThickness, 0.0, beamThickness)
		-- platform:disable()
		var support1 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(0)(0)-halfThickness, groundHeight),
									   Vec2.stackAlloc(platform:endpoint(0)(0)-halfThickness, 0.5*sceneHeight),
									   beamThickness, beamThickness)
		-- support1:disable()
		var support2 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(1)(0)+halfThickness, groundHeight),
							   		   Vec2.stackAlloc(platform:endpoint(1)(0)+halfThickness, 0.5*sceneHeight),
							   		   beamThickness, beamThickness)
		-- support2:disable()
		scene.objects:push(platform)
		scene.objects:push(support1)
		scene.objects:push(support2)

		-- Connections
		var gs1a, gs1b = Connections.FrictionalContact.makeBeamContacts(support1, ground, 0, 1)
		var gs2a, gs2b = Connections.FrictionalContact.makeBeamContacts(support2, ground, 0, 1)
		var ps1 = Connections.NailJoint.heapAlloc(platform, support1, 0, 1, nailDiameter, nailLength, 1)
		var ps2 = Connections.NailJoint.heapAlloc(platform, support2, 2, 3, nailDiameter, nailLength, 1)
		connections:push(ps1)
		connections:push(ps2)
		connections:push(gs1a); connections:push(gs1b)
		connections:push(gs2a); connections:push(gs2b)

		-- Extra load
		platform:applyExternalLoad(gravityConstant, 20.0, platform:centerOfMass() + Vec2.stackAlloc(0.0, halfThickness))

		return scene, connections
	end)

	Examples.simpleNailTest2 = pfn(terra()
		var groundHeight = mm(5.0)
		var sceneWidth = mm(250.0)
		var sceneHeight = mm(250.0)
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Approx. measurements for a gauge 14-1/2 box nail
		var nailDiameter = mm(2.0)
		var nailLength = mm(32.0)

		-- We'd like to have:
		--  * penetrationDepth > 10*nailDiameter
		--  * penetrationDepth ~= 2*sideMemberThickness --> sideMemberThickness ~= 1/3 * nailLength
		-- So
		--  * penetrationDepth > 20mm
		--  * sideMemberThickness ~= 10mm

		-- We'll use beams with square cross sections; that is, width == depth
		var beamThickness = mm(10.0)
		var halfThickness = 0.5*beamThickness

		-- Objects
		var platformHeight = 0.4*sceneHeight
		var platformLength = 0.75*sceneWidth
		var platform = BeamT.heapAlloc(Vec2.stackAlloc(0.5*sceneWidth, groundHeight + platformHeight),
									   platformLength, beamThickness, 0.0, beamThickness)
		-- platform:disable()
		var support1 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(0)(0)+halfThickness, groundHeight),
									   Vec2.stackAlloc(platform:endpoint(0)(0)+halfThickness, platform:endpoint(0)(1)-halfThickness),
									   beamThickness, beamThickness)
		-- support1:disable()
		var support2 = BeamT.heapAlloc(Vec2.stackAlloc(platform:endpoint(1)(0)-halfThickness, groundHeight),
							   		   Vec2.stackAlloc(platform:endpoint(1)(0)-halfThickness, platform:endpoint(1)(1)-halfThickness),
							   		   beamThickness, beamThickness)
		-- support2:disable()
		scene.objects:push(platform)
		scene.objects:push(support1)
		scene.objects:push(support2)

		-- Connections
		var gs1a, gs1b = Connections.FrictionalContact.makeBeamContacts(support1, ground, 0, 1)
		var gs2a, gs2b = Connections.FrictionalContact.makeBeamContacts(support2, ground, 0, 1)
		var ps1 = Connections.NailJoint.heapAlloc(support1, platform, 2, 3, nailDiameter, nailLength, 1)
		var ps2 = Connections.NailJoint.heapAlloc(support2, platform, 2, 3, nailDiameter, nailLength, 1)
		connections:push(ps1)
		connections:push(ps2)
		connections:push(gs1a); connections:push(gs1b)
		connections:push(gs2a); connections:push(gs2b)

		-- Extra load
		platform:applyExternalLoad(gravityConstant, 20.0, platform:centerOfMass() + Vec2.stackAlloc(0.0, halfThickness))

		return scene, connections
	end)

	Examples.aFrameTest = pfn(terra()
		var groundHeight = mm(5.0)
		var sceneWidth = mm(250.0)
		var sceneHeight = mm(250.0)
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Approx. measurements for a gauge 14-1/2 box nail
		var nailDiameter = mm(2.0)
		var nailLength = mm(32.0)

		-- We'll use beams with square cross sections; that is, width == depth
		var beamThickness = mm(10.0)
		var halfThickness = 0.5*beamThickness

		-- Objects

		var left = Vec2.stackAlloc(-1.0, 0.0)

		var support1BaseX = 0.15*sceneWidth
		var support2BaseX = 0.85*sceneWidth
		-- var support1Ang = [0.5*math.pi]
		-- var support2Ang = [0.5*math.pi]
		-- var support1Ang = [0.45*math.pi]
		-- var support2Ang = [0.55*math.pi]
		-- var support1Ang = [0.35*math.pi]
		-- var support2Ang = [0.65*math.pi]
		var support1Ang = boundedUniform([0.35*math.pi], [0.65*math.pi])
		var support2Ang = boundedUniform([0.35*math.pi], [0.65*math.pi])
		var supportLength = 0.5*sceneHeight
		var support1 = BeamT.heapAlloc(supportLength, beamThickness, support1Ang, Vec2.stackAlloc(support1BaseX, groundHeight), beamThickness)
		var support2 = BeamT.heapAlloc(supportLength, beamThickness, support2Ang, Vec2.stackAlloc(support2BaseX, groundHeight), beamThickness)
		scene.objects:push(support1)
		scene.objects:push(support2)

		var platformT = 0.6
		-- var platformT = boundedUniform(0.1, 0.9)
		var platform = BeamT.createBridgingBeam(support1, support2, platformT, platformT, beamThickness, beamThickness)
		scene.objects:push(platform)
		-- platform:disable()

		-- Connections
		-- var gs1a, gs1b = Connections.FrictionalContact.makeBeamContacts(support1, ground, 0, 1)
		-- var gs2a, gs2b = Connections.FrictionalContact.makeBeamContacts(support2, ground, 0, 1)
		var gs1 = Connections.FrictionalContact.heapAlloc(support1, ground, 0, 1)
		var gs2 = Connections.FrictionalContact.heapAlloc(support2, ground, 0, 1)
		var ps1 = Connections.NailJoint.heapAlloc(platform, support1, 0, 1, nailDiameter, nailLength, 1)
		var ps2 = Connections.NailJoint.heapAlloc(platform, support2, 2, 3, nailDiameter, nailLength, 1)
		connections:push(ps1)
		connections:push(ps2)
		-- connections:push(gs1a); connections:push(gs1b)
		-- connections:push(gs2a); connections:push(gs2b)
		connections:push(gs1)
		connections:push(gs2)

		-- Extra load
		platform:applyExternalLoad(gravityConstant, 40.0, platform:centerOfMass() + Vec2.stackAlloc(0.0, halfThickness))

		return scene, connections
	end)

	Examples.funkyTable = pfn(terra()
		var groundHeight = mm(10.0)
		var sceneWidth = mm(915.0)	-- Roughly 3 feet
		var sceneHeight = mm(915.0)
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		-- Approx. measurements for a gauge 14-1/2 box nail
		var nailDiameter = mm(2.0)
		var nailLength = mm(32.0)
		-- var nailDiameter = mm(4.0)
		-- var nailLength = mm(64.0)

		var beamThickness = mm(10.0)
		-- var beamThickness = mm(20.0)
		var beamDepth = mm(200.0) -- Here, we'll assume the beams extend back into the screen to form boards
		-- var beamDepth = beamThickness

		-- We insert multiple nails at each connection, depending on how deep the beams are
		var nailSpacing = mm(20.0)
		-- var numNails = uint(beamDepth/nailSpacing)
		var numNails = 1

		-- Parameters
		var baseCenterX = 0.6*sceneWidth
		-- var baseWidth = 0.4*sceneWidth
		var mainAngle = radians(35.0)
		var supportConnectT = 0.5
		var shelfHeight = 0.65*sceneHeight
		-- var shelfLength = 0.45*sceneWidth
		var secondSupportConnectT1 = 0.5
		-- var secondSupportConnectT2 = 0.25
		-- var baseCenterX = 0.6*sceneWidth
		var baseWidth = boundedUniform(0.2*sceneWidth, 0.6*sceneWidth)
		-- var mainAngle = radians(boundedUniform(0.0, 45.0))
		-- var supportConnectT = boundedUniform(0.4, 0.6)	-- was (0.25, 0.75)
		-- var shelfHeight = 0.65*sceneHeight
		var shelfLength = boundedUniform(0.2*sceneWidth, 0.7*sceneWidth)
		-- var secondSupportConnectT1 = boundedUniform(0.25, 0.75)
		var secondSupportConnectT2 = boundedUniform(0.05, 0.5)

		-- Make objects
		var mainBeam = BeamT.heapAlloc(shelfHeight/ad.math.cos(mainAngle), beamThickness,
									   [math.pi/2]-mainAngle, Vec2.stackAlloc(baseCenterX - 0.5*baseWidth, groundHeight),
									   beamDepth)
		var supportBeam = BeamT.createConnectingBeamWithEndpoint(mainBeam, supportConnectT,
													 			 Vec2.stackAlloc(baseCenterX + 0.5*baseWidth, groundHeight),
													 			 beamThickness, beamDepth)
		var shelf = BeamT.createConnectingBeamWithLengthVec(mainBeam, 0.95, Vec2.stackAlloc(-shelfLength, 0.0),
											   				beamThickness, beamDepth)
		var secondSupport = BeamT.createBridgingBeam(mainBeam, shelf, secondSupportConnectT1, secondSupportConnectT2,
													 beamThickness, beamDepth)
		scene.objects:push(mainBeam)
		scene.objects:push(supportBeam)
		scene.objects:push(shelf)
		scene.objects:push(secondSupport)

		-- Make connections
		connections:push(Connections.FrictionalContact.heapAlloc(mainBeam, ground, 0, 1))
		connections:push(Connections.FrictionalContact.heapAlloc(supportBeam, ground, 0, 1))
		connections:push(Connections.NailJoint.heapAlloc(supportBeam, mainBeam, 2, 3, nailDiameter, nailLength, numNails))
		connections:push(Connections.NailJoint.heapAlloc(shelf, mainBeam, 2, 3, nailDiameter, nailLength, numNails))
		connections:push(Connections.NailJoint.heapAlloc(secondSupport, mainBeam, 0, 1, nailDiameter, nailLength, numNails))
		connections:push(Connections.NailJoint.heapAlloc(secondSupport, shelf, 2, 3, nailDiameter, nailLength, numNails))

		return scene, connections
	end)

	-- Returns the point and tangent at the interpolant value t
	local terra bezier(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: real)
		var t2 = t*t
		var t3 = t*t2
		var oneMinusT = 1.0 - t
		var oneMinusT2 = oneMinusT*oneMinusT
		var oneMinusT3 = oneMinusT*oneMinusT2
		var point = oneMinusT3*p0 + 3.0*oneMinusT2*t*p1 + 3.0*oneMinusT*t2*p2 + t3*p3
		var tangent = 3.0*oneMinusT2*(p1 - p0) + 6.0*oneMinusT*t*(p2 - p1) + 3.0*t2*(p3 - p2)
		return point, tangent
	end
	Examples.arch = pfn(terra()
		var groundHeight = 0.5
		var sceneWidth = 10.0
		var sceneHeight = 10.0
		var scene = RigidSceneT.stackAlloc(sceneWidth, sceneHeight)
		var connections = [Vector(&Connections.RigidConnection)].stackAlloc()
		var ground = Ground(groundHeight, 0.0, sceneWidth)
		scene.objects:push(ground)

		var sceneXmid = 0.5*sceneWidth

		var numBlocks = 9
		var blockDensity = 2700.0  -- approx. density of granite
		-- var blockWidth = 1.5
		-- var peakHeight = 8.0
		-- var baseWidth = 8.0
		-- var blockWidth = 0.5
		-- var peakHeight = 3.0
		-- var baseWidth = 3.0
		var blockWidth = boundedUniform(0.2, 1.5)
		var peakHeight = boundedUniform(2.0, 10.0)
		var baseWidth = boundedUniform(2.0, 8.0)

		-- Generate blocks
		var baseHalfWidth = 0.5*baseWidth
		var baseXmin = -baseHalfWidth
		var baseXmax = baseHalfWidth
		var blockHalfWidth = 0.5*blockWidth
		var p0 = Vec2.stackAlloc(baseXmin, groundHeight)
		var p1 = Vec2.stackAlloc(baseXmin, peakHeight)
		var p2 = Vec2.stackAlloc(baseXmax, peakHeight)
		var p3 = Vec2.stackAlloc(baseXmax, groundHeight)
		var blocks = [Vector(&BeamT)].stackAlloc()
		for i=0,numBlocks do
			var tlo = double(i)/numBlocks
			var thi = double(i+1)/numBlocks
			var plo, dlo = bezier(p0, p1, p2, p3, tlo)
			var phi, dhi = bezier(p0, p1, p2, p3, thi)
			plo(0) = plo(0) + sceneXmid
			phi(0) = phi(0) + sceneXmid
			var nlo = staticsUtils.perp(dlo); nlo:normalize(); nlo = blockHalfWidth*nlo
			var nhi = staticsUtils.perp(dhi); nhi:normalize(); nhi = blockHalfWidth*nhi
			var block = BeamT.heapAlloc(plo + nlo, plo - nlo, phi + nhi, phi - nhi, blockWidth)
			scene.objects:push(block)
			blocks:push(block)
		end

		-- Ground contacts
		var gc0, gc1 = Connections.FrictionalContact.makeBeamContacts(blocks(0), ground, 0, 1)
		var gc2, gc3 = Connections.FrictionalContact.makeBeamContacts(blocks:back(), ground, 2, 3)
		connections:push(gc0); connections:push(gc1); connections:push(gc2); connections:push(gc3)

		-- Inter-block contacts
		for i=0,blocks.size-1 do
			var bc0, bc1 = Connections.FrictionalContact.makeBeamContacts(blocks(i), blocks(i+1), 2, 3)
			connections:push(bc0); connections:push(bc1)
		end

		m.destruct(blocks)

		return scene, connections
	end)

	return Examples
end




return genExamples





