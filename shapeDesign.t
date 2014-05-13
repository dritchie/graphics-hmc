terralib.require("prob")

local Vec = terralib.require("linalg").Vec
local util = terralib.require("util")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local ad = terralib.require("ad")
local Vector = terralib.require("vector")
local gl = terralib.require("gl")
local colors = terralib.require("colors")
local rendering = terralib.require("rendering")
local rand = terralib.require("prob.random")

local C = terralib.includecstring [[
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
]]


-- An element in a triangle mesh
local struct TriMeshElement
{
	node0: uint,
	node1: uint,
	node2: uint
}

terra TriMeshElement:otherNode(n0: uint, n1: uint)
	if (self.node0 == n0 and self.node1 == n1) or (self.node0 == n1 and self.node1 == n0) then
		return self.node2
	elseif (self.node0 == n0 and self.node2 == n1) or (self.node0 == n1 and self.node2 == n0) then
		return self.node1
	elseif (self.node1 == n0 and self.node2 == n1) or (self.node1 == n1 and self.node2 == n0) then
		return self.node0
	else
		util.fatalError("TriMeshElement:otherNode - triangle does not contain the two nodes provided\n")
	end
end


-- Stores the topology of a triangle mesh
local struct TriMeshTopology
{
	elements: Vector(TriMeshElement),
	node2tri: Vector(Vector(uint))
}

terra TriMeshTopology:__construct() : {}
	m.init(self.elements)
	m.init(self.node2tri)
end

-- Construct by loading from a file
terra TriMeshTopology:__construct(elefile: rawstring) : {}
	self:__construct()

	-- *** Load elements by reading .ele file *** ---
	var buf : int8[1024]
	var f = C.fopen(elefile, "r")
	-- Read header (num elements, nodes per tri, num attribs)
	C.fgets(buf, 2013, f)
	var numElems = C.atoi(C.strtok(buf, " "))
	self.elements:resize(numElems)
	-- Read element lines (elem index, node0, node1, node2)
	for i=0,numElems do
		C.fgets(buf, 1023, f)
		var index = C.atoi(C.strtok(buf, " "))
		var elem = self.elements:getPointer(index)
		elem.node0 = C.atoi(C.strtok(nil, " "))
		elem.node1 = C.atoi(C.strtok(nil, " "))
		elem.node2 = C.atoi(C.strtok(nil, " "))
	end
	C.fclose(f)

	-- *** Build map of node -> triangle *** --
	for i=0,numElems do
		var elem = self.elements:getPointer(i)
		if self.node2tri.size < elem.node0+1 then self.node2tri:resize(elem.node0+1) end
		self.node2tri(elem.node0):push(i)
		if self.node2tri.size < elem.node1+1 then self.node2tri:resize(elem.node1+1) end
		self.node2tri(elem.node1):push(i)
		if self.node2tri.size < elem.node2+1 then self.node2tri:resize(elem.node2+1) end
		self.node2tri(elem.node2):push(i)
	end
end

terra TriMeshTopology:__destruct()
	m.destruct(self.elements)
	m.destruct(self.node2tri)
end

-- Get all triangles containing the two provided nodes
terra TriMeshTopology:elementsContainingEdge(n0: uint, n1: uint, outlist: &Vector(uint))
	outlist:clear()
	var n0tris = self.node2tri:getPointer(n0)
	for i=0,n0tris.size do
		var elemid = n0tris(i)
		var elem = self.elements:getPointer(elemid)
		if elem.node0 == n1 or elem.node1 == n1 or elem.node2 == n1 then
			outlist:push(elemid)
		end
	end
end

m.addConstructors(TriMeshTopology)


-- Associates a list of node positions with
--    a triangle mesh topology
local TriMesh = templatize(function(real)
	local Vec2 = Vec(real, 2)
	local gaussian_logprob = rand.gaussian_logprob

	local struct TriMeshT
	{
		topology: &TriMeshTopology,
		ownsTopology: bool,
		nodes: Vector(Vec2),
		nodeBoundaryMarkers: Vector(bool),
		symmetries: Vector(real),
		convexCenters: Vector(real),
		convexRanges: Vector(real),
		convexitySigns: Vector(real)
	}

	terra TriMeshT:__construct() : {}
		self.topology = nil
		self.ownsTopology = false
		m.init(self.nodes)
		m.init(self.nodeBoundaryMarkers)
		m.init(self.symmetries)
		m.init(self.convexCenters)
		m.init(self.convexRanges)
		m.init(self.convexitySigns)
	end

	terra TriMeshT:__construct(topo: &TriMeshTopology) : {}
		self:__construct()
		self.topology = topo
	end

	terra TriMeshT:__construct(elefile: rawstring, nodefile: rawstring) : {}
		var topo = TriMeshTopology.heapAlloc(elefile)
		self:__construct(topo)
		self.ownsTopology = true

		-- *** Load node positions from .node file *** --
		var buf : int8[1024]
		var f = C.fopen(nodefile, "r")
		-- Read header (num nodes, dimension, num attribs, num boundary markers)
		C.fgets(buf, 1023, f)
		var numNodes = C.atoi(C.strtok(buf, " "))
		self.nodes:resize(numNodes)
		self.nodeBoundaryMarkers:resize(numNodes)
		-- Read node lines (node index, x, y, boundary marker)
		for i=0,numNodes do
			C.fgets(buf, 1023, f)
			var index = C.atoi(C.strtok(buf, " "))
			var node = self.nodes:getPointer(index)
			node(0) = C.atof(C.strtok(nil, " "))
			node(1) = C.atof(C.strtok(nil, " "))
			self.nodeBoundaryMarkers(i) = bool(C.atoi(C.strtok(nil, " ")))
		end
		C.fclose(f)
	end

	terra TriMeshT:__construct(elefile: rawstring, nodefile: rawstring, numsymmetries: int, numconvex: int, numconcave:int) : {}
		self:__construct(elefile, nodefile)
		self.symmetries:resize(numsymmetries)
		var numcenters = numconvex + numconcave
		self.convexCenters:resize(numcenters)
		self.convexRanges:resize(numcenters)
		self.convexitySigns:resize(numcenters)
		-- for i=0,numcenters do
		-- 	if (i < numconvex) then
		-- 		self.convexitySigns(i) = 1.0
		-- 	else
		-- 		self.convexitySigns(i) = -1.0
		-- 	end
		-- end
	end

	terra TriMeshT:__copy(other: &TriMeshT)
		self.topology = other.topology
		self.ownsTopology = other.ownsTopology
		self.nodes = m.copy(other.nodes)
		self.nodeBoundaryMarkers = m.copy(other.nodeBoundaryMarkers)
		self.symmetries = m.copy(other.symmetries)
		self.convexCenters = m.copy(other.convexCenters)
		self.convexRanges = m.copy(other.convexRanges)
		self.convexitySigns = m.copy(other.convexitySigns)
	end

	terra TriMeshT:__destruct()
		m.destruct(self.nodes)
		m.destruct(self.nodeBoundaryMarkers)
		m.destruct(self.symmetries)
		m.destruct(self.convexCenters)
		m.destruct(self.convexRanges)
		if self.ownsTopology then m.delete(self.topology) end
	end

	terra TriMeshT:unsignedElementArea(i: uint)
		var elem = self.topology.elements:getPointer(i)
		var p0 = self.nodes(elem.node0)
		var p1 = self.nodes(elem.node1)
		var p2 = self.nodes(elem.node2)
		var a = (p1-p0):norm()
		var b = (p2-p1):norm()
		var c = (p0-p2):norm()
		var s = 0.5*(a+b+c)
		return ad.math.sqrt(s*(s-a)*(s-b)*(s-c))
	end

	terra TriMeshT:signedElementArea(i: uint)
		var elem = self.topology.elements:getPointer(i)
		var p1 = self.nodes(elem.node0)
		var p2 = self.nodes(elem.node1)
		var p3 = self.nodes(elem.node2)
		return 0.5*(-p2(0)*p1(1) + p3(0)*p1(1) + p1(0)*p2(1) - p3(0)*p2(1) - p1(0)*p3(1) + p2(0)*p3(1))
	end

	terra TriMeshT:totalSignedArea()
		var a = real(0.0)
		for i=0,self.topology.elements.size do
			a = a + self:signedElementArea(i)
		end
		return a
	end

	-- Flip the winding order of any triangle with negative area
	-- NOTE: alters associated TriMeshTopology object
	terra TriMeshT:orientElementsForPositiveArea()
		for i=0,self.topology.elements.size do
			if self:signedElementArea(i) < 0.0 then
				var elem = self.topology.elements:getPointer(i)
				var tmp = elem.node0
				elem.node0 = elem.node2
				elem.node2 = tmp
				util.assert(self:signedElementArea(i) > 0.0)
			end
		end
	end

	terra TriMeshT:elementInteriorAngles(i: uint)
		var elem = self.topology.elements:getPointer(i)
		var p0 = self.nodes(elem.node0)
		var p1 = self.nodes(elem.node1)
		var p2 = self.nodes(elem.node2)
		var v1 = p1-p0; v1:normalize()
		var v2 = p2-p1; v2:normalize()
		var v3 = p0-p2; v3:normalize()
		var d1 = v2:dot(-v1)
		var d2 = v3:dot(-v2)
		var d3 = v1:dot(-v3)
		return ad.math.acos(d1), ad.math.acos(d2), ad.math.acos(d3)
	end

	terra TriMeshT:numBoundaryNodes()
		var n = 0
		for i=0,self.nodeBoundaryMarkers.size do
			n = n + int(self.nodeBoundaryMarkers(i))
		end
		return n
	end

	-- Compute total signed curvature along a boundary polyline in the mesh
	-- Assumes that the given node indices actually do connect to form a polyline.
	-- Assumes that the given nodes are actually on the boundary
	-- (If it becomes necessary, we could insert checks for both of these things)
	terra TriMeshT:totalSignedCurvature(nodeList: &Vector(uint))
		util.assert(nodeList.size >= 3, "Need at least three nodes to compute total signed curvature\n")
		var trilist = [Vector(uint)].stackAlloc()
		var tsc = real(0.0)
		for i=1,nodeList.size-1 do
			var i0 = nodeList(i-1)
			var i1 = nodeList(i)
			var i2 = nodeList(i+1)
			var p0 = self.nodes(i0)
			var p1 = self.nodes(i1)
			var p2 = self.nodes(i2)
			var v10 = p0 - p1; v10:normalize()
			var v12 = p2 - p1; v12:normalize()
			-- Take dot product to get unsigned angle, then figure out whether we need to
			--    flip the sign.
			var d = v10:dot(v12)
			var ang = [math.pi] - ad.math.acos(d)
			self.topology:elementsContainingEdge(i0, i1, &trilist)
			var otherVert1 = self.nodes(self.topology.elements(trilist(0)):otherNode(i0, i1))
			self.topology:elementsContainingEdge(i1, i2, &trilist)
			var otherVert2 = self.nodes(self.topology.elements(trilist(0)):otherNode(i1, i2))
			var avgOtherVert = 0.5*(otherVert1 + otherVert2)
			var inwardVec = avgOtherVert - p1
			var halfVec = 0.5*(v10 + v12)
			if inwardVec:dot(halfVec) < 0.0 then
				ang = - ang
			end
			tsc = tsc + ang
		end
		m.destruct(trilist)
		return tsc
	end

	--Planck-taper window, centered at 0
	-- e controls the flatness/sharpness, smaller values give sharper corners
	terra TriMeshT:SmoothWindow(x:real, range:real, e:double)
		-- center at 0
		var value = x + range/2.0
		var za = 2.0*e*(1.0/(1+2.0*value/(range-1)) + 1.0/(1-2.0*e+2.0*value/(range-1)))
		var zb = 2.0*e*(1.0/(1-2.0*value/(range-1)) + 1.0/(1-2.0*e-2.0*value/(range-1)))
		if (value >= 0 and value < e*(range-1)) then
			return real(1.0/(ad.math.exp(za) + 1))
		elseif (value > e*(range-1) and value < (1-e)*(range-1)) then
			return real(1.0)
		elseif (value > (1-e)*(range-1) and value <= (range-1)) then
			return real(1.0/(ad.math.exp(zb)+1))
		else
			return real(0.0)
		end
	end

	terra TriMeshT:totalSignedCurvatureWeighted(nodeList: &Vector(uint), center:real, range:real)
		util.assert(nodeList.size >= 3, "Need at least three nodes to compute total signed curvature\n")
		var trilist = [Vector(uint)].stackAlloc()
		var tsc = real(0.0)


		--C.printf("Hello\n")
		for i=0,nodeList.size-1 do
			var i0 = i-1
			if (i == 0) then
				i0 = nodeList(nodeList.size-1)
			else
				i0 = nodeList(i-1)
			end
			var i1 = nodeList(i)
			var i2 = nodeList(i+1)
			var p0 = self.nodes(i0)
			var p1 = self.nodes(i1)
			var p2 = self.nodes(i2)
			var v10 = p0 - p1; v10:normalize()
			var v12 = p2 - p1; v12:normalize()
			-- Take dot product to get unsigned angle, then figure out whether we need to
			--    flip the sign.
			var d = v10:dot(v12)
			var ang = [math.pi] - ad.math.acos(d)
			self.topology:elementsContainingEdge(i0, i1, &trilist)
			var otherVert1 = self.nodes(self.topology.elements(trilist(0)):otherNode(i0, i1))
			self.topology:elementsContainingEdge(i1, i2, &trilist)
			var otherVert2 = self.nodes(self.topology.elements(trilist(0)):otherNode(i1, i2))
			var avgOtherVert = 0.5*(otherVert1 + otherVert2)
			var inwardVec = avgOtherVert - p1
			var halfVec = 0.5*(v10 + v12)
			if inwardVec:dot(halfVec) < 0.0 then
				ang = - ang
			end
			var centerDist = ad.math.fmin(ad.math.fabs(i-center), ad.math.fabs(nodeList.size-i+center))
			var weight = self:SmoothWindow(centerDist, range, 0.3)
			--C.printf("%d, dist %f, weight %f\n", i, ad.val(centerDist), ad.val(weight))
			--ad.math.exp(real(-0.5*centerDist*centerDist/(range*range)))
			--C.printf("weight %f, center %f, range %f, i %d\n", weight, center, range, i)
			tsc = tsc + weight*ang
		end
		m.destruct(trilist)
		--C.printf("curve %f\n", tsc)
		return tsc
	end

	local elemColor = colors.Tableau10.Blue
	local edgeColor = colors.Black
	local edgeThickness = `2.0
	local axisColor = colors.Tableau10.Purple
	local convexColor = colors.Tableau10.Green
	local concaveColor = colors.Tableau10.Red
	terra TriMeshT:draw()
		for i=0,self.topology.elements.size do
			var elem = self.topology.elements:getPointer(i)
			-- Draw triangle
			gl.glColor3d([elemColor])
			gl.glBegin(gl.mGL_TRIANGLES())
			gl.glVertex2d(ad.val(self.nodes(elem.node0)(0)), ad.val(self.nodes(elem.node0)(1)))
			gl.glVertex2d(ad.val(self.nodes(elem.node1)(0)), ad.val(self.nodes(elem.node1)(1)))
			gl.glVertex2d(ad.val(self.nodes(elem.node2)(0)), ad.val(self.nodes(elem.node2)(1)))
			gl.glEnd()
			-- Draw edges
			gl.glColor3d([edgeColor])
			gl.glLineWidth(edgeThickness)
			gl.glBegin(gl.mGL_LINE_LOOP())
			gl.glVertex2d(ad.val(self.nodes(elem.node0)(0)), ad.val(self.nodes(elem.node0)(1)))
			gl.glVertex2d(ad.val(self.nodes(elem.node1)(0)), ad.val(self.nodes(elem.node1)(1)))
			gl.glVertex2d(ad.val(self.nodes(elem.node2)(0)), ad.val(self.nodes(elem.node2)(1)))
			gl.glEnd()
		end

		--draw the convex centers

		
		for i=0,self.convexCenters.size do
			if (self.convexitySigns(i) > 0.0) then
				gl.glColor3d([convexColor])
			else
				gl.glColor3d([concaveColor])
			end
			gl.glPointSize(ad.val(self.convexRanges(i)))
			gl.glBegin(gl.mGL_POINTS())
			var lidx = int(ad.val(ad.math.floor(self.convexCenters(i))))
			var uidx = int(ad.val(ad.math.ceil(self.convexCenters(i)))) 
			var delta = self.convexCenters(i)-lidx

			var pos = self.nodes(lidx)*(1.0-delta) + self.nodes(uidx)*delta

			gl.glVertex2d(ad.val(pos(0)), ad.val(pos(1)))
			gl.glEnd()
		end




		var numBoundaryNodes = self:numBoundaryNodes()
		gl.glShadeModel(gl.mGL_SMOOTH())
		for s=0,self.symmetries.size do
			var axis = Vec2.stackAlloc()
			axis(0) = ad.math.cos(self.symmetries(s))
			axis(1) = ad.math.sin(self.symmetries(s))

			-- Draw axes
			gl.glColor3d([axisColor])
			gl.glLineWidth(edgeThickness)
			gl.glBegin(gl.mGL_LINES())		
			gl.glVertex2d(ad.val(2.0*axis(0)), ad.val(2.0*axis(1)))
			gl.glVertex2d(ad.val(-2.0*axis(0)), ad.val(-2.0*axis(1)))
			gl.glEnd()


			-- Draw the mirrored triangles
			--gl.glColor3d([mirrorColor])
			gl.glLineWidth(edgeThickness)
			--C.printf("boundary %f\n", numBoundaryNodes)
			gl.glBegin(gl.mGL_LINE_LOOP())
				for b=0,numBoundaryNodes do
					--if the node is on the right side of the line
					var side = self.nodes(b)(0)*axis(1)- self.nodes(b)(1)*axis(0)
					--C.printf("side %f\n", side)
					if (side <= 0) then
						var flippedNode = 2*self.nodes(b):dot(axis)/(axis:dot(axis))*axis - self.nodes(b)
						var err = ad.math.fmin(self:symmetrySqErrorIndividual(b,axis,s)*10, 1.0)
						--C.printf("err %f\n", err)

						gl.glColor3f(ad.val(1.0-err), ad.val((1.0-err)*0.5), ad.val((1.0-err)*0.5))
						gl.glVertex2d(ad.val(flippedNode(0)), ad.val(flippedNode(1)))
						--C.printf("%f\t%f\n", flippedNode(0), flippedNode(1))
					end
				end
			gl.glEnd()

		end

		-- gl.glBegin(gl.mGL_LINE_LOOP())
		-- for i=0,numBoundaryNodes do
		-- 	gl.glVertex2d(ad.val(self.nodes(i)(0)), ad.val(self.nodes(i)(1)))
		-- end
		-- gl.glEnd()

	end


	--- Symmetry error, assume the symmetry axes are defined in terms of radians [0, pi]
	--- This has a min in it though...is it differentiable enough?
	terra TriMeshT:symmetrySqErrorIndividual(nodeIdx:int, axis:Vec2, axisIdx:int)
		var flippedNode = 2*self.nodes(nodeIdx):dot(axis)/(axis:dot(axis))*axis - self.nodes(nodeIdx)
		var minSqErr = real([math.huge])

		--may need to compute symmetry correspondences beforehand? Or use softmin
		var numBoundaryNodes = self:numBoundaryNodes()
		-- for m=0,numBoundaryNodes do
		-- 	var v = flippedNode - self.nodes(m)
		-- 	var sqErr = v:dot(v)
		-- 	if (sqErr < minSqErr) then
		-- 		minSqErr = sqErr
		-- 	end
		-- end
		-- return minSqErr


		-- var num = real(0.0)
		-- var denom = real(0.0)
		-- var k = real(100.0)

		-- for m=0,numBoundaryNodes do
		-- 	var v = flippedNode - self.nodes(m)
		-- 	var sqErr = -1.0*v:dot(v)
		-- 	num = num + sqErr*ad.math.exp(k*sqErr)
		-- 	denom = denom + ad.math.exp(k*sqErr)
		-- end
		-- return -1.0*num/denom

		var nodeIdx = self:getReflectedIndex(nodeIdx, axisIdx, numBoundaryNodes)
		var lowerIdx = [int](ad.math.floor(ad.val(nodeIdx)))
		var upperIdx = [int](ad.math.ceil(ad.val(nodeIdx)))
		var delta = nodeIdx - lowerIdx

		if (upperIdx >= numBoundaryNodes) then
			upperIdx = 0
		end


		var err1 = flippedNode - self.nodes(lowerIdx)
		var err2 = flippedNode - self.nodes(upperIdx)
		return (1.0-delta)*err1:dot(err1) + delta*err2:dot(err2)

	end

	terra TriMeshT:getReflectedIndex(nodeIdx:int, axisIdx:int, numBoundaryNodes:int)
		-- return a double indicating the index of the corresponding reflected node
		-- this assumes the nodes are originally arranged in a circle
		var stepSize = 2.0*[math.pi]/numBoundaryNodes
		var cangle = stepSize*nodeIdx
		var axisAngle = self.symmetries(axisIdx)
		var rangle = 2*axisAngle - cangle
		if (rangle < 0) then
			rangle = rangle + 2.0*[math.pi]
		end
		if (rangle >= 2.0*[math.pi]) then
			rangle = rangle - 2.0*[math.pi]
		end

		var rIdx = rangle/stepSize
		return rIdx
	end

	terra TriMeshT:symmetrySqErrorCorrespondence()
		var numBoundaryNodes = self:numBoundaryNodes()
		var totalSqErr = real(0.0)
		
		for i=0,self.symmetries.size do
			--flip each node over the axis, and find the distance to the closest boundary node
			--transform symmetry axis to vector
			var axis = Vec2.stackAlloc()
			axis(0) = ad.math.cos(self.symmetries(i))
			axis(1) = ad.math.sin(self.symmetries(i))

			
			for b=0,numBoundaryNodes do
				--flip
				var flippedNode = 2*self.nodes(b):dot(axis)/(axis:dot(axis))*axis - self.nodes(b)
				var nodeIdx = self:getReflectedIndex(b, i, numBoundaryNodes)
				var lowerIdx = [int](ad.math.floor(ad.val(nodeIdx)))
				var upperIdx = [int](ad.math.ceil(ad.val(nodeIdx)))
				var delta = nodeIdx - lowerIdx

				var err1 = flippedNode - self.nodes(lowerIdx)
				var err2 = flippedNode - self.nodes(upperIdx)
				var increment = (1.0-delta)*err1:dot(err1) + delta*err2:dot(err2)

				totalSqErr = totalSqErr + increment

			end
		end

		return totalSqErr
	end



	terra TriMeshT:symmetrySqError()
		var numBoundaryNodes = self:numBoundaryNodes()
		var totalSqErr = real(0.0)
		var totalOld = real(0.0)
		
		for i=0,self.symmetries.size do
			--flip each node over the axis, and find the distance to the closest boundary node
			--transform symmetry axis to vector
			var axis = Vec2.stackAlloc()
			axis(0) = ad.math.cos(self.symmetries(i))
			axis(1) = ad.math.sin(self.symmetries(i))

			
			for b=0,numBoundaryNodes do
				--flip
				var flippedNode = 2*self.nodes(b):dot(axis)/(axis:dot(axis))*axis - self.nodes(b)
				-- var minSqErr = real([math.huge])

				-- -- --may need to compute symmetry correspondences beforehand? Or use softmin
				--  for m=0,numBoundaryNodes do
				--  	var v = flippedNode - self.nodes(m)
				--  	var sqErr = v:dot(v)
				--  	if (sqErr < minSqErr) then
				--  		minSqErr = sqErr
				-- 	end
				-- end
				-- totalOld = totalOld + minSqErr
				-- totalSqErr = totalSqErr + minSqErr
				var num = real(0.0)
				var denom = real(0.0)
				var k = real(10.0)
				var offset = 300

				for m=0,numBoundaryNodes do
					var v = flippedNode - self.nodes(m)
					var sqErr = real(-1.0)*v:dot(v)
					var exp = ad.math.exp(k*sqErr+offset)
					-- if ((k*sqErr) < -10) then
					-- 	exp = 0.0
					-- end
					num = num + sqErr*exp
					denom = denom + exp
				end
				
				var div = ad.math.log(-1.0*num) - ad.math.log(denom)
				var increment = ad.math.exp(div)
				if (not (increment == increment)) then
					--C.printf("div %f, check %d, num %f\n", div, div==div, num)
					increment = real([-math.huge])
				end

				--C.printf("min %f, softmin %f\n", minSqErr, -1*num/denom)
				--C.printf("num %f, denom %f, num/denom %f\n", num, denom, increment)
				totalSqErr = totalSqErr + increment

			end
		end
		var check1 = (totalOld == totalOld)
		var check2 = (totalSqErr == totalSqErr)
		--C.printf("Total %d\t%d \n", check1, check2)
		return totalSqErr
		--return totalOld
	end



	m.addConstructors(TriMeshT)
	return TriMeshT
end)

-------------------------------------------------------------------------------

gl.exposeConstants({"GL_VIEWPORT", {"GLUT_BITMAP_HELVETICA_18", "void*"}})

local terra displayString(font: &opaque, str: rawstring)
	if str ~= nil and C.strlen(str) > 0 then
		while @str ~= 0 do
			gl.glutBitmapCharacter(font, @str)
			str = str + 1
		end
	end
end

local lpfont = gl.mGLUT_BITMAP_HELVETICA_18()
local lpcolor = colors.Black
local terra displayLogprob(lp: double)
	gl.glMatrixMode(gl.mGL_PROJECTION())
	gl.glLoadIdentity()
	var viewport : int[4]
	gl.glGetIntegerv(gl.mGL_VIEWPORT(), viewport)
	gl.gluOrtho2D(double(viewport[0]), double(viewport[2]), double(viewport[1]), double(viewport[3]))
	gl.glMatrixMode(gl.mGL_MODELVIEW())
	gl.glLoadIdentity()
	var str : int8[64]
	C.sprintf(str, "lp: %d", int(ad.math.ceil(lp)))
	gl.glColor3f([lpcolor])
	gl.glRasterPos2f(viewport[0] + 0.02*viewport[2], viewport[1] + 0.9*viewport[3])
	displayString(lpfont, str)
end

local imageWidth = 500
local function renderInitFn(samples, im)
	return quote
		var argc = 0
		gl.glutInit(&argc, nil)
		gl.glutInitWindowSize(imageWidth, imageWidth)
		gl.glutInitDisplayMode(gl.mGLUT_RGB() or gl.mGLUT_SINGLE())
		gl.glutCreateWindow("Render")
		gl.glViewport(0, 0, imageWidth, imageWidth)
		im:resize(imageWidth, imageWidth)
	end
end

local function renderDrawFn(sample, im)
	return quote
		var mesh = &sample.value
		gl.glClearColor(1.0, 1.0, 1.0, 1.0)
		gl.glClear(gl.mGL_COLOR_BUFFER_BIT())
		gl.glMatrixMode(gl.mGL_PROJECTION())
		gl.glLoadIdentity()
		gl.gluOrtho2D(-1.5, 1.5, -1.5, 1.5)
		gl.glMatrixMode(gl.mGL_MODELVIEW())
		gl.glLoadIdentity()
		mesh:draw()
		displayLogprob(sample.logprob)
		gl.glFlush()
		gl.glReadPixels(0, 0, im.width, im.height,
			gl.mGL_RGB(), gl.mGL_UNSIGNED_BYTE(), im.data)
	end
end

-------------------------------------------------------------------------------

-- Topology and initial node pos is constant, so we store it in a global
local globalMesh = global(TriMesh(double))
local globalNumBoundaryNodes = global(int)
local globalPerimeter = global(double)
local terra initGlobalMesh()
	 globalMesh = [TriMesh(double)].stackAlloc("meshes/hundredCircle.ele", "meshes/hundredCircle.node", 1, 1, 0)
	-- globalMesh = [TriMesh(double)].stackAlloc("meshes/circle.ele", "meshes/circle.node", 1, 1, 0)
	globalMesh:orientElementsForPositiveArea()
	globalNumBoundaryNodes = globalMesh:numBoundaryNodes()

		var perimeter = 0.0
		for i=1,globalNumBoundaryNodes do
			var prev = i-1
			var curr = i
			var v = globalMesh.nodes(prev) - globalMesh.nodes(curr);

			perimeter = perimeter + ad.math.sqrt(v:dot(v))
		end
	globalPerimeter = perimeter
	C.printf("perim %f\n", perimeter)

end
initGlobalMesh()

local shapeModel = probcomp(function()
	local Vec2 = Vec(real, 2)
	local TriMeshT = TriMesh(real)

	local gaussian_logprob = rand.gaussian_logprob

	local radians = macro(function(x)
		return `x*[math.pi]/180.0
	end)

	local degrees = macro(function(x)
		return `180.0*x/[math.pi]
	end)

	local lowerBarrier = macro(function(val, lowerBound, shape)
		return quote
			if val < lowerBound then factor([-math.huge])
			else factor(-1.0/(shape*(val-lowerBound))) end
		end
	end)

	local upperBarrier = macro(function(val, upperBound, shape)
		return quote
			if val > upperBound then factor([-math.huge])
			else factor(1.0/(shape*(val-upperBound))) end
		end
	end)

	return terra()
		var mesh = TriMeshT.stackAlloc(globalMesh.topology)
		mesh.nodes:resize(globalMesh.nodes.size)

		-- Put random perturbation on nodes
		for i=0,mesh.nodes.size do
			-- mesh.nodes(i)(0) = gaussian(globalMesh.nodes(i)(0), 0.2, {structural=false, initialVal=globalMesh.nodes(i)(0)})
			-- mesh.nodes(i)(1) = gaussian(globalMesh.nodes(i)(1), 0.2, {structural=false, initialVal=globalMesh.nodes(i)(1)})
			mesh.nodes(i)(0) = gaussian(globalMesh.nodes(i)(0), 100.0, {structural=false, initialVal=globalMesh.nodes(i)(0)})
			mesh.nodes(i)(1) = gaussian(globalMesh.nodes(i)(1), 100.0, {structural=false, initialVal=globalMesh.nodes(i)(1)})
		end
		mesh.symmetries:resize(globalMesh.symmetries.size)
		for s=0,mesh.symmetries.size do
			-- var param = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0, mass=0.1})
			-- mesh.symmetries(s) = param*[math.pi]
			--var param = uniform(0.0, 1.0, {structural=false, lowerBound=0, upperBound=5.0, mass=0.5})
			var param = gaussian(0.0, 10000, {structural=false, mass=0.5})
			param = param * [math.pi]
			var outside = ad.math.fabs(param/[math.pi])
			--C.printf("param %f, outside %f, ceil %f\n", ad.val(param), ad.val(outside), ad.val(ad.math.ceil(outside)))
			if (param < real(0.0)) then
				param = param + ad.math.ceil(outside)*[math.pi]
			elseif (param > real([math.pi])) then
				param = param - ad.math.ceil(outside)*[math.pi]
			end
			while (param < 0.0) do
				param = param + [math.pi]
			end
			while (param > [math.pi]) do
				param = param - [math.pi]
			end
			--C.printf("clamped param %f\n", ad.val(param))
			mesh.symmetries(s) = param
		end
		mesh.convexCenters:resize(globalMesh.convexCenters.size)
		mesh.convexRanges:resize(globalMesh.convexCenters.size)
		mesh.convexitySigns:resize(globalMesh.convexitySigns.size)
		for i=0,mesh.convexCenters.size do
			mesh.convexCenters(i) = uniform(0.0, globalNumBoundaryNodes, {structural=false, lowerBound=0.0, upperBound=globalNumBoundaryNodes, mass=0.5})
			if (mesh.convexCenters(i) == globalNumBoundaryNodes) then
				mesh.convexCenters(i) = real(0.0)
			end
			mesh.convexRanges(i) = real(globalNumBoundaryNodes/4.0) --10.0--uniform(3, globalNumBoundaryNodes/4, {structural=false, lowerBound=3, upperBound=globalNumBoundaryNodes, mass=0.5})
			mesh.convexitySigns(i) = 2.0*[math.pi]--uniform(-2.0*[math.pi], 2.0*[math.pi], {structural=false, lowerBound=-2.0*[math.pi], upperBound=2.0*[math.pi]})
		end
		mesh.nodeBoundaryMarkers = m.copy(globalMesh.nodeBoundaryMarkers)
		


		-- Prevent elements from reaching zero area
		-- Prevent interior angles from getting too bad
		 for i=0,mesh.topology.elements.size do
			-- var area = mesh:unsignedElementArea(i)
			-- var area = mesh:signedElementArea(i)
			-- lowerBarrier(area, 0.0, 100.0)
			var ang1, ang2, ang3 = mesh:elementInteriorAngles(i)
			lowerBarrier(ang1, radians(20.0), 100.0)
			lowerBarrier(ang2, radians(20.0), 100.0)
			lowerBarrier(ang3, radians(20.0), 100.0)
		end

		-- Enforce smoothness along the boundary
		-- Enforcing a minimum angle of pi - 2pi/numBoundaryNodes would
		--    mean that a circle is the only viable shape.
		-- Allowing bigger angles makes for more shape possibilities
		-- for i=0,globalNumBoundaryNodes do
		-- 	var previ : int = i - 1; if previ < 0 then previ = globalNumBoundaryNodes-1 end
		-- 	var nexti = (i + 1) % globalNumBoundaryNodes
		-- 	var p0 = mesh.nodes(previ)
		-- 	var p1 = mesh.nodes(i)
		-- 	var p2 = mesh.nodes(nexti)
		-- 	var v1 = p0 - p1; v1:normalize()
		-- 	var v2 = p2 - p1; v2:normalize()
		-- 	var d = v1:dot(v2)
		-- 	var ang = ad.math.acos(d)
		-- 	lowerBarrier(ang, [math.pi] - 3.0*[2*math.pi]/globalNumBoundaryNodes, 100.0)
		-- end

		-- Enforce flat angles along boundary
		-- for i=0,globalNumBoundaryNodes do
		-- 	var previ : int = i - 1; if previ < 0 then previ = globalNumBoundaryNodes-1 end
		-- 	var nexti = (i + 1) % globalNumBoundaryNodes
		-- 	var p0 = mesh.nodes(previ)
		-- 	var p1 = mesh.nodes(i)
		-- 	var p2 = mesh.nodes(nexti)
		-- 	var v1 = p0 - p1; v1:normalize()
		-- 	var v2 = p2 - p1; v2:normalize()
		-- 	var d = v1:dot(v2)
		-- 	var ang = ad.math.fabs(d)
		-- 	factor([gaussian_logprob(real)](ang, 1.0, 0.1))
		-- end

		-- penalize nonflat regions (softmax)
		var sfactor = real(5.0)
		var smax = real(0.0)
		var denom = real(0.0)
		for i=0,globalNumBoundaryNodes do
			var previ : int = i - 1; if previ < 0 then previ = globalNumBoundaryNodes-1 end
			var nexti = (i + 1) % globalNumBoundaryNodes
			var p0 = mesh.nodes(previ)
			var p1 = mesh.nodes(i)
			var p2 = mesh.nodes(nexti)
			var v1 = p0 - p1; v1:normalize()
			var v2 = p2 - p1; v2:normalize()
			var d = v1:dot(v2)
			var ang = 1.0 - ad.math.fabs(d)
			var exp = ad.math.exp(ang*sfactor)
			smax = smax + ang * exp
			denom = denom + exp		
		end
		factor(softeq(smax/denom, 0.0, 0.001))

		-- Enforce small boundary circumference
		-- var perimeter = real(0.0)
		-- for i=0,globalNumBoundaryNodes do
		-- 	var prev = i-1
		-- 	if (i == 0) then
		-- 		prev = globalNumBoundaryNodes-1
		-- 	end
		-- 	var curr = i
		-- 	var v = mesh.nodes(prev) - mesh.nodes(curr);

		-- 	perimeter = perimeter + ad.math.sqrt(v:dot(v))
		-- end
		-- factor([gaussian_logprob(real)](perimeter, 2*[math.pi], 0.1))

		-- -- Enforce boundaries to be about the same segment length
		-- for i=1,globalNumBoundaryNodes-1 do
		-- 	var v1 = i-1
		-- 	var v2 = i
		-- 	var v3 = i+1
		-- 	var d1 = mesh.nodes(v1) - mesh.nodes(v2)
		-- 	var d2 = mesh.nodes(v2) - mesh.nodes(v3)
		-- 	var diff = d1:dot(d1) - d2:dot(d2)
		-- 	factor([gaussian_logprob(real)](diff, 0, 0.1))
		-- end

		-- Enforce unit area
		var totalArea = mesh:totalSignedArea()
		-- C.printf("%g                           \n", ad.val(totalArea))
		factor(softeq(totalArea, [math.pi], 0.01))

		-- Encourage high curvature along part of the boundary
		-- var nodelist = [Vector(uint)].stackAlloc(globalNumBoundaryNodes/4)
		-- for i=0,globalNumBoundaryNodes/4 do
		-- 	nodelist(i) = i
		-- end
		-- var tsc = mesh:totalSignedCurvature(&nodelist)
		-- factor(softeq(tsc, [2*math.pi], [math.pi/16]))
		-- --C.printf("init\n")
		-- var center = real(5.0)--uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})*globalNumBoundaryNodes
		-- if (center == globalNumBoundaryNodes) then
		-- 	center = 0.0
		-- end
		-- var range = uniform(0.0, 1.0, {structural=false, lowerBound=0.0, upperBound=1.0})*(globalNumBoundaryNodes)
		
		var nodelist = [Vector(uint)].stackAlloc(globalNumBoundaryNodes)
		for i=0,globalNumBoundaryNodes do
			nodelist(i) = i
		end
		for i=0,mesh.convexCenters.size do
			var coeff = mesh.convexitySigns(i)
			var tsc = mesh:totalSignedCurvatureWeighted(&nodelist, mesh.convexCenters(i), mesh.convexRanges(i))
			factor(softeq(tsc, coeff, [math.pi]/160))
		end
		nodelist:__destruct()

		-- Encourage symmetry
		-- var symmetrySqError = mesh:symmetrySqError()
		-- var symmetrySqError = mesh:symmetrySqErrorCorrespondence()
		-- factor(softeq(symmetrySqError, 0.0, 0.5))
		-- factor(softeq(symmetrySqError, 0.0, 0.1))

		-- Encourage symmetry axes that are spaced far enough apart
		-- for i=0,mesh.symmetries.size do
		-- 	for j=i+1,mesh.symmetries.size do
		-- 		var dist = ad.math.fmin(ad.math.fabs(mesh.symmetries(i) - mesh.symmetries(j)), ad.math.fabs([math.pi]-ad.math.fabs(mesh.symmetries(i)+mesh.symmetries(j))))
		-- 		lowerBarrier(dist, [math.pi]/(2*mesh.symmetries.size), 100.0)
		-- 	end
		-- end

		--encourage symmetry around pi/2
		--factor(softeq(mesh.symmetries(0), [math.pi]/2.0, 0.01))

		return mesh
	end
end)

-------------------------------------------------------------------------------

local numsamps = 1000--1000

local terra run()
	 return [mcmc(shapeModel, HMC({numSteps=1000, lag=1, verbosity=0}), {numsamps=numsamps, verbose=true})]()
	--return [mcmc(shapeModel, RandomWalk(), {numsamps=100*numsamps, verbose=verbose})]()
	-- return [forwardSample(shapeModel, 1)]--numsamps)]()
end
local samples = m.gc(run())
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename)

local terra printSamples()

	var nodelist = [Vector(uint)].stackAlloc(globalNumBoundaryNodes)
	for i=0,globalNumBoundaryNodes do
		nodelist(i) = i
	end
	for i=0, samples.size do
		if (i%(numsamps/10) == 0) then
			var axis = samples(i).value.symmetries(0)
			var perimeter = real(0.0)
			var mesh = samples(i).value
			for p=0,globalNumBoundaryNodes do
				var prev = p-1
				if (p == 0) then
					prev = globalNumBoundaryNodes-1
				end
				var curr = p
				var v = mesh.nodes(prev) - mesh.nodes(curr);

				perimeter = perimeter + ad.math.sqrt(v:dot(v))
			end

			var sfactor = real(5.0)
			var smax = real(0.0)
			var denom = real(0.0)
			for i=0,globalNumBoundaryNodes do
				var previ : int = i - 1; if previ < 0 then previ = globalNumBoundaryNodes-1 end
				var nexti = (i + 1) % globalNumBoundaryNodes
				var p0 = mesh.nodes(previ)
				var p1 = mesh.nodes(i)
				var p2 = mesh.nodes(nexti)
				var v1 = p0 - p1; v1:normalize()
				var v2 = p2 - p1; v2:normalize()
				var d = v1:dot(v2)
				var ang = 1.0 - ad.math.fabs(d)
				var exp = ad.math.exp(ang*sfactor)
				smax = smax + ang * exp
				denom = denom + exp		
			end

			var angleSoftMax = smax/denom

			var area = mesh:totalSignedArea()
			C.printf("axis: %f, perim: %f, area: %f, angles %f\n",axis, perimeter, area, angleSoftMax)
			for a=0,samples(i).value.convexRanges.size do
				var mesh = samples(i).value
				var tsc = mesh:totalSignedCurvatureWeighted(&nodelist, mesh.convexCenters(a), mesh.convexRanges(a))
					
				C.printf("\tsign %d, center: %f, range: %f, tsc %f, target %f\n", samples(i).value.convexitySigns(a), ad.val(samples(i).value.convexCenters(a)), ad.val(samples(i).value.convexRanges(a)), ad.val(tsc), ad.val(mesh.convexitySigns(a)))
			end
		end
	end
end
printSamples()

-- local terra test()
-- 	var mesh = [TriMesh(double)].stackAlloc("meshes/circle.ele", "meshes/circle.node")
-- 	C.printf("numElems: %u, numNodes: %u\n", mesh.topology.elements.size, mesh.nodes.size)
-- 	m.destruct(mesh)
-- end
-- test()







