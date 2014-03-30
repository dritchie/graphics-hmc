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

-- Stores the topology of a triangle mesh
local struct TriMeshTopology
{
	elements: Vector(TriMeshElement)
}

terra TriMeshTopology:__construct() : {}
	m.init(self.elements)
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
end

terra TriMeshTopology:__destruct()
	m.destruct(self.elements)
end

m.addConstructors(TriMeshTopology)


-- Associates a list of node positions with
--    a triangle mesh topology
local TriMesh = templatize(function(real)
	local Vec2 = Vec(real, 2)

	local struct TriMeshT
	{
		topology: &TriMeshTopology,
		ownsTopology: bool,
		nodes: Vector(Vec2),
		nodeBoundaryMarkers: Vector(bool)
	}

	terra TriMeshT:__construct() : {}
		self.topology = nil
		self.ownsTopology = false
		m.init(self.nodes)
		m.init(self.nodeBoundaryMarkers)
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

	terra TriMeshT:__copy(other: &TriMeshT)
		self.topology = other.topology
		self.ownsTopology = other.ownsTopology
		self.nodes = m.copy(other.nodes)
		self.nodeBoundaryMarkers = m.copy(other.nodeBoundaryMarkers)
	end

	terra TriMeshT:__destruct()
		m.destruct(self.nodes)
		m.destruct(self.nodeBoundaryMarkers)
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

	local elemColor = colors.Tableau10.Blue
	local edgeColor = colors.Black
	local edgeThickness = `2.0
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
	C.sprintf(str, "lp: %g", lp)
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
local terra initGlobalMesh()
	globalMesh = [TriMesh(double)].stackAlloc("meshes/circle.ele", "meshes/circle.node")
	globalMesh:orientElementsForPositiveArea()
	globalNumBoundaryNodes = globalMesh:numBoundaryNodes()
end
initGlobalMesh()

local shapeModel = probcomp(function()
	local Vec2 = Vec(real, 2)
	local TriMeshT = TriMesh(real)

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
			mesh.nodes(i)(0) = gaussian(globalMesh.nodes(i)(0), 0.2, {structural=false, initialVal=globalMesh.nodes(i)(0)})
			mesh.nodes(i)(1) = gaussian(globalMesh.nodes(i)(1), 0.2, {structural=false, initialVal=globalMesh.nodes(i)(1)})
		end

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
		for i=0,globalNumBoundaryNodes do
			var previ : int = i - 1; if previ < 0 then previ = globalNumBoundaryNodes-1 end
			var nexti = (i + 1) % globalNumBoundaryNodes
			var p0 = mesh.nodes(previ)
			var p1 = mesh.nodes(i)
			var p2 = mesh.nodes(nexti)
			var v1 = p0 - p1; v1:normalize()
			var v2 = p2 - p1; v2:normalize()
			var d = v1:dot(v2)
			var ang = ad.math.acos(d)
			lowerBarrier(ang, [math.pi] - 3.0*[2*math.pi]/globalNumBoundaryNodes, 100.0)
		end

		-- Enforce unit area
		var totalArea = mesh:totalSignedArea()
		factor(softeq(totalArea, [math.pi], 0.01))

		return mesh
	end
end)

-------------------------------------------------------------------------------

local numsamps = 1000

local terra run()
	return [mcmc(shapeModel, HMC({numSteps=100, verbosity=0}), {numsamps=numsamps, verbose=true})]
	-- return [forwardSample(shapeModel, numsamps)]
end
local samples = m.gc(run())
moviename = arg[1] or "movie"
rendering.renderSamples(samples, renderInitFn, renderDrawFn, moviename)

-- local terra test()
-- 	var mesh = [TriMesh(double)].stackAlloc("meshes/circle.ele", "meshes/circle.node")
-- 	C.printf("numElems: %u, numNodes: %u\n", mesh.topology.elements.size, mesh.nodes.size)
-- 	m.destruct(mesh)
-- end
-- test()







