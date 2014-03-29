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
	}

	terra TriMeshT:__construct() : {}
		self.topology = nil
		self.ownsTopology = false
		m.init(self.nodes)
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
		-- Read node lines (node index, x, y, ...)
		for i=0,numNodes do
			C.fgets(buf, 1023, f)
			var index = C.atoi(C.strtok(buf, " "))
			var node = self.nodes:getPointer(index)
			node(0) = C.atof(C.strtok(nil, " "))
			node(1) = C.atof(C.strtok(nil, " "))
		end
		C.fclose(f)
	end

	terra TriMeshT:__copy(other: &TriMeshT)
		self.topology = other.topology
		self.ownsTopology = other.ownsTopology
		self.nodes = m.copy(other.nodes)
	end

	terra TriMeshT:__destruct()
		m.destruct(self.nodes)
		if self.ownsTopology then m.delete(self.topology) end
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
local terra initGlobalMesh()
	globalMesh = [TriMesh(double)].stackAlloc("meshes/circle.ele", "meshes/circle.node")
end
initGlobalMesh()

local shapeModel = probcomp(function()
	local Vec2 = Vec(real, 2)
	local TriMeshT = TriMesh(real)
	return terra()
		var mesh = TriMeshT.stackAlloc(globalMesh.topology)
		mesh.nodes:resize(globalMesh.nodes.size)
		for i=0,mesh.nodes.size do
			-- mesh.nodes(i) = Vec2(globalMesh.nodes(i))
			mesh.nodes(i)(0) = gaussian(globalMesh.nodes(i)(0), 0.1, {structural=false, initialVal=globalMesh.nodes(i)(0)})
			mesh.nodes(i)(1) = gaussian(globalMesh.nodes(i)(1), 0.1, {structural=false, initialVal=globalMesh.nodes(i)(1)})
		end
		return mesh
	end
end)

-------------------------------------------------------------------------------

local numsamps = 10

local terra run()
	return [forwardSample(shapeModel, numsamps)]
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







