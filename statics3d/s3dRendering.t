terralib.require("prob")

local m = terralib.require("mem")
local util = terralib.require("util")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")
local colors = terralib.require("colors")
local s3dCore = terralib.require("s3dCore")
local gl = terralib.require("gl")

local glutils = terralib.require("glutils")
util.importEntries(glutils, "Camera", "Light", "Material")

gl.exposeConstants({
	"GL_POLYGON_OFFSET_FILL"
})


local Color4d = Vec(double, 4)


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)


----- RENDER SETTINGS

-- How we control the rendering of scenes
local RenderPass = uint
local Faces = 0
local Edges = 1
local struct RenderSettings
{
	renderPass: RenderPass,
	renderForces: bool,
	bgColor: Color4d,
	activeBodyColor: Color4d,
	passiveBodyColor: Color4d,
	edgeColor: Color4d,
	forceColor: Color4d,
	edgeWidth: real,
	forceWidth: real,
	forceScale: real
}
RenderSettings.RenderPass = RenderPass
RenderSettings.Faces = Faces
RenderSettings.Edges = Edges

terra RenderSettings:__construct()
	self.renderPass = Faces
	self.renderForces = false
	self.bgColor = Color4d.stackAlloc([colors.White], 1.0)
	self.activeBodyColor = Color4d.stackAlloc([colors.Tableau10.Blue], 1.0)
	self.passiveBodyColor = Color4d.stackAlloc([colors.Tableau10.Brown], 1.0)
	self.edgeColor = Color4d.stackAlloc([colors.Black], 1.0)
	self.forceColor = Color4d.stackAlloc([colors.Tableau10.Gray], 1.0)
	self.edgeWidth = 2.0
	self.forceWidth = 3.0
	self.forceScale = 0.03
end

m.addConstructors(RenderSettings)


----- RENDERING FACES

local coreFace = Face
core.Face = templatize(function(nverts)
	local FaceT = coreFace(nverts)

	terra FaceT:render(settings: &RenderSettings)
		var n = self:normal()
		-- if n >= 0.0 then
		-- 	gl.glColor3d(0.0, 0.0, 1.0)
		-- else
		-- 	gl.glColor3d(1.0, 0.0, 0.0)
		-- end
		-- gl.glColor3d([Vec3.elements(`n:abs())])
		gl.glBegin(gl.mGL_POLYGON())
		[(function()
			local stmts = {}
			for i=1,nverts do
				table.insert(stmts, quote gl.glNormal3d([Vec3.elements(n)]) end)
				table.insert(stmts, quote gl.glVertex3d([Vec3.elements(`self:vertex([i-1]))]) end)
			end
			return stmts
		end)()]
		gl.glEnd()
	end

	return FaceT
end)
Face = core.Face


----- RENDERING SHAPES

inheritance.purevirtual(Shape, "render", {&RenderSettings}->{})

local coreAggregateShape = AggregateShape
core.AggregateShape = templatize(function(numParts)
	local AggregateShapeT = coreAggregateShape(numParts)

	terra AggregateShapeT:render(settings: &RenderSettings) : {}
		[(function()
			local stmts = {}
			for i=0,numParts do
				table.insert(stmts, quote self.shapes[ [i-1] ]:render(settings) end)
			end
			return stmts
		end)()]
	end
	inheritance.virtual(AggregateShapeT, "render")

	return AggregateShapeT
end)
AggregateShape = core.AggregateShape


----- RENDERING FORCES

terra Force:render(settings: &RenderSettings)
	gl.glLineWidth(settings.forceWidth)
	gl.glColor4d([Color4d.elements(`settings.forceColor)])
	gl.glBegin(gl.mGL_LINES())
	var bot = self.appPoint
	var top = self.appPoint + settings.forceScale*self.force
	gl.glVertex3d([Vec3.elements(bot)])
	gl.glVertex3d([Vec3.elements(top)])
	gl.glEnd()
end


----- RENDERING BODIES

terra Body:render(settings: &RenderSettings)
	var mycolor: Color4d
	if self.active then
		mycolor = settings.activeBodyColor
	else
		mycolor = settings.passiveBodyColor
	end
	var material = Material.stackAlloc(mycolor, Color4d.stackAlloc(0.0, 0.0, 0.0, 1.0), 0.0)
	material:setupGLMaterial()
	self.shape:render(settings)
end


----- RENDERING SCENES

terra Scene:render(settings: &RenderSettings)

	settings.renderPass = [RenderSettings.Faces]
	gl.glEnable(gl.mGL_LIGHTING())
	gl.glShadeModel(gl.mGL_FLAT())
	gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_FILL())
	-- Offset solid face pass so that we can render lines on top
	gl.glEnable(gl.mGL_POLYGON_OFFSET_FILL())
	gl.glPolygonOffset(1.0, 1.0)	-- are these good numbers? maybe use camera zmin/zmax?
	for i=0,self.bodies.size do
		self.bodies(i):render(settings)
	end
	gl.glDisable(gl.mGL_POLYGON_OFFSET_FILL())
	gl.glDisable(gl.mGL_LIGHTING())

	settings.renderPass = [RenderSettings.Edges]
	gl.glColor4d([Color4d.elements(`settings.edgeColor)])
	gl.glLineWidth(settings.edgeWidth)
	gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_LINE())
	for i=0,self.bodies.size do
		self.bodies(i):render(settings)
	end

	if settings.renderForces then
		for i=0,self.bodies.size do
			var b = self.bodies(i)
			for j=0,b.forces.size do
				b.forces(j):render(settings)
			end
		end
	end
end


-- Packages up a scene with info needed to render it
-- (Camera, lights, etc.)
local struct RenderableScene
{
	scene: Scene,
	camera: Camera,
	lights: Vector(Light)
}

-- Assumes ownership of scene and camera
terra RenderableScene:__construct(scene: Scene, camera: Camera)
	self.scene = scene
	self.camera = camera
	m.init(self.lights)
end

terra RenderableScene:__destruct()
	m.destruct(self.scene)
	m.destruct(self.lights)
end

terra RenderableScene:addLight(light: Light)
	self.lights:push(light)
end

terra RenderableScene:render(settings: &RenderSettings)
	gl.glClearColor([Color4d.elements(`settings.bgColor)])
	gl.glClear(gl.mGL_COLOR_BUFFER_BIT() or gl.mGL_DEPTH_BUFFER_BIT())
	gl.glEnable(gl.mGL_DEPTH_TEST())
	gl.glEnable(gl.mGL_CULL_FACE())
	gl.glEnable(gl.mGL_NORMALIZE())

	self.camera:setupGLPerspectiveView()

	util.assert(self.lights.size < gl.mGL_MAX_LIGHTS(),
		"Too many lights; max is %d, got %d\n", self.lights.size, gl.mGL_MAX_LIGHTS())
	for i=0,self.lights.size do
		self.lights(i):setupGLLight(i)
	end

	self.scene:render(settings)
end

m.addConstructors(RenderableScene)



----- EXPORTS
return
{
	RenderSettings = RenderSettings,
	RenderableScene = RenderableScene
}

end)







