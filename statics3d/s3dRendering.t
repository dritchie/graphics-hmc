terralib.require("prob")

local util = terralib.require("util")
local templatize = terralib.require("templatize")
local inheritance = terralib.require("inheritance")
local colors = terralib
local s3dCore = terralib.require("s3dCore")
local gl = terralib.require("gl")
local Camera = terralib.require("glutils").Camera

gl.exposeContants({
	"GL_POLYGON_OFFSET_FILL"
})


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)

-- Rendering stuff only needs to exist for real == double
if real ~= double then return {} end

local Color3 = Vec3


----- RENDER SETTINGS

-- How we control the rendering of scenes
local RenderPass = uint
local Faces = 0
local Edges = 1
local struct RenderSettings
{
	renderPass: RenderPass,
	renderForces: bool,
	bgColor: Color3,
	faceColor: Color3,	-- TODO: Use basic shading instead
	edgeColor: Color3,
	forceColor: Color3,
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
	self.bgColor = Color3.stackAlloc([colors.White])
	self.faceColor = Color3.stackAlloc([colors.Tableau10.Blue])
	self.edgeColor = Color3.stackAlloc([colors.Black])
	self.forceColor = Color3.stackAlloc([colors.Tableau10.Gray])
	self.edgeWidth = 2.0
	self.forceWidth = 3.0
	self.forceScale = 0.03
end

m.addConstructors(RenderSettings)


----- RENDERING FACES

local coreFace = Face
core.Face = templatize(function(nsides)
	local FaceT = coreFace(nsides)

	terra FaceT:render(settings: &RenderSettings)
		gl.glBegin(gl.mGL_POLYGON())
		[(function()
			local stmts = {}
			for i=1,nsides do
				table.insert(stmts, quote gl.glVertex3d([Vec.elements(`self:vertex([i-1])]) end)
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
		end))()]
	end
	inheritance.virtual(AggregateShapeT, "render")

	return AggregateShapeT
end)
AggregateShape = core.AggregateShape


----- RENDERING FORCES

terra Force:render(settings: &RenderSettings)
	gl.glLineWidth(settings.forceWidth)
	gl.glColor3d([Color3.elements(`settings.forceColor)])
	gl.glBegin(gl.mGL_LINES())
	var bot = self.appPoint
	var top = self.appPoint + settings.forceScale*self.force
	gl.glVertex3d([Vec3.elements(bot)])
	gl.glVertex3d([Vec3.elements(top)])
	gl.glEnd()
end


----- RENDERING BODIES

terra Body:render(settings: &RenderSettings)
	self:shape:render(settings)
end


----- RENDERING SCENES

-- Packages up a scene with info needed to render it
-- (Camera, lights, etc.)
local struct RenderableScene
{
	scene: Scene,
	camera: Camera
}

-- Assumes ownership of scene and camera
terra RenderableScene:__construct(scene: &Scene, camera: &Camera)
	self.scene = @scene
	self.camera = @camera
end

terra RenderableScene:__destruct()
	m.destruct(self.scene)
end

terra RenderableScene:render(settings: &RenderSettings)
	gl.glClearColor([Color3.elements(`settings.bgColor)])
	gl.glClear(gl.mGL_COLOR_BUFFER_BIT() or gl.mGL_DEPTH_BUFFER_BIT())
	gl.glEnable(gl.mGL_DEPTH_TEST())
	self.camera:setupGLPerspectiveView()
	-- TODO: Set up lights and stuff?
	self.scene:render(settings)
end

m.addConstructors(RenderableScene)



terra Scene:render(settings: &RenderSettings)
	settings.renderPass = RenderSettings.Faces
	gl.glColor3d([Color3.elements(`settings.faceColor)])
	gl.glPolygonMode(gl.mGL_FRONT_AND_BACK(), gl.mGL_FILL())
	-- Offset solid face pass so that we can render lines on top
	gl.glEnable(gl.mGL_POLYGON_OFFSET_FILL())
	gl.glPolygonOffset(1.0, 1.0)	-- are these good numbers? maybe use camera zmin/zmax?
	for i=0,self.bodies.size do
		self.bodies(i):render(settings)
	end
	gl.glDisable(gl.mGL_POLYGON_OFFSET_FILL())

	settings.renderPass = RenderSettings.Edges
	gl.glColor3d([Color3.elements(`settings.edgeColor)])
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


----- EXPORTS
return
{
	RenderSettings = RenderSettings,
	RenderableScene = RenderableScene
}

end)







