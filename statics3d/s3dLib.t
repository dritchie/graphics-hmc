require("prob")

local util = require("util")
local core = require("s3dCore")
local rendering = require("s3dRendering")
local lp = require("s3dLP")
local materials = require("s3dMaterials")
local connections = require("s3dConnections")
local shapes = require("s3dShapes")

return probmodule(function(pcomp)
	return util.joinTables(
		core(pcomp),
		rendering(pcomp),
		lp(pcomp),
		materials(pcomp),
		connections(pcomp),
		shapes(pcomp)
	)
end)
