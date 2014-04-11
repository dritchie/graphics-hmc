terralib.require("prob")

local util = terralib.require("util")
local core = terralib.require("s3dCore")
local rendering = terralib.require("s3dRendering")
local materials = terralib.require("s3dMaterials")
local connections = terralib.require("s3dConnections")

return probmodule(function(pcomp)
	return util.joinTables(
		core(pcomp),
		rendering(pcomp),
		materials(pcomp),
		connections(pcomp)
	)
end)