terralib.require("prob")

local util = terralib.require("util")
local core = terralib.require("s3dCore")
local rendering = terralib.require("s3dRendering")

return probmodule(function(pcomp)
	return util.joinTables(
		core(pcomp),
		rendering(pcomp)
	)
end)