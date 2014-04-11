terralib.require("prob")

local util = terralib.require("util")
local s3dCore = terralib.require("s3dCore")


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)


----- MATERIALS

-- Convenience methods for defining bodies with typical material properties
local oakDensity = `700.0	-- kg/m^3
local oakFriction = math.sqrt(0.54)
Body.methods.oak = terra(shape: &Shape)
	return Body.heapAlloc(shape, oakDensity, oakFriction)
end

return {}
end)