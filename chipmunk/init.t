
local sourcefile = debug.getinfo(1, "S").source:gsub("@", "")
local header = sourcefile:gsub("init.t", "include/chipmunk/chipmunk.h")

local cp = terralib.includecstring(string.format([[
#define static
#include "%s"
]], header))

local dylib = sourcefile:gsub("init.t", "libchipmunk.6.2.1.dylib")
terralib.linklibrary(dylib)

-- function string.starts(String,Start)
--    return string.sub(String,1,string.len(Start))==Start
-- end
-- for k,v in pairs(cp) do
-- 	if k:starts("cp") then
-- 		print(k, v)
-- 	end
-- end

-- terra test()
-- 	-- Do we have access to the typical vector stuff?
-- 	var v = cp.cpv(0.0, 10.0)
-- 	return v
-- end
-- print(test())

return cp