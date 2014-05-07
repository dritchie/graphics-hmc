
local util = terralib.require("util")
local Params = terralib.require("testparams")




local configname = arg[1] or "config.txt"
local params = Params.new():loadFile(configname)
local outdir = params.outputdir
local logfilename = outdir .. "/log.txt"
print("Runnning...")
local logcontents = util.wait("terra -g test.t " .. configname)
local logfile = io.open(logfilename, "w")
logfile:write(logcontents)
logfile:close()
print("done")