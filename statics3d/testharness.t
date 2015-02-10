
local util = require("util")
local Params = require("testparams")




local configname = arg[1] or "config.txt"
local params = Params.new():loadFile(configname)
local outdir = params.outputdir
local logfilename = outdir .. "/log.txt"
print("Runnning...")
local logcontents = util.wait("terra -g test.t " .. configname)
local logfile = io.open(logfilename, "w")
if not logfile then
	print("")
	print(logcontents)
	error("testharness: couldn't find logfile; test run probably died. Output printed above.")
end
logfile:write(logcontents)
logfile:close()
print("done")
