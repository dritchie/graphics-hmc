terralib.require("prob")

-- Allow us to include stuff from the parent directory
package.path = "../?.t;" .. package.path 

local s3dLib = terralib.require("s3dLib")
local rendering = terralib.require("rendering")

-- TODO: Write stupid simple driver program
