-- Include quicksand
terralib.require("prob")

-- Other libraries we'll need
local Vector = terralib.require("vector")
local image = terralib.require("image")
local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local ad = terralib.require("ad")
local terrain = terralib.require("terrain")
local U = terralib.require("terrainUtils")
local constraints = terralib.require("terrainConstraints")

-- C standard library stuff
local C = terralib.includecstring [[
#include <stdio.h>
]]

-- Terrain Model
local function terrainModel()

  -- Params
	local temp = 0.001
	local rigidity = 0.2
	local tension = 0.5
	local c = 1000000.05
  local width = 100
  local height = 100

  -- General types and initializations
  local RealGrid = image.Image(real, 1)
  local DoubleGrid = image.Image(double, 1)
  local tgtImage = DoubleGrid.methods.load(image.Format.PNG, "targets/heightmap_100_100.png")
  local TerrainMapT = terrain.TerrainMap(real)
  -- local terrainMap = `TerrainMapT.stackAlloc(width, height)

  -- Height constraints
  local HeightConstraintT = constraints.HeightConstraint(real)
  local DeltaHeightConstraintT = constraints.DeltaHeightConstraint(real)
  local hE1 = HeightConstraintT(0.0, 0.2, 0, 15, 0, 15)
  local hE2 = HeightConstraintT(0.8, 1.0, 20, 30, 20, 30)
  local dHE = DeltaHeightConstraintT(0.2, 10, 90, 10, 90)

  -- Uniform unconstrained lattice
  local terra setUniformLattice(lattice: &RealGrid)
    var scale = 0.01
    --C.printf("%d, %d", lattice.width, lattice.height)
    for y = 0, lattice.height do
      for x = 0, lattice.width do
        var pix = gaussian(0.0, scale, {structural=false})
        -- var pix = uniform(-scale, scale, {structural=false, hasPrior=false})
        lattice(x,y)(0) = U.logistic(pix/scale)
        -- lattice(x,y)(0) = uniform(0.0, 1.0, {structural=false, hasPrior=false})
        -- lattice(x,y)(0) = gaussian(0.5, 0.25, {structural=false, mass=1.0})
        -- bound(lattice(x,y)(0), 0.0, 1.0, 0.2)
      end
    end
  end

  -- Variational spline smoothness constraint
  local terra splineConstraint(lattice: &RealGrid)
    var h = 1.0 / ad.math.fmax(width, height)
    -- var totalEnergy = 0.0
    for y = 1, height-1 do
      for x = 1, width-1 do
        var dx = U.Dx(lattice, x, y, h)
        var dy = U.Dy(lattice, x, y, h)
        var dxx = U.Dxx(lattice, x, y, h)
        var dyy = U.Dyy(lattice, x, y, h)
        var dxy = U.Dxy(lattice, x, y, h)
        var e_p = 0.5 * rigidity *
          ((1.0-tension)*(dx*dx + dy*dy) + tension*(dxx*dxx + dyy*dyy + dxy*dxy))
        --var diff = lattice(x,y):distSq(tgtImage(x, y))
        --var e_d = 0.5 * c * diff
        var energy = e_p(0) --+ e_d
        factor(-energy / temp)
        -- var totalEnergy = totalEnergy + energy
      end
    end
    -- factor(-0.5 * c * totalEnergy/(temp*(width-1)*(height-1)))
  end

  -- Compose full terrain model
	return terra()
    var terrainMap = TerrainMapT.stackAlloc(width, height)
    var lattice = terrainMap.grid

    -- Set lattice to uniform priors
    setUniformLattice(&lattice)

    -- Variational spline derivative constraints
    splineConstraint(&lattice)

    -- Region constraints
    var he1 = hE1(&lattice)
    var he2 = hE2(&lattice)
    factor(-0.5 * c * he1 / temp)
    factor(-0.5 * c * he2 / temp)
    factor(-0.5 * c * dHE(&lattice, &tgtImage) / temp)

    return lattice
  end
end

-- Do HMC inference on the model
-- (Switch to RandomWalk to see random walk metropolis instead)
local numsamps = 1000
local verbose = true
local kernel = HMC({numSteps=1})
local terra doInference()
	return [mcmc(terrainModel, kernel, {numsamps=numsamps, verbose=verbose})]
end
local samples = m.gc(doInference())

-- Render movie
local moviename = arg[1] or "movie"
U.renderSamplesToMovie(samples, moviename)