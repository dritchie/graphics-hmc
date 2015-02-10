require("prob")

local m = require("mem")
local util = require("util")
local inheritance = require("inheritance")
local Vec = require("linalg").Vec
local Vector = require("vector")
local s3dCore = require("s3dCore")
local lpsolve = require("lpsolve")


local Vec3d = Vec(double, 3)


local C = terralib.includecstring [[
#include "stdio.h"
]]


return probmodule(function(pcomp)

local core = s3dCore(pcomp)
util.importAll(core)


----- Adding stuff for exactly stability determination via linear programming
-- (only applies when working with primal (read: double) values)
if real ~= double then return {} end


----- CONNECTIONS

-- Every connection needs to know the indices of its force variables in the LP
Connection.entries:insert({field="firstLPVarID", type=int})

-- Needs to be virtual because Connections may be composed of sub-Connections, which should
--    react differently to this method.
terra Connection:setFirstLPVarID(id: int) : {}
	self.firstLPVarID = id
end
inheritance.virtual(Connection, "setFirstLPVarID")

inheritance.purevirtual(Connection, "numLPVars", {}->{int})
inheritance.purevirtual(Connection, "addStabilityLPConstraints", {&lpsolve._lprec}->{})
inheritance.purevirtual(Connection, "forceCoeffsForBody", {&Body, &Vector(int), &Vector(Vec3d)}->{})
inheritance.purevirtual(Connection, "torqueCoeffsForBody", {&Body, &Vector(int), &Vector(Vec3d)}->{})


----- BODIES

terra Body:addStabilityLPConstraints(lp: &lpsolve._lprec)
	var indices = [Vector(int)].stackAlloc()
	var coeffs = [Vector(Vec3d)].stackAlloc()
	var scalarCoeffs = [Vector(double)].stackAlloc()

	-- Sanity check: this body is involved in at least one connection (otherwise
	--    there's no way we could hope to cancel out the external forces)
	util.assert(self.connections.size > 0,
		"Body cannot possibly be stable if it has zero connections. Did you forget to call Body:addConnection somewhere?\n")

	-- Forces: 3 constraints, one for x, y, z
	var extForce = Vec3.stackAlloc(0.0, 0.0, 0.0)
	for i=0,self.forces.size do
		if self.forces(i).isExternal then
			extForce = extForce + self.forces(i).force
		end
	end
	for i=0,self.connections.size do
		self.connections(i):forceCoeffsForBody(self, &indices, &coeffs)
	end
	var numVarsInvolved = coeffs.size
	scalarCoeffs:resize(numVarsInvolved)
	[(function()
		local stmts = {}
		for i=0,2 do
			table.insert(stmts, quote
				for j=0,coeffs.size do
					scalarCoeffs(j) = coeffs(j)(i)
				end
				lpsolve.add_constraintex(lp, numVarsInvolved,
					&scalarCoeffs(0), &indices(0), lpsolve.EQ, -extForce(i))
			end)
		end
		return stmts
	end)()]

	indices:clear()
	coeffs:clear()
	scalarCoeffs:clear()

	-- Torques: 3 constraints, one for x, y, z
	var com = self:centerOfMass()
	var extTorque = Vec3.stackAlloc(0.0, 0.0, 0.0)
	for i=0,self.forces.size do
		if self.forces(i).isExternal then
			extTorque = extTorque + self.forces(i):torque(com)
		end
	end
	for i=0,self.connections.size do
		self.connections(i):torqueCoeffsForBody(self, &indices, &coeffs)
	end
	numVarsInvolved = coeffs.size
	scalarCoeffs:resize(numVarsInvolved)
	[(function()
		local stmts = {}
		for i=0,2 do
			table.insert(stmts, quote
				for j=0,coeffs.size do
					scalarCoeffs(j) = coeffs(j)(i)
				end
				lpsolve.add_constraintex(lp, numVarsInvolved,
					&scalarCoeffs(0), &indices(0), lpsolve.EQ, -extTorque(i))
			end)
		end
		return stmts
	end)()]

	m.destruct(indices)
	m.destruct(coeffs)
	m.destruct(scalarCoeffs)
end


----- SCENES

terra Scene:isStable()
	-- Clear forces, apply gravity (so we just have external forces present)
	for i=0,self.bodies.size do
		self.bodies(i).forces:clear()
		if self.bodies(i).active then
			self.bodies(i):applyGravityForce(self.gravityConst, self.upVector)
		end
	end

	-- Figure out how many vars we have and set up the base var ids
	--    for each connection
	var numVars = 0
	for i=0,self.connections.size do
		self.connections(i):setFirstLPVarID(numVars+1)	-- lpsolve uses one-based indexing
		numVars = numVars + self.connections(i):numLPVars()
	end

	-- Initialize the LP
	var lp = lpsolve.make_lp(0, numVars)
	lpsolve.set_verbose(lp, 0)

	-- Add constraints imposed by connections (bounds, etc.)
	for i=0,self.connections.size do
		self.connections(i):addStabilityLPConstraints(lp)
	end

	-- Add constraints imposed by bodies (force & torque balance)
	for i=0,self.bodies.size do
		if self.bodies(i).active then
			self.bodies(i):addStabilityLPConstraints(lp)
		end
	end

	-- Solve LP, check for stability
	var retcode = lpsolve.solve(lp)
	-- if retcode == lpsolve.NUMFAILURE then
	-- 	lpsolve.print_lp(lp)
	-- end
	var isstable = (retcode == lpsolve.OPTIMAL)
	-- lpsolve.print_lp(lp)
	lpsolve.delete_lp(lp)
	return isstable
end


return {}
end)




