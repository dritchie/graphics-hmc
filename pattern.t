local m = terralib.require("mem")
local templatize = terralib.require("templatize")
local util = terralib.require("util")
local Vec = terralib.require("linalg").Vec
local Vector = terralib.require("vector")
local C = terralib.includecstring [[
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
]]

-- numGroups: number of color groups
-- adj: vector of adjacencies
local Pattern = templatize(function(real)

	local Color = Vec(real, 3) -- color vars
	local Adjacencies = Vector(Vector(int))--list of adjacent indices, assume no duplicates

	local struct PatternInfo
	{
		vars: &Color,
		backgroundId: uint,
		adjacencies: Adjacencies,
		templateId: uint,
		numGroups: uint
	}
	PatternInfo.Color = Color
	PatternInfo.Adjacencies = Adjacencies

	terra PatternInfo:__construct()
		self.vars = nil
		self.adjacencies:__construct()
		self.templateId = 0
		self.backgroundId = 0
		self.numGroups = 0
	end

	terra PatternInfo:getVarPtr(x: uint)
		return self.vars + x
	end
	util.inline(PatternInfo.methods.getVarPtr)

	terra PatternInfo:getVarValue(x: uint)
		return m.copy(@self.getVarPtr(x))
	end
	util.inline(PatternInfo.methods.getVarValue)

	PatternInfo.metamethods.__apply = macro(function(self, x)
		return `@(self.vars + x)
	end)

	terra PatternInfo:setVar(x: uint, color: &Color)
		var pix = self:getVarPtr(x)
		[Color.entryExpList(pix)] = [Color.entryExpList(color)]
	end
	util.inline(PatternInfo.methods.setVar)

	terra PatternInfo:__construct(numGroups:uint, adj:Adjacencies, bid:uint, tid:uint)
		--C.printf("%d, %d, %d, %d", numGroups, adj.size(), bid, tid)
		self.backgroundId = bid
		
		self.adjacencies:__construct()
		self.adjacencies:__copy(&adj)

		self.templateId = tid
		self.numGroups = numGroups
		self.vars = [&Color](C.malloc(self.numGroups * sizeof(Color)))
		for x=0,numGroups do
			self:getVarPtr(x):__construct()
		end
	end

	terra PatternInfo:__destruct()
		C.free(self.vars)
		self.adjacencies:__destruct()
	end

	terra PatternInfo:__copy(other: &PatternInfo)
		self.numGroups = other.numGroups
		self.vars = [&Color](C.malloc(self.numGroups*sizeof(Color)))
		for x=0,self.numGroups do
			@self:getVarPtr(x) = m.copy(@other:getVarPtr(x))
		end
		self.adjacencies:__copy(&other.adjacencies)
		self.templateId = other.templateId
		self.backgroundId = other.backgroundId
	end

	m.addConstructors(PatternInfo)
	return PatternInfo
end)

return Pattern