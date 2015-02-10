local m = require("mem")
local templatize = require("templatize")
local util = require("util")
local Vec = require("linalg").Vec
local Vector = require("vector")
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
		numGroups: uint,
		sizes: Vector(double),
		l_indices: Adjacencies,
		h_indices: Adjacencies
	}
	PatternInfo.Color = Color
	PatternInfo.Adjacencies = Adjacencies

	terra PatternInfo:__construct()
		self.vars = nil
		m.init(self.adjacencies)
		self.templateId = 0
		self.backgroundId = 0
		self.numGroups = 0
		m.init(self.h_indices)
		m.init(self.l_indices)
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

	terra PatternInfo:__construct(numGroups:uint, adj:Adjacencies, bid:uint, tid:uint, sizes:Vector(double), l_indices:Adjacencies, h_indices:Adjacencies)
		--C.printf("%d, %d, %d, %d", numGroups, adj.size(), bid, tid)
		self.backgroundId = bid

		self.adjacencies = m.copy(adj)
		self.sizes = m.copy(sizes)
		self.l_indices = m.copy(l_indices)
		self.h_indices = m.copy(h_indices)
		self.templateId = tid
		self.numGroups = numGroups
		self.vars = [&Color](C.malloc(self.numGroups * sizeof(Color)))
		for x=0,numGroups do
			m.init(@self:getVarPtr(x))
		end
	end

	terra PatternInfo:__destruct()
		--C.printf("Destructing\n")
		C.free(self.vars)
		m.destruct(self.adjacencies)
		m.destruct(self.sizes)
		m.destruct(self.l_indices)
		m.destruct(self.h_indices)
	end

	terra PatternInfo:__copy(other: &PatternInfo)
		self.numGroups = other.numGroups
		self.vars = [&Color](C.malloc(self.numGroups*sizeof(Color)))
		for x=0,self.numGroups do
			@self:getVarPtr(x) = m.copy(@other:getVarPtr(x))
		end
		self.adjacencies = m.copy(other.adjacencies)
		self.templateId = other.templateId
		self.backgroundId = other.backgroundId
		self.sizes = m.copy(other.sizes)
		self.l_indices = m.copy(other.l_indices)
		self.h_indices = m.copy(other.h_indices)
	end

	m.addConstructors(PatternInfo)
	return PatternInfo
end)

return Pattern


