'''

Generates a planar straight line graph for a circle with some specified
number of perimeter vertices.

To turn this into a triangle mesh, use triangle:
http://www.cs.cmu.edu/~quake/triangle.html

For example, try the command:
triangle -pqa0.01 circle.poly

(where circle.poly is the output of this script)

-p tells triangle that you want to triangulate a planar straight line graph

-q says you want a a 'quality' mesh (no interior angle smaller than 20 degrees)

-a0.01 says you want a maximum triangle area of 0.01. Fiddle with this to generate
   denser/coarser meshes.

'''

import sys
import math

def usage():
	print "usage: genCirclePSLG numPerimVerts outFilename"


def main(numPerimVerts, outFilename):
	f = open(outFilename, "w")
	# Write vertex header (num verts, dimension, num attributes, num boundary markers)
	f.write("{0} 2 0 0\n".format(numPerimVerts))
	# Write vertices (vert index, x, y)
	for i in range(numPerimVerts):
		theta = 2*math.pi*(float(i)/numPerimVerts)
		f.write("{0} {1} {2}\n".format(i, math.cos(theta), math.sin(theta)))
	# Write segment header (num segments, num boundary markers)
	f.write("{0} 0\n".format(numPerimVerts))
	# Write segments
	for i in range(numPerimVerts):
		nexti = (i + 1) % numPerimVerts
		f.write("{0} {1} {2}\n".format(i, i, nexti))
	# Write hole header (number of holes)
	f.write("0\n")
	f.close()


if __name__ == "__main__":
	if len(sys.argv) != 3:
		usage()
		exit(1)
	numPerimVerts = int(sys.argv[1])
	outFilename = sys.argv[2]
	main(numPerimVerts, outFilename)