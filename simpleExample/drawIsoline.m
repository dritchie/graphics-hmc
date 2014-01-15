function [ ] = drawIsoline( fn, xrange, yrange, isoval )
z = discretize2DFunction(fn, xrange, yrange);
contour(xrange, yrange, z, [isoval, isoval], 'Color', 'k');
axis square;
end

