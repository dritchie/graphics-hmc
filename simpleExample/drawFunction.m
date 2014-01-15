function [ ] = drawFunction( fn, xrange, yrange )
z = discretize2DFunction(fn, xrange, yrange);
pcolor(xrange, yrange, z);
% contourf(xrange, yrange, z, 0:0.1:1);
colormap(1-gray);
shading interp;
axis square;
% axis([xrange(1), xrange(end), yrange(1), yrange(end)]);
end

