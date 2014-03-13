function [ ] = drawFunction( fn, xrange, yrange )
z = discretize2DFunction(fn, xrange, yrange);
pcolor(xrange, yrange, z);
colormap(1-gray);
shading interp;
axis square;
end

