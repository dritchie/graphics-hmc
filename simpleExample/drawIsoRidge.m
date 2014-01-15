function [ ] = drawIsoRidge( fn, xrange, yrange, isoval, softness )
ridgefn = @(x,y) normpdf(fn(x,y), isoval, softness);
drawFunction(ridgefn, xrange, yrange);
end

