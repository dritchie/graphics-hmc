function [ isoval ] = figureEightFieldFunction( x, y )
x2 = x * x;
y2 = y * y;
y4 = y2 * y2;
isoval = y4 - y2 + x2;
end

