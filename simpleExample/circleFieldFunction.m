function [ isoval ] = circleFieldFunction( x, y )
r = 1.0;
c = [0.0, 0.0];
p = [x, y];
dist = norm(p - c);
isoval = dist - r;
end

