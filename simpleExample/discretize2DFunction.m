function [ z ] = discretize2DFunction( fn, x, y )
z = zeros(length(x), length(y));
for i=1:length(x)
    for j=1:length(y)
        z(i,j) = fn(x(i), y(j));
    end
end
end

