function [ z ] = discretize2DFunction( fn, x, y )
z = zeros(length(y), length(x));
for i=1:length(y)
    for j=1:length(x)
        z(i,j) = fn(x(j), y(i));
    end
end
end

