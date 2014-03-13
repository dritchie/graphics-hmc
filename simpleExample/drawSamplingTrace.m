function [ ] = drawSamplingTrace( csvfile, axisRanges )
M = csvread(csvfile);
X = M(:,1);
Y = M(:,2);
T = M(:,3);
scatter(X, Y, 4.0, T, 'fill');
colormap jet;
axis square;
axis(axisRanges);
% colormap(hot);
end

