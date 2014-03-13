function [ ] = genFigures( )

% Constants
xmin = -1.15;
xmax = 1.15;
ymin = -1.15;
ymax = 1.15;
xstep = 0.005;
ystep = 0.005;
xrange = xmin:xstep:xmax;
yrange = ymin:ystep:ymax;
axisBounds = [xmin, xmax, ymin, ymax];
targetIsoval = 0.25;
softness = 0.005;

% Render and save everything
drawIsoline(@figureEightFieldFunction, xrange, yrange, targetIsoval);
saveas(gcf, 'isoline.pdf');
drawIsoRidge(@figureEightFieldFunction, xrange, yrange, targetIsoval, softness);
saveas(gcf, 'isoridge.pdf');
drawSamplingTrace('random.csv', axisBounds);
saveas(gcf, 'randomSamps.pdf');
drawSamplingTrace('hmc.csv', axisBounds);
saveas(gcf, 'hmcSamps.pdf');
close;

end

