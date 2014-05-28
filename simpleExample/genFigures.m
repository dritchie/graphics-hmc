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
easy_softness = 0.1;
difficult_softness = 0.005;

% Render and save everything
% drawIsoline(@figureEightFieldFunction, xrange, yrange, targetIsoval);
% saveas(gcf, 'isoline.pdf');

drawMergedAutocorrPair('hmc_difficult_autocorr.csv', 'random_difficult_autocorr.csv');
pbaspect([2 1 1]);
saveas(gcf, 'difficult_autocorr.pdf');

drawIsoRidge(@figureEightFieldFunction, xrange, yrange, targetIsoval, easy_softness);
setFigSizeInInches(3,3);
saveas(gcf, 'isoridge_easy.pdf');
drawIsoRidge(@figureEightFieldFunction, xrange, yrange, targetIsoval, difficult_softness);
setFigSizeInInches(3,3);
saveas(gcf, 'isoridge_difficult.pdf');
drawSamplingTrace('random_easy.csv', axisBounds);
setFigSizeInInches(3,3);
saveas(gcf, 'randomSamps_easy.pdf');
drawSamplingTrace('random_difficult.csv', axisBounds);
setFigSizeInInches(3,3);
saveas(gcf, 'randomSamps_difficult.pdf');
drawSamplingTrace('hmc_difficult.csv', axisBounds);
setFigSizeInInches(3,3);
saveas(gcf, 'hmcSamps_difficult.pdf');

close;

% Note: to trim the whitespace on these, use pdfcrop on the command line.
% e.g. find . -exec pdfcrop {} {} \; in a directory containing only the
%    pdfs we want to crop

end

