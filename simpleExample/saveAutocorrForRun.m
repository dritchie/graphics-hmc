function [ ] = saveAutocorrForRun(dirname)
csv1 = sprintf('%s/hmc_autocorr.csv', dirname);
csv2 = sprintf('%s/svmh_autocorr.csv', dirname);
% outname1 = sprintf('%s/hmc_autocorr.pdf', dirname);
% outname2 = sprintf('%s/svmh_autocorr.pdf', dirname);
% saveAutocorrPair(csv1, csv2, outname1, outname2);
drawMergedAutocorrPair(csv1, csv2);
legend('HMC', 'SVMH', 'Location', 'SouthEast');
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [3, 3]);
set(gcf, 'PaperPosition', [0, 0, 3, 3]);
outname = sprintf('%s/autocorr.pdf', dirname);
saveas(gcf, outname);
close();
end