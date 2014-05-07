function [ ] = saveAutocorrForRun(dirname)
csv1 = sprintf('%s/hmc_autocorr.csv', dirname);
csv2 = sprintf('%s/svmh_autocorr.csv', dirname);
outname1 = sprintf('%s/hmc_autocorr.pdf', dirname);
outname2 = sprintf('%s/svmh_autocorr.pdf', dirname);
saveAutocorrPair(csv1, csv2, outname1, outname2);
end