function [ ] = saveAutocorrPair( csv1, csv2, outname1, outname2)
% Compute y bounds
M1 = csvread(csv1, 1, 0);   % Skip header
M2 = csvread(csv2, 1, 0);   % Skip header
min1 = min(M1(:,2));
min2 = min(M2(:,2));
max1 = max(M1(:,2));
max2 = max(M2(:,2));
absmin = min(min(min1, min2), -1.0);
absmax = max(max(max1, max2), 1.0);
absmin = min(absmin, -absmax);
absmax = max(absmax, -absmin);

% Draw and save plots

drawAutocorrPlot(csv1, false);
ylim([absmin absmax])
saveas(gcf, outname1);

drawAutocorrPlot(csv2, false);
ylim([absmin absmax])
saveas(gcf, outname2);

close();
end