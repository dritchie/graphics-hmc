function [ ] = drawMergedAutocorrPair( csv1, csv2 )
% Compute y bounds
M1 = csvread(csv1, 1, 0);   % Skip header
M2 = csvread(csv2, 1, 0);   % Skip header
min1 = min(M1(:,2)); min1 = min1 - 0.05*abs(min1);
min2 = min(M2(:,2)); min2 = min2 - 0.05*abs(min2);
max1 = max(M1(:,2)); max1 = max1 + 0.05*abs(max1);
max2 = max(M2(:,2)); max2 = max2 + 0.05*abs(max2);
absmin = min(min(min1, min2), -1.0);
absmax = max(max(max1, max2), 1.0);
absmin = min(absmin, -absmax);
absmax = max(absmax, -absmin);

% Draw

Lag1 = M1(:,1);
Lag2 = M2(:,1);
AC1 = M1(:,2);
AC2 = M2(:,2);
plot(Lag1, AC1, Lag2, AC2, 'LineWidth',3);
ylim([absmin absmax]);
pbaspect([1 1 1]);
% pbaspect([2 1 1]);
xlabel('Lag');
ylabel('Autocorrelation');
legend('HMC', 'MH', 'Location', 'SouthWest');
end