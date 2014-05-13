function [ ] = drawAutocorrPlot( csvfile, setUnitYLim)
M = csvread(csvfile, 1, 0);   % Skip header
Lag = M(:,1);
Autocorr = M(:,2);
plot(Lag,Autocorr);
pbaspect([2 1 1]);
xlabel('Lag');
ylabel('Autocorrelation');
if setUnitYLim
    ylim([-1 1]);
end
end