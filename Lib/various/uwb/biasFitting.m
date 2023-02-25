function y = biasFitting(value)

tmp  = importdata('UWB-bias-last.dat');
gt   = tmp(:,1)/100;
uwb  = tmp(:,2)/100;
diff = gt - uwb;

[xData, yData] = prepareCurveData( gt, diff );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 0.0146215299887771;

% Fit model to data.
[fitresult, ~] = fit( xData, yData, ft, opts );

% % From the OLD experiments
% X = [0,0.4,0.5,0.6,0.7,0.8,0.85,1,1.1,2,2.1,2.9,3,4];
% Y = [0.3,0.3,0.28,0.28,0.25,0.25,0.2,0.2,0.3,0.3,0.25,0.25,0.3,0.3];
% 
% [xData, yData] = prepareCurveData( X, Y );
% 
% % Set up fittype and options.
% ft = fittype( 'smoothingspline' );
% opts = fitoptions( 'Method', 'SmoothingSpline' );
% opts.Normalize = 'on';
% opts.SmoothingParam = 0.998414585216365;
% 
% % Fit model to data.
% [fitresult, ~] = fit( xData, yData, ft, opts );

bias = fitresult(value);
y = bias + value;

% % Plot fit with data.
% figure( 'Name', 'UWB' );
% h = plot( fitresult, xData, yData );
% legend( h, 'Y vs. X', 'UWB', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'X', 'Interpreter', 'none' );
% ylabel( 'Y', 'Interpreter', 'none' );
% grid on;