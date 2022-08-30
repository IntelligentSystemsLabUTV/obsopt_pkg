%% generate and save data (alternative to Tushar ECM_parameters.mat)

%%% initial stuff %%%
t0 = 0;
tend = 10000;
Ts = 1;

% define time
Time = t0:Ts:tend;
Niter = length(Time);

%%% define current %%%
Current = zeros(Niter,1);

% square wave
period = 100*Ts;
duty_cycle = 0.01;
amplitude = 6;
for i=1:Niter
    if mod(i,period) < duty_cycle*period
        Current(i) = amplitude;
    else
        Current(i) = -amplitude;
    end
end

%%% SOC %%%
% model assumption
dSOC = 1e-2;
SOC = 0:dSOC:1;
LSOC = length(SOC);

%%% OCV %%%
% model assumption
OCV = 3.5 + SOC + 0.1*sin(100*SOC);

%%% R0 %%%
% model assumption
R0 = 2e-2 + 0*SOC + 0.01*SOC.^2;

%%% R1 %%%
% model assumption
R1 = 5e-3*exp(-SOC);

%%% C1 %%%
% model assumption
C1 = 1e4*exp(-0.1*SOC).*sin(SOC);

%%% save
save('data/ECM_parameters_fedeoli.mat')