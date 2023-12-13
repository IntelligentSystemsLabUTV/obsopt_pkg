%% matlab example on wavelet analysis

% load dataset
clear; close all; clc;

if 0
%%% battery dataset
load Lib/various/wavelets/battery_meas.mat; 
else
%%% fancy dataset
% sine wave: y_true = 5*sin(0.5*time) + 10*sin(0.1*time); (1% noise gaussian)
load Lib/various/wavelets/sinewave_meas.mat; 
end

% reframe data
time_long = time;
y_meas_long = y_meas;
y_true_long = y_true;
start = 1;
stop = 600;
time = time_long(start:stop);
y_meas = y_meas_long(start:stop);
% define sampling frequency
Fs = 1; % 1 Hz
Ts = 1/Fs;
wvname = 'amor';

%% plot signal
% plot
figure;
plot(time,y_meas);
grid on
xlabel('t [s]');
ylabel('V [V]');

%% time frequency using STFT
figure;
spectrogram(y_meas,[],[],[],Fs,'yaxis');
title('Spectrogram')

%% time frequency using STFT
figure;
% define sampling frequency
spectrogram(y_meas,64,[],[],Fs,'yaxis');
title('Spectrogram: shorter window')

%% time frequency using wavelets
figure;
cwt(y_meas,wvname,Fs);
title('CWT analysis')

%% richer frequency analysis with wavelets
figure;
Nv = 48;

if 0
% period constraint
t = seconds(Ts);
PLIMITS = [5e0*t 12e1*t];
cwt(y_meas,wvname,t,'VoicesPerOctave',Nv,'PeriodLimits',PLIMITS);
else
% frequency constraint
PLIMITS = [5e0*Ts 12e1*Ts];
FLIMITS = fliplr(1./PLIMITS);
cwt(y_meas,wvname,Fs,'VoicesPerOctave',Nv,'FrequencyLimits',FLIMITS);
end


title('CWT analysis: richer')

%% reconstruct the signal
[WT, F] = cwt(y_meas,Fs);
yrec = icwt(WT, wvname, F, [F(end) F(1)], 'SignalMean', mean(y_meas));
figure;
grid on
subplot(2,1,1)
plot(time,y_meas,time,yrec);
subplot(2,1,2)
heatmap(time,F,real(WT))
grid off
colormap jet