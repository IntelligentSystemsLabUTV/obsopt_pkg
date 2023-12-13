%% simple script

%mu = 0 - linear system
A = obs.setup.params.eps*[0 1; -1 0];
B = [1; 1];
C = [0 1];
D = 0;
VDP_lin = ss(A,B,C,D);
options = bodeoptions;
options.FreqUnits = 'Hz';
figure()
bode(VDP_lin,options);

%% if you have an obs on which comuting the wavelet
Ts = 0.01;
Nv = 48;
wvname = 'amor';
buffer = squeeze(obs.init.Y_full_story.val(1,:,:));
figure()
cwt(buffer,wvname,1/Ts,'VoicesPerOctave',Nv);

%% wavelet on the signal richness
% Ts = 0.01;
% Nv = 48;
% wvname = 'amor';
% buffer = obs.init.PE_story;
% figure()
% cwt(buffer,wvname,1/Ts,'VoicesPerOctave',Nv);
%% wavelet 
% dpwn sampling instants
WindowTime = obs.setup.time(obs.init.temp_time);
data = reshape(obs.init.target_story(obs.init.traj).val(1,1,obs.init.temp_time),1,length(WindowTime));
figure()
cwt(data,wvname,1/(obs.setup.Nts*Ts),'VoicesPerOctave',Nv);