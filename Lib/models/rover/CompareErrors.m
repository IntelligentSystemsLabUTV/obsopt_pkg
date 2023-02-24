%%
clear
clc
close all


% save first data
load('results/simulations/rover/vinesOBS/test_Mtraj/1traj_seed01_yesbias_ekf','obs')
obs_ekf = obs;

% save second data
load('results/simulations/rover/vinesOBS/test_Mtraj/1traj_seed01_yesbias_hyb','obs')
obs_hyb = obs;
clear obs

% define errors  - steady state
pos=200;
params = obs_ekf.init.params;
for i=1:params.Ntraj
    % hyb
    eh(i).val([params.pos_p params.pos_v],:) = obs_hyb.init.X(i).val([params.pos_p params.pos_v],:)-obs_hyb.init.X_est(i).val([params.pos_p params.pos_v],:);    
    eh(i).val(params.pos_acc,:) = squeeze(obs_hyb.init.Y_full_story(i).val(1,params.pos_acc_out,:))-obs_hyb.init.X_est(i).val([params.pos_acc],:);
    deh.sigmax_ss((i)) = std(eh(i).val(params.pos_p(1),pos:end));
    deh.sigmay_ss((i)) = std(eh(i).val(params.pos_p(2),pos:end));
    deh.sigmaz_ss((i)) = std(eh(i).val(params.pos_p(3),pos:end));
    deh.meanx_ss((i)) = mean(eh(i).val(params.pos_p(1),pos:end));
    deh.meany_ss((i)) = mean(eh(i).val(params.pos_p(2),pos:end));
    deh.meanz_ss((i)) = mean(eh(i).val(params.pos_p(3),pos:end));

    % ekf
    ek(i).val([params.pos_p params.pos_v],:) = obs_ekf.init.X(i).val([params.pos_p params.pos_v],:)-obs_ekf.init.X_est(i).val([params.pos_p params.pos_v],:);    
    ek(i).val(params.pos_acc,:) = squeeze(obs_ekf.init.Y_full_story(i).val(1,params.pos_acc_out,:))-obs_ekf.init.X_est(i).val([params.pos_acc],:);
    dek.sigmax_ss((i)) = std(ek(i).val(params.pos_p(1),pos:end));
    dek.sigmay_ss((i)) = std(ek(i).val(params.pos_p(2),pos:end));
    dek.sigmaz_ss((i)) = std(ek(i).val(params.pos_p(3),pos:end));
    dek.meanx_ss((i)) = mean(ek(i).val(params.pos_p(1),pos:end));
    dek.meany_ss((i)) = mean(ek(i).val(params.pos_p(2),pos:end));
    dek.meanz_ss((i)) = mean(ek(i).val(params.pos_p(3),pos:end));
end

% define errors  - transient
pos=200;
params = obs_ekf.init.params;
for i=1:params.Ntraj
    % hyb
    eh(i).val([params.pos_p params.pos_v],:) = obs_hyb.init.X(i).val([params.pos_p params.pos_v],:)-obs_hyb.init.X_est(i).val([params.pos_p params.pos_v],:);    
    eh(i).val(params.pos_acc,:) = squeeze(obs_hyb.init.Y_full_story(i).val(1,params.pos_acc_out,:))-obs_hyb.init.X_est(i).val([params.pos_acc],:);
    deh.sigmax_tt((i)) = std(eh(i).val(params.pos_p(1),1:pos));
    deh.sigmay_tt((i)) = std(eh(i).val(params.pos_p(2),1:pos));
    deh.sigmaz_tt((i)) = std(eh(i).val(params.pos_p(3),1:pos));
    deh.meanx_tt((i)) = mean(eh(i).val(params.pos_p(1),1:pos));
    deh.meany_tt((i)) = mean(eh(i).val(params.pos_p(2),1:pos));
    deh.meanz_tt((i)) = mean(eh(i).val(params.pos_p(3),1:pos));

    % ekf
    ek(i).val([params.pos_p params.pos_v],:) = obs_ekf.init.X(i).val([params.pos_p params.pos_v],:)-obs_ekf.init.X_est(i).val([params.pos_p params.pos_v],:);    
    ek(i).val(params.pos_acc,:) = squeeze(obs_ekf.init.Y_full_story(i).val(1,params.pos_acc_out,:))-obs_ekf.init.X_est(i).val([params.pos_acc],:);
    dek.sigmax_tt((i)) = std(ek(i).val(params.pos_p(1),1:pos));
    dek.sigmay_tt((i)) = std(ek(i).val(params.pos_p(2),1:pos));
    dek.sigmaz_tt((i)) = std(ek(i).val(params.pos_p(3),1:pos));
    dek.meanx_tt((i)) = mean(ek(i).val(params.pos_p(1),1:pos));
    dek.meany_tt((i)) = mean(ek(i).val(params.pos_p(2),1:pos));
    dek.meanz_tt((i)) = mean(ek(i).val(params.pos_p(3),1:pos));
end

% plots

%%%%%%%%%%%%%%% POSITION %%%%%%%%%%%%%
% position - x
figure()
subplot(3,1,1)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_p(1),:));
    plot(ek(i).val(params.pos_p(1),:));
end

% position - y
subplot(3,1,2)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_p(2),:));
    plot(ek(i).val(params.pos_p(2),:));
end

% position - z
subplot(3,1,3)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_p(3),:));
    plot(ek(i).val(params.pos_p(3),:));
end

%%%%%%%%%% VELOCITY %%%%%%%%%%
% velocity - x
figure()
subplot(3,1,1)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_v(1),:));
    plot(ek(i).val(params.pos_v(1),:));
end

% velocity - y
subplot(3,1,2)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_v(2),:));
    plot(ek(i).val(params.pos_v(2),:));
end

% velocity - z
subplot(3,1,3)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_v(3),:));
    plot(ek(i).val(params.pos_v(3),:));
end

%%%%%%%%%% ACCELERATION %%%%%%%%%%
% acceleration - x
figure()
subplot(3,1,1)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_acc(1),:));
    plot(ek(i).val(params.pos_acc(1),:));
end

% acceleration - y
subplot(3,1,2)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_acc(2),:));
    plot(ek(i).val(params.pos_acc(2),:));
end

% acceleration - z
subplot(3,1,3)
hold on
grid on
for i=1:params.Ntraj
    plot(eh(i).val(params.pos_acc(3),:));
    plot(ek(i).val(params.pos_acc(3),:));
end



