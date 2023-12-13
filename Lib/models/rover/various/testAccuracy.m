%%
clear  e ex ey ez meanx meany meanz sigmax sigmay sigmaz 
start=10000;


if 0
%     tmp = 'results/simulations/rover/testhyb/10TRAJ_200S_LINEAR_YESBIAS_YESNOISE_RANDHILL_YP_SEED0';
%     tmp = 'results/simulations/rover/testEKF/5TRAJ_200S_EKF_YESBIAS_YESNOISE_RANDHILL_YP_SEED0';
    nsim = 5;
    for sim=1:nsim
        name(sim).val = [tmp, num2str(sim)];
    end
else
%     tmp = 'results/simulations/rover/testEKF/5TRAJ_200S_EKF_YESBIAS_YESNOISE_RANDHILL_YP_SEED01.mat';
%     tmp = 'results/simulations/rover/testhyb/01TRAJ_200S_LINEAR_YESBIAS_NONOISE_RANDHILL_YP_SEED23';
    tmp = 'results/simulations/rover/testhyb/01TRAJ_200S_CUBIC_YESBIAS_YESNOISE_RANDHILL_YP_SEED23.mat';
    nsim = 1;
    name(1).val = tmp;
end
sigma = [];
avg = [];
for sim = 1:nsim
    load(name(sim).val)
    for i=1:params.Ntraj
        e(i).val = obs.init.X(i).val-obs.init.X_est(i).val;    
        sigma(:,end+1) = std(e(i).val(params.pos_p,start:end)');
        avg(:,end+1) = mean(e(i).val(params.pos_p,start:end)');
    end
end

clear start i sim tmp name 

%% 
if 0
    startplot = 1;
    stopplot = 1000;
    c = 'b';
    
    figure(1)
    hold on
    plot(obs.setup.time(startplot:stopplot),e.val(params.pos_p,startplot:stopplot)',c,'LineWidth',2)
    xlabel('time [s]')
    ylabel('e_p [m]')
    
    figure(2)
    hold on
    plot(obs.setup.time(startplot:stopplot),e.val(params.pos_v,startplot:stopplot)',c,'LineWidth',2)
    xlabel('time [s]')
    ylabel('e_v [m]')
    
    figure(3)
    hold on
    plot(obs.setup.time(startplot:stopplot),e.val(params.pos_acc,startplot:stopplot)',c,'LineWidth',2)
    xlabel('time [s]')
    ylabel('e_a [m]')
    
    figure(4)
    hold on
    plot(obs.setup.time(startplot:stopplot),e.val(params.pos_bias,startplot:stopplot)',c,'LineWidth',2)
    xlabel('time [s]')
    ylabel('e_b [m]')
    
    clear startplot stopplot c
end