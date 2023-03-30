%%
clear  e ex ey ez meanx meany meanz sigmax sigmay sigmaz 
start=3000;

tmp = 'results/simulations/rover/testsferlazza/10TRAJ_200S_LINEAR_YESBIAS_YESNOISE_RANDHILL_YP_SEED0';
for sim=1:5
    name(sim).val = [tmp, num2str(sim)];
end
sigma = [];
for sim = 1:5
    load(name(sim).val)
    for i=1:params.Ntraj
        e(i).val = obs.init.X(i).val-obs.init.X_est(i).val;    
        sigma(:,end+1) = std(e(i).val(params.pos_p,:)');
    end
end

clear start i sim tmp name 

%% 
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