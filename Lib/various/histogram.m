%% histogram plot

T = 0;
N=9;

figure(1)
grid on
box on
hold on
figure(2)
grid on
box on
hold on
figure(3)
grid on
box on
hold on
distrib = 'normal';

load results/simulations/Pendulum/finalTest/Pendulum_0905_Ts005_7s_40iter_fminsearch.mat
time_pos = find(obs.init.opt_time > T);
time_vec = obs.init.opt_time(time_pos);
p1 = fitdist(time_vec','Normal');

figure(1)
histfit(time_vec,N,distrib);
figure(2)
plot(obs.init.opt_time)
figure(3)
for iter=1:obs.setup.Niter
    est_error_norm(iter) = norm(obs.init.X(1).val(:,iter) - obs.init.X_est_runtime(1).val(:,iter));
end 
set(gca, 'YScale', 'log')
plot(obs.setup.time,abs(est_error_norm),'k','LineWidth',2);


load results/simulations/Pendulum/finalTest/Pendulum_0505_Ts005_7s_40iter_fminsearch.mat
time_pos = find(obs.init.opt_time > T);
time_vec = obs.init.opt_time(time_pos);
p2 = fitdist(time_vec','Normal');

figure(1)
histfit(time_vec,ceil(N/2),distrib);
figure(2)
plot(obs.init.opt_time)
figure(3)
for iter=1:obs.setup.Niter
    est_error_norm(iter) = norm(obs.init.X(1).val(:,iter) - obs.init.X_est_runtime(1).val(:,iter));
end 
set(gca, 'YScale', 'log')
plot(obs.setup.time,abs(est_error_norm),'k--','LineWidth',2);

load results/simulations/Pendulum/finalTest/Pendulum_0305_Ts005_7s_40iter_fminsearch.mat
time_pos = find(obs.init.opt_time > T);
time_vec = obs.init.opt_time(time_pos);
p3 = fitdist(time_vec','Normal');    

figure(1)
histfit(time_vec,ceil(N/3),distrib);
figure(2)
plot(obs.init.opt_time)
figure(3)
for iter=1:obs.setup.Niter
    est_error_norm(iter) = norm(obs.init.X(1).val(:,iter) - obs.init.X_est_runtime(1).val(:,iter));
end 
set(gca, 'YScale', 'log')
plot(obs.setup.time,abs(est_error_norm),'k:','LineWidth',2);


% legends and labels
figure(1)
legend('N=9','N=9 (fit)','N=5 + G_f','N=5 + G_f (fit)','N=3 + G_f + G_i','N=3 + G_f + G_i (fit)')
xlabel('time [s]')
ylabel('occurrencies')
figure(2)
legend('N=9','N=5 + G_f','N=3 + G_f + G_i')
xlabel('time [s]')
ylabel('iteration time')
figure(3)
legend('N=9','N=5 + G_f','N=3 + G_f + G_i')
xlabel('time [s]')
ylabel('|e|')