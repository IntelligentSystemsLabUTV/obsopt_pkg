%% histogram plot
T = 0;
figure(1)
grid on
box on
hold on

load results/simulations/Pendulum/finalTest/Pendulum_0305_Ts005_7s_40iter_fminsearch.mat
time_pos = find(obs.init.opt_time > T);
time_vec = obs.init.opt_time(time_pos);
p1 = fitdist(time_vec','Normal');
figure(1)
for iter=1:obs.setup.Niter
    est_error_norm(iter) = norm(obs.init.X(1).val(:,iter) - obs.init.X_est_runtime(1).val(:,iter));
end 
set(gca, 'YScale', 'log')
plot(obs.setup.time,abs(est_error_norm),'k','LineWidth',2);

load results/simulations/Pendulum/finalTest/Pendulum_0305_Ts005_7s_40iter_fminsearch_adaptive.mat
time_pos = find(obs.init.opt_time > T);
time_vec = obs.init.opt_time(time_pos);
p2 = fitdist(time_vec','Normal');
figure(1)
for iter=1:obs.setup.Niter
    est_error_norm(iter) = norm(obs.init.X(1).val(:,iter) - obs.init.X_est_runtime(1).val(:,iter));
end 
set(gca, 'YScale', 'log')
plot(obs.setup.time,abs(est_error_norm),'k--','LineWidth',2);


figure(1)
legend('N=3','N=3 adaptive')
xlabel('time [s]')
ylabel('|e|')