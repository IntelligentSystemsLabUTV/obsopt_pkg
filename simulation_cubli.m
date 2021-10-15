function [params,obs] = simulation_cubli

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc

% init model
Ts = 1e-1;
params = cubli_init(Ts);

% ode
params.ode = @oderk4;

% init observer
Nw = 5;
NTs = 3;
obs = obsopt_cubli(Ts,Nw,NTs);
obs = obs.obs_init(params.OutDim,params.StateDim);
obs.init.time = params.time;
obs.init.Niter = params.Niter;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:params.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(params.time(i)),'/',num2str(params.time(params.Niter))])
    end
    
    % DynOpt current iter 
    params.iter = i;
    obs.init.ActualTimeIndex = i;
    obs.init.t = params.time(i);
    
    %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    if(params.iter > 1)
        X = params.ode(@(t,x)params.model(t, x, params), params.tspan, params.X(:,params.iter-1));   
        params.X(:,params.iter) = X.y(:,end);
        
        X = params.ode(@(t,x)params.model(t, x, params), params.tspan, params.X_est(:,params.iter-1));
        params.X_est(:,params.iter) = X.y(:,end);      
    end
    
    
    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%    
    params.Y(:,params.iter) = params.measure(params.X(:,params.iter),params) + params.noise*(1e-1 + 5e-2*randn);
    
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    obs.init.X = params.X;
    obs.init.X_est = params.X_est;
    obs = observer(obs,params.X_est(:,params.iter),params.Y(:,params.iter),params);
    params.X = obs.init.X;
    params.X_est = obs.init.X_est;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
figure()
start = 1;
y_count = 0;
for i=1:params.StateDim
    subplot(params.StateDim,1,i);
    hold on
    grid on
    box on
    plot(params.time(start:end),params.X(i,start:end),'b--');
    plot(params.time(start:end),params.X_est(i,start:end),'r.');
    
    if find(params.observed_state == i)
        y_count = y_count+1;
        plot(params.time(start:end),params.Y(y_count,start:end),'k--');
    end
end

figure()
hold on
box on
grid on
plot(obs.init.Jstory(start:end));
end

