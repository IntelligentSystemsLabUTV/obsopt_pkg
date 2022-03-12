function [params,obs] = model_test_v1(obs)

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
close all

% init model
% set sampling time
Ts = 1e-1;

% set initial and final time instant
t0 = 0;
tend = 10;
%     tend = (Nw*Nts+1)*Ts;

%%%%%%%%%%% params function %%%%%%%%%%%
params_init = @params_double_pendulum_default;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% model function %%%%%%%%%%%
model = @model_double_pendulum_default;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% measure function %%%%%%%%%%%
measure = @measure_general;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set the integration method
ode = @oderk4;

% define the input law used: here it's just for a test. You can also
% comment out this line, a default sinusoidal input is hard coded in
% model_init();
input_law = @control;

% init the parameters structure. The model_init file has lots of setup
% options (varargin). The most important is the 'params_init' option, 
% which takes as input the function handle to the previously defined
% @params_init. For more information see directly the file.
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',[0, 0],...
        'model',model,'measure',measure,'StateDim',10,'ObservedState',[1 2],'ode',ode,...
        'input_enable',1,'dim_input',1,'input_law',input_law,'params_init',params_init);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integration loop
tic
for i = 1:params.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(params.time(i)),'/',num2str(params.time(params.Niter))])
    end
    
    % set current interation in the class
    params.ActualTimeIndex = i;
    params.t = params.time(i);
    
    %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    for traj = 1:params.Ntraj
        
        if(params.ActualTimeIndex > 1)
            % params update
            x = obs.init.X_est(1).val(:,params.ActualTimeIndex-1);
            params = params_update(params,x);
            
            % input
            params.u = params.input(params.X_est(traj).val(:,params.ActualTimeIndex-1),params);

            % true system
            X = params.ode(@(t,x)model(t, x, params), params.tspan, params.X_est(traj).val(:,params.ActualTimeIndex-1));   
            params.X_est(traj).val(:,params.ActualTimeIndex) = X.y(:,end);   
        end
        
    end

end
obs.init.total_time = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
for i = 1:params.Ntraj
    figure(1);
    hold on
    plot(params.time,params.X_est(i).val(1:2,:));

    figure(2)
    hold on
    plot(params.time,params.X_est(i).val(3:4,:));
end
end

