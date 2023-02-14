%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [params,obs] = simulation_realdata(data,time)

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 60;
Nts = 5;

% noise
rng default
first_guess_flag = 1;

% set sampling time
Ts = 1e0;

% set initial and final time instant
% remember to set the final time and sampling time accordingly to the data
% that you measured
t0 = time(1);
tend = time(end);
% uncomment to test the MHE with a single optimisation step
% tend = 1*(Nw*Nts-1)*Ts;

%%%%%% general functions
params_init = @params_battery_tushar;
params_update = @params_update_battery_tushar;
model = @model_battery_tushar;
measure = @measure_battery_tushar;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% filters %%%%
[filter, filterScale, reference] = filter_define(Ts,Nts);

%%%% integration method %%%%
% ode45-like integration method. For discrete time systems use @odeDD
ode = @odeDD;

%%%% input law %%%
input_law = @control_battery;

%%%% measurement noise %%%%
% this should be a vector with 2 columns and as many rows as the state
% dimension. All the noise are considered as Gaussian distributed. The 
% first column defines the mean while the second column the variance.
noise_mat = 0*ones(6,2);

%%%% params init %%%%
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1, 'params_update', params_update, ...
        'model',model,'measure',measure,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'input_law',input_law,'params_init',params_init);

%%%% observer init %%%%
obs = obsopt('DataType', 'measured', 'optimise', 1 , 'GlobalSearch', 0, 'MultiStart', 0, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 1, 'params',params, 'filters', filterScale,'filterTF', ...
      filter, 'Jdot_thresh',0.9,'MaxIter',20, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 6, 'AdaptiveHist', [5e-4, 1e-3, 1e-3], ...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminsearch, 'terminal', 0, 'terminal_states', [1:2], 'terminal_weights', [1 1], 'terminal_normalise', 1, ...
      'ConPos', [1], 'LBcon', [0], 'UBcon', [1], 'Bounds', 0);
  
  
%%%% measure filter state %%%%
obs.init.x_filter = reference.x0;

%%%% first guess %%%%
if first_guess_flag
    first_guess;        
end

%% %%%% SIMULATION %%%%
% generate data
%%%% SWITCH Y WITH THE COLLECTED DATA %%%%%%
y = data;

%%% THIS SHOULD REMAIN IN THE CODE %%%
obs.init.Y_full_story(1).val = reshape(y,size(obs.init.Y_full_story(1).val,1),size(obs.init.Y_full_story(1).val,2),size(y,2));

% start time counter
t0 = tic;

% integration loop
for i = 1:obs.setup.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
        disp(['Last J: ',num2str(obs.init.Jstory(end))]);
    end
    
    % set current iteration in the obsopt class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate
    for traj = 1:obs.setup.Ntraj
                 
        % propagate only if the time gets over the initial time instant
        if(obs.init.ActualTimeIndex > 1)
            
            % define the time span of the integration
            startpos = obs.init.ActualTimeIndex-1;
            stoppos = obs.init.ActualTimeIndex;
            tspan = obs.setup.time(startpos:stoppos);                        

            % real system - initial condition perturbed 
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];      
        end
        
        %%%% FILTER MEASUREMENT %%%%                 
        % filter on the measurements
        if 0
            % reference filter (filterScale)
            try
                u = [y(:,obs.init.ActualTimeIndex);reshape(y(1,:,obs.init.ActualTimeIndex),1,obs.setup.dim_out)];
            catch
                u = [y(:,obs.init.ActualTimeIndex);zeros(1,obs.setup.dim_out)];
            end
            [y_f, ~, x] = lsim(reference.TF,u,obs.setup.tspan,obs.init.x_filter(max(1,obs.init.ActualTimeIndex-1)));  
            obs.init.x_filter(:,obs.init.ActualTimeIndex) = x(end);
            y_meas(traj).val = transpose(y_f(end,:));
        else
            y_meas(traj).val = y(:,obs.init.ActualTimeIndex);
        end
                            
    end
      
    %%%% OBSERVER %%%%
    % start time counter for the observation
    t1 = tic;
    
    % call the observer
    obs = obs.observer(obs.init.X_est,y_meas);    
    
    % stop time counter for the observer. Each optimisation process is
    % timed.
    obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);

end

% overall computation time
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

