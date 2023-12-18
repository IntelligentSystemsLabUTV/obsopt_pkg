%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: out

%%%% Init Section %%%%
clc
clear
close all
rng('default');

% set sampling time
Ts = 1e-2;

% set initial and final time instant
T0 = 0;
Tend = 10;

% initialize Params
Params = model_init('ParamsInit',       @params_oscillator_VDP, ...
                    'Ts',               Ts, ...
                    'T0',               T0, ...
                    'Tend',             Tend, ...
                    'Model',            @model_oscillator_VDP, ...     
                    'Measure',          @measure_general, ...
                    'Ode',              @odeEuler, ...
                    'InputLaw',         @control, ...
                    'ParamsUpdate',     @params_update_oscillator_VDP, ...
                    'Ntraj',            1 ...
                    );


% create an instance of obs (just because I am using the model)
obs = obsopt('Params',Params);

% define initial conditions
for traj = 1:Params.Ntraj

    % state
    X(traj).val(:,1) = Params.X(traj).val(:,1);

    % input
    U(traj).val(:,1) = Params.Input(T0,X(traj).val(:,1),Params,[]);

    % measure
    [Y(traj).val(:,1), ~] = Params.Measure(X(traj).val(:,1),Params,Params.T(1:2),U(traj).val(:,1),[]);

end

%% model integration

% cycle over the time vector
for i = 2:Params.Niter

    % tspan
    tspan = Params.T(i-1:i);

    % cycle over the trajectories
    for traj = 1:Params.Ntraj

        % true system - correct initial condition and no noise considered                 
        Xtmp = Params.Ode(@(t,x)Params.Model(t, x, Params, obs), tspan, X(traj).val(:,i-1),Params.Odeset); 
        
        % assign state
        X(traj).val(:,i-1:i) = [Xtmp.y(:,1),Xtmp.y(:,end)];

        % assign input
        U(traj).val(:,i) = Params.Input(T0,X(traj).val(:,i-1),Params,obs);

        % measure
        [Y(traj).val(:,i), ~] = Params.Measure(X(traj).val(:,i),Params,Params.T(i-1:i),U(traj).val(:,i),[]);

    end

end

T = Params.T;
keep X Y U T
