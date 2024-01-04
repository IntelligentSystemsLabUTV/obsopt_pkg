%% generate_plant
% file: generate_plant.m
% author: Federico Oliva
% date: 20/12/2023
% description: script generating data from a nominal Van Der Pol oscillator
% info: see obsopt_manual.pdf for reference
% INPUT: none
% OUTPUT: 
%           Y: measurements
%           X: state
%           U: input
%           T: time vector

%% Init Section 
% clear and cose all
clc
clear
close all

% random seed init
rng('default');

% set sampling time
Ts = 1e-2;

% set initial and final time instant
T0 = 0;
Tend = 10;

% init Params (see ref)
% in this case look at the code in "params_oscillator_VDP.m"
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


% create an instance of obsopt
% we do this just because it's easy to use the obsopt class but you can do
% differently
obs = obsopt('Params',Params);

% set the noise characteristics
% enable noise
noise_flag = 1;
% noise mean
noise_mu = zeros(Params.DimOut,1);
% noise standard deviation
noise_std = 5e-2*ones(Params.DimOut,1);

% store initial conditions in the output variables
for traj = 1:Params.Ntraj

    % state
    X(traj).val(:,1) = Params.X(traj).val(:,1);

    % input
    U(traj).val(:,1) = Params.Input(T0,X(traj).val(:,1),Params,[]);

    % measure
    % noise considers the characteristics from the init section
    [Y(traj).val(:,1), ~] = Params.Measure(X(traj).val(:,1),Params,Params.T(1:2),U(traj).val(:,1),[]);
    Y(traj).val(:,1) = Y(traj).val(:,1) + noise_flag*(noise_mu + noise_std.*randn(size(noise_std)));

end

%% model integration
% cycle over the time vector starting from 2 because the initial condition
% has been set before
for i = 2:Params.Niter

    % tspan
    tspan = Params.T(i-1:i);

    % cycle over the trajectories (see ref)
    for traj = 1:Params.Ntraj

        % true system - correct initial condition and no noise considered                 
        Xtmp = Params.Ode(@(t,x)Params.Model(t, x, Params, obs), tspan, X(traj).val(:,i-1),Params.Odeset); 
        
        % assign state
        X(traj).val(:,i-1:i) = [Xtmp.y(:,1),Xtmp.y(:,end)];

        % assign input
        U(traj).val(:,i) = Params.Input(T0,X(traj).val(:,i-1),Params,obs);

        % measure
        % noise considers the characteristics from the init section
        [Y(traj).val(:,i), ~] = Params.Measure(X(traj).val(:,i),Params,Params.T(i-1:i),U(traj).val(:,i),[]);
        Y(traj).val(:,i) = Y(traj).val(:,i) + noise_flag*(noise_mu + noise_std.*randn(size(noise_std)));

    end

end

% assign the time vector to the time output variable
T = Params.T;

% clear workspace except for the outputs
keep X Y U T
