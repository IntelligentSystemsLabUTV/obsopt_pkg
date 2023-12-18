%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE Observer on general model
% INPUT: none
% OUTPUT: Params,Obs
function [Obs,Params] = simulation_general(X, U, Y, T)

% initialize Params
Params = model_init('ParamsInit',       @params_oscillator_VDP, ...
                    'Ts',               T(2)-T(1), ...
                    'T0',               T(1), ...
                    'Tend',             T(end), ...
                    'Model',            @model_oscillator_VDP, ...     
                    'Measure',          @measure_general, ...
                    'Ode',              @odeEuler, ...
                    'InputLaw',         @control, ...
                    'ParamsUpdate',     @params_update_oscillator_VDP, ...
                    'Ntraj',            1 ...
                    );


% create Observer class instance. For more information on the setup
% options check directly the class constructor in Obsopt.m
Obs = obsopt('Params',          Params, ...
             'Optimise',        1, ...
             'Print',           0, ...
             'MaxIter',         50, ...
             'N',               Params.N, ...
             'Nts',             Params.Nts, ...
             'JdotThresh',      1, ...
             'JNormalise',      1, ...
             'BoundsPos',       [], ...
             'BoundsValLow',    [], ...
             'BoundsValUp',     [], ...
             'BoundsWeight',    [], ...
             'Acon',            [], ...
             'Bcon',            [], ...
             'AconEq',          [], ...
             'BconEq',          [], ...
             'LBcon',           [], ...
             'UBcon',           [], ...
             'NONCOLcon',       [] ...
             );

% cycle over trajectories
for traj = 1:Obs.Params.Ntraj

    % init state
    Obs.X(traj).val = X(traj).val;

    % init input
    Obs.U(traj).val = U(traj).val;

    % init input -estimated
    % remark: here I assume an observation problem, i.e., the input is the
    % same as the nominal plant. If you want to consider a cntrol law
    % depending on the model state and parameters, code it in "Input" and
    % add it in the "Model" function. 
    Obs.Uhat(traj).val = U(traj).val;

    % init measurement
    Obs.Y(traj).val = Y(traj).val;

end

%% %%%% SIMULATION %%%%
% start time counter
t0 = tic;

% integration loop
for i = 2:Obs.Params.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(Obs.T(i)),'/',num2str(Obs.T(Obs.Params.Niter))])
        disp(['Last J:', num2str(Obs.Params.JStory(end,Obs.Params.ActualTimeIndex))]);
    end
    
    % set current iteration in the Obsopt class
    Obs.Params.ActualTimeIndex = i;

    % define time span (single integration)
    startpos = max(1,Obs.Params.ActualTimeIndex-1);
    stoppos = Obs.Params.ActualTimeIndex;
    tspan = Obs.T(startpos:stoppos);
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate    
        
    for traj = 1:Obs.Params.Ntraj
        
        % update traj
        Obs.Params.Traj = traj;
             
        % estimated system          
        Xtmp = Params.Ode(@(t,x)Params.Model(t, x, Params, Obs), tspan, Obs.Xhat(traj).val(:,i-1),Params.Odeset); 
        
        % assign state
        Obs.Xhat(traj).val(:,i-1:i) = [Xtmp.y(:,1),Xtmp.y(:,end)];

        % measure
        Ymeas(traj).val = Obs.Y(traj).val(:,Obs.Params.ActualTimeIndex);
        
    end
    
    %%%% MHE OBSERVER (SAVE MEAS) %%%%
    t1 = tic;
    Obs = Obs.observer(Obs.Xhat,Ymeas);
    Obs.Params.IterTime(Obs.Params.ActualTimeIndex) = toc(t1);   
    
end

% overall computation time
Obs.Params.TotalTime = toc(t0);

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

