%% simulation_general
% file: simulation_general.m
% author: Federico Oliva
% date: 20/12/2023
% description: function to setup and use the MHE Observer on general model
% INPUT: 
%           Y: measurements
%           X: state
%           U: input
%           T: time vector
% OUTPUT: 
%           Obs: obsopt instance
%           Params: structure with all the necessary parameter to the model
function [Obs,Params] = simulation_general(X, U, Y, T)

    % initialize Params: check model init and ref
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
    % options check ref and the class constructor in Obsopt.m
    Obs = obsopt('Params',          Params, ...
                 'Optimise',        1, ...
                 'Print',           0, ...
                 'MaxIter',         100, ...
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
                 'LBcon',           [-Inf -Inf -3], ...
                 'UBcon',           [+Inf +Inf +3], ...
                 'NONCOLcon',       [] ...
                 );

    % cycle over trajectories (see ref)
    for traj = 1:Obs.Params.Ntraj

        % init state
        Obs.X(traj).val = X(traj).val;

        % init input
        Obs.U(traj).val = U(traj).val;

        % init input - estimated
        % remark: here I assume an observation problem, i.e., the input is the
        % same as the nominal plant. If you want to consider a control law
        % depending on the model state and parameters, code it in "Input" and
        % add it in the "Model" function. 
        Obs.Uhat(traj).val = U(traj).val;

        % init measurement
        Obs.Y(traj).val = Y(traj).val;

    end

    % simulation
    % start time clock (we will save the execution time)
    t0 = tic;

    % integration loop
    for i = 2:Obs.Params.Niter
    
        % Display iteration step
        if ((mod(i,10) == 0) || (i == 1))

            % clear screen
            clc

            % show iteration and last cost function
            disp(['Iteration Number: ', num2str(Obs.T(i)),'/',num2str(Obs.T(Obs.Params.Niter))])
            disp(['Last J:', num2str(Obs.Params.JStory(end,Obs.Params.ActualTimeIndex))]);

        end
    
        % set current iteration in the Obsopt class
        Obs.Params.ActualTimeIndex = i;

        % define time span (single integration)
        % starting position (the max() is to avoid index 0)
        startpos = max(1,Obs.Params.ActualTimeIndex-1);
        % stop position
        stoppos = Obs.Params.ActualTimeIndex;
        % time interval
        tspan = Obs.T(startpos:stoppos);
    
        % propagation
        % forward propagation of the previous estimate    
        % cycle over the trajectories  
        for traj = 1:Obs.Params.Ntraj
        
            % update traj
            Obs.Params.Traj = traj;
             
            % integrate the estimated plant (check Model for the equations)          
            Xtmp = Params.Ode(@(t,x)Params.Model(t, x, Params, Obs), tspan, Obs.Xhat(traj).val(:,i-1),Params.Odeset); 
        
            % assign state
            Obs.Xhat(traj).val(:,i-1:i) = [Xtmp.y(:,1),Xtmp.y(:,end)];

            % measure
            Ymeas(traj).val = Obs.Y(traj).val(:,Obs.Params.ActualTimeIndex);
        
        end
    
        % MHE observer: calls the obs.observer method
        % start the clock to measure the execcution time of the observer
        t1 = tic;
        % call the observer with the last measure
        Obs = Obs.observer(Obs.Xhat,Ymeas);
        % stop the clock and save the execution time
        Obs.Params.IterTime(Obs.Params.ActualTimeIndex) = toc(t1);   
    
    end

    % stop the clock and save the overall computation time
    Obs.Params.TotalTime = toc(t0);

    % the whole process could be long, why not going for a nap? No worries, 
    % this "sounds" like a nice way to wake up. (Uncomment next two lines)
    % load handel
    % sound(y,Fs)
end

