%% obsopt
% file: obsopt.m
% author: Federico Oliva
% date: 20/12/2023
% description: class implementing MHE observers and TBOD methods (see ref)

% class definition - obsopt observer
% the obsopt class is a handle class: Objects of handle classes are 
% referred to by reference, meaning that when you assign a handle object 
% to a variable or pass it as an argument to a function, you're actually 
% working with a reference to the object rather than a copy of the 
% object itself.
classdef obsopt < handle

    %%% class properties
    properties

        % Params: structure containing all the useful variables
        Params;

        % methods provided from external scripts (function handles)
        Functions;

        % simulation time vector
        T;

        % state of the plant - nominal (optional)
        X;

        % output of the plant - nominal (optional)
        Y;

        % control input of the plant - nominal 
        U;

        % state of the plant - estimated
        Xhat;

        % output of the plant - estimated
        Yhat;

        % control input of the plant  - estimated
        Uhat;
    end
    
    %%% class methods (built-in)
    methods

        % class constructor. Each of these variables can be specifically
        % set by calling the constructor with specific properties, i.e. by
        % properly using the varargin parameter. Some of then are mandatory,
        % others are optional. In this case a default value is used
        function Obj = obsopt(varargin)
            
            % set the Params structure. This structure is used as a
            % container for all the variables that might be useful during
            % the simulation. There are some standard variables within
            % Params, but the user can freely add or remove them depending
            % on th goal. The main approach to handle the class is to keep
            % all the properties fixed except from Params. Consider it as a
            % sort of workspace.
            try

                % check if the Params option is present in varagin
                any(strcmp(varargin,'Params'))
                
                % find the related position
                pos = find(strcmp(varargin,'Params'));

                % the structure is right after
                Params = varargin{pos+1};

                % assign Params
                Obj.Params = Params;
                
                % get model handle function from Params
                Obj.Functions.Model = Params.Model;
                
                % get measure handle function from Params
                Obj.Functions.Measure = Params.Measure;

                % get input handle function from Params
                Obj.Functions.Input = Params.Input;
                
                % get the integration algorithm handle function from Params
                Obj.Functions.Ode = Params.Ode;

                % store the integration options (see model_init.m)
                Obj.Params.Odeset = Params.Odeset;
               
                % get state initial value (see model_init)
                Obj.X = Params.X;       % nominal plant
                Obj.Xhat = Params.Xhat; % estimated plant

                % set the time vector
                Obj.T = Params.T;
                                
            catch 

                % something is missing, check the Params definition
                error('Error in parsing "Params"');

            end
             
            % the class implements an MHE observer ora TBOD method. One
            % might want to check the estimated plant behaviour in absence
            % of any observer, just to see how different initial
            % conditions, parameters or setup affect the estimated state
            % trajectories. in this case, no optimization problem should be
            % run in any instant of the simultion. If this is the case,
            % please set Optimize to zero. 
            if any(strcmp(varargin,'Optimize'))

                % find Optimize in varargin
                pos = find(strcmp(varargin,'Optimize'));

                % set the option in Params
                Obj.Params.Optimize = varargin{pos+1};

            else

                % Optimize is not mandatory. if not found, set to default 0
                warning('No "optimize" option provided: set default to: true');
                Obj.Params.Optimize = 0;
            end
                       
            % option to print out the optimization process or not. 
            if any(strcmp(varargin,'Print'))

                % find Print in varargin
                pos = find(strcmp(varargin,'Print'));

                % set the option in Params
                Obj.Params.Print = varargin{pos+1};

            else

                % Print is not mandatory. Default is not printing (0).
                warning('No "print" option provided: set default to: false');
                Obj.Params.Print = 0;

            end
            
            % get the maximum number of iterations in the optimization
            % process.
            if any(strcmp(varargin,'MaxIter'))

                % find MaxIter in varargin
                pos = find(strcmp(varargin,'MaxIter'));
                Obj.Params.MaxIter = varargin{pos+1};

            else

                % MaxIter is not mandatory. Default is 100
                warning('No "MaxIter" option provided: set default to: 100');
                Obj.Params.MaxIter = 100;

            end
           
            % get the conditions to consider the optimization result. 
            % Briefly, if the cost function of the MHE problem is at least 
            % a percentage lower than the initial one, accept the new state 
            % estimation. See reference for more detailed information. 
           if any(strcmp(varargin,'JdotThresh'))

               % find JdotThresh in varargin
                pos = find(strcmp(varargin,'JdotThresh'));

                % set the value in Params
                Obj.Params.JdotThresh = varargin{pos+1};

           else

                % JdotThresh is not mandatory. Default is 90%
                warning('No "JdotThresh" option provided: set default to: 0.9');
                Obj.Params.JdotThresh = 0.9;
            end
            
            % normalize the cost function. As pointed out in ref, the
            % different terms in the output mismatch used in the cost
            % function might have different orders of magnitude. thus, a
            % normalization is required. This option enables or not such a
            % procedure.
            if any(strcmp(varargin,'JNormalize'))

                % find JNormalize in varargin
                pos = find(strcmp(varargin,'JNormalize'));

                % set the value in Params
                Obj.Params.JNormalize = varargin{pos+1};

            else

                % JNormalize is not mandatory. Default is true
                warning('No "JNormalize" option provided: set default to: true');

                % If you don't want to normalize but still consider this
                % issue you can directly specify the cost function weights
                % in the Params definition. see reference
                warning('Remark: if you want to manually set the cost function weights, set "JNormalize = false" and specify the W matrices in the "Params" file.');

                % set the value
                Obj.Params.JNormalize = 1;

            end

            % get the optimization method. Default is fminsearchcon from
            % MATLAB. You might want to change this. In this case a
            % relevant rewriting of the obsopt class might be necessary. We
            % have already done this on other branches of the repository,
            % but the whole thing might be quite messy. So, feel free to
            % contact us for help. 
            Obj.Functions.Fmin = @fminsearchcon;
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsPos'))
                pos = find(strcmp(varargin,'BoundsPos'));
                Obj.Params.BoundsPos = varargin{pos+1};                
            else
                warning('No "BoundsPos" option provided: set default to: [].');
                Obj.Params.BoundsPos = [];
            end            
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsValLow'))
                pos = find(strcmp(varargin,'BoundsValLow'));
                Obj.Params.BoundsValLow = varargin{pos+1};                
            else
                warning('No "BoundsValLow" option provided: set default to: [].');
                Obj.Params.BoundsValLow = [];
            end
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsValUp'))
                pos = find(strcmp(varargin,'BoundsValUp'));
                Obj.Params.boundsValUp = varargin{pos+1};                
            else
                warning('No "BoundsValUp" option provided: set default to: [].');
                Obj.Params.BoundsValUp = [];
            end
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsWeight'))
                pos = find(strcmp(varargin,'BoundsWeight'));
                Obj.Params.BoundsWeight = varargin{pos+1};                
            else
                warning('No "BoundsWeight" option provided: set default to the identity matrix of dimension "BoundsPos".');
                Obj.Params.BoundsWeight = ones(1,numel(Obj.Params.BoundsPos));
            end
            
            % handle constraints
            if any(strcmp(varargin,'Acon'))
                pos = find(strcmp(varargin,'Acon'));
                Obj.Params.Acon = varargin{pos+1};
            else
                warning('No "Acon" option provided: set default to: [].');
                Obj.Params.Acon = [];                    
            end
            if any(strcmp(varargin,'Bcon'))
                pos = find(strcmp(varargin,'Bcon'));
                Obj.Params.Bcon = varargin{pos+1};
            else
                warning('No "Bcon" option provided: set default to: [].');
                Obj.Params.Bcon = [];                
            end
            if any(strcmp(varargin,'AconEq'))
                pos = find(strcmp(varargin,'AconEq'));
                Obj.Params.AconEq = varargin{pos+1};
            else
                warning('No "AconEq" option provided: set default to: [].');
                Obj.Params.AconEq = [];                
            end
            if any(strcmp(varargin,'BconEq'))
                pos = find(strcmp(varargin,'BconEq'));
                Obj.Params.BconEq = varargin{pos+1};
            else
                warning('No "BconEq" option provided: set default to: [].');
                Obj.Params.BconEq = [];                
            end
            if any(strcmp(varargin,'LBcon'))
                pos = find(strcmp(varargin,'LBcon'));
                Obj.Params.LBcon = varargin{pos+1};
            else
                warning('No "LBcon" option provided: set default to: -Inf.');
                Obj.Params.LBcon = -Inf*ones(1,length(Obj.Params.VarsOpt));                
            end
            if any(strcmp(varargin,'UBcon'))
                pos = find(strcmp(varargin,'UBcon'));
                Obj.Params.UBcon = varargin{pos+1};
            else
                warning('No "UBcon" option provided: set default to: Inf.');
                Obj.Params.UBcon = Inf*ones(1,length(Obj.Params.VarsOpt));          
            end
            if any(strcmp(varargin,'NONCOLcon'))
                pos = find(strcmp(varargin,'NONCOLcon'));
                Obj.Functions.NONCOLcon = varargin{pos+1}; 
            else
                warning('No "NONCOLcon" option provided: set default to @Obj.NONCOLcon.');
                Obj.Functions.NONCOLcon = @Obj.NONCOLcon;
            end
            
            % terms
            Obj.Params.JNterm = 1; % currently one for the Y. The arrival is an additional variable 

            % Output weights
            try
                warning('Remark: if you want to manually set the cost function weights, specify the "OutputWeights" matrices in the "Params" file.');
                Obj.Params.OutputWeights = Params.OutputWeights;
                Obj.Params.OutputWeightsScaled = Obj.Params.OutputWeights;
            catch
                error('No terminal weights found in "Params"');
            end
            
            % terminal weights
            try
                warning('Remark: if you want to manually set the cost function weights, specify the "TerminalWeights" matrices in the "Params" file.');
                Obj.Params.TerminalWeights = Params.TerminalWeights;
                Obj.Params.TerminalWeightsScaled = Obj.Params.TerminalWeights;
            catch
                error('No terminal weights found in "Params"');
            end
            
            % number of reference trajectories            
            Obj.Params.Ntraj = Params.Ntraj;
            Obj.Params.Traj = 1;

            % Nts saved and pos
            Obj.Params.Nsaved = 0;
            Obj.Params.NTsPos = 1;

            % set the params update
            Obj.Functions.ParamsUpdate = Params.ParamsUpdate;
            
            % create scale factor, namely the weight over time for all the
            % cost function terms. In V1.1 no forgetting factor is
            % implemented. 
            Obj.Params.Normalized = 0;
            
            % BackIterIndex: t0 index (starting from 1). For more
            % information check the reference. 
            Obj.Params.BackTimeIndex = 1; 
            Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);

            % ActualTime: t0 index (starting from 1). For more
            % information check the reference.
            Obj.Params.ActualTimeIndex = 1;
            Obj.Params.ActualTime = Obj.T(Obj.Params.ActualTimeIndex);
            
            % measure buffer: these buffers are used to store the measured
            % and estimated values on the observed states.           
            for i=1:Obj.Params.Ntraj

                % output
                Obj.Y(i).val =  zeros(Obj.Params.DimOut,Obj.Params.N);
                Obj.Yhat(i).val = zeros(Obj.Params.DimOut,0);
                
                % control
                Obj.U(i).val(:,1) = zeros(Obj.Params.DimInput,1);
                Obj.Uhat(i).val(:,1) = zeros(Obj.Params.DimInput,1);
            end
            
            % buffer adaptive sampling: these buffers keep track of the
            % time instants in which the measured data have been stored. 
            Obj.Params.YSpace = zeros(1,Obj.Params.N);
            Obj.Params.YSpaceFullStory = 1; % init to first element of T

            % buffer
            for i=1:Obj.Params.Ntraj
                Obj.Params.Ybuffer(i).val = zeros(1,Obj.Params.N);
                Obj.Params.Target(i).val = Obj.Params.Ybuffer(i).val;
                Obj.Params.TargetStory(i).val = [];
            end

            % buffer
            for i=1:Obj.Params.Ntraj
                Obj.Params.TempX0NonOpt(i).val = [];
                Obj.Params.TempX0Opt(i).val = [];
                Obj.Params.TempX0(i).val = [];
            end
         
            % observer cost function init
            Obj.Params.J = 0;
            Obj.Params.JStory = zeros(Obj.Params.MaxIter,Obj.Params.Niter);
            % reset OptIterVal
            Obj.Params.OptIterVal = 0;
             
            % J_components is used to keep track of the different cost
            % function terms amplitude. Not implemented in V1.1
            Obj.Params.JComponents = ones(Obj.Params.DimOut,Obj.Params.JNterm);
            Obj.Params.JustOptimized = 0;
            
            % time instants in which the optimization is run
            Obj.Params.SampleTime = [];
            % time instand in which the optimization is accepted 
            % (see J_dot_thresh for more information)
            Obj.Params.IterTime = [];
            Obj.Params.TotalTime = [];
            Obj.Params.ExecutionTime = [];
            
            % optimization counters. Related to SampleTime and chosen_time
            Obj.Params.OptCounter = 0;
            Obj.Params.SelectCounter = 0;
            Obj.Params.OptTime = [];
            Obj.Params.SelectTime = [];
            
            %%% start of optimization setup %%%
            % optimset: check documentation for fminsearch or fminunc
            Obj.Params.TolX = 0;
            Obj.Params.TolFun = 0;
            Obj.Params.DiffMinChange = 1e-3;
            
            % set options
            if Obj.Params.Print 
                Obj.Params.Display = 'iter';
            else
                Obj.Params.Display = 'off'; 
            end
    
            
            % optimset                    
            Obj.Params.Myoptioptions = optimset('MaxIter',              Obj.Params.MaxIter, ...
                                                'display',              Obj.Params.Display, ...
                                                'MaxFunEvals',          Inf, ...
                                                'OutputFcn',            @Obj.outfun, ...
                                                'TolFun',               Obj.Params.TolFun, ...
                                                'TolX',                 Obj.Params.TolX, ...
                                                'TolCon',               1e-10 ...
                                                );
        end


        % nonlinear constraing default
        function [c, ceq] = NONCOLcon(varargin)
            c = 0;
            ceq = 0;
        end
        
        % outfun: this method check wether or not the optimization process
        % shall be stopped or not. Check setup.J_thresh for more
        % information. 
        function stop = outfun(Obj,x, optimValues, state)
            if (optimValues.iteration == Obj.Params.MaxIter) || (optimValues.fval <= Obj.Params.TolFun)
                stop = true;
            else
                stop = false;
            end
        end
        
        
        % cost function: Objective to be minimised by the MHE observer
        function [J_final,Obj] = cost_function(Obj,varargin) 

            % init the cost function
            J_final = 0;

            % update the iterval
            Obj.Params.OptIterVal = Obj.Params.OptIterVal + 1;

            % get opt state - same for all the trajectories
            x_opt = varargin{1};

            % create state and set opt vars
            x = zeros(Obj.Params.DimState,1);
            x(Obj.Params.VarsOpt) = x_opt;
    
            % cycle over trajectories
            for traj = 1:Obj.Params.Ntraj

                % set the trajectory
                Obj.Params.traj = traj;
                
                % non optimized vals - traj dependent
                x_nonopt = varargin{2}(traj).val;
                
                % add non optimized vars to state
                x(Obj.Params.VarsNonOpt) = x_nonopt;
                
                % update Params
                % remark: it is fundamental to update the parameters,
                % otherwise you compute the cost function with always the
                % same Theta values. See manual for more information.
                Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,x);
                
                % get desired trajectory
                y_Target = varargin{3}(traj).val;
                
                %%% integrate %%%
                % define time array
                tspan_pos = [Obj.Params.BackTimeIndex, nonzeros(Obj.Params.YSpace)'];
                tspan = Obj.T(tspan_pos(1):tspan_pos(end));  
                
                % initial condition
                x_start = x(1:Obj.Params.DimState); 

                % get evolution with input only if at least 2 instans are
                % considered
                if length(tspan)>1
                    X = Obj.Functions.Ode(@(t,x)Obj.Functions.Model(t, x, Obj.Params, Obj), tspan, x_start,Obj.Params.Odeset); 
                else
                    X.y = x_start;
                end
                
                % check for NaN or Inf
                NaN_Flag = find(isnan(X.y));
                if NaN_Flag
                    J_final = NaN;
                    break
                end
                
                % comment on that 
                Inf_Flag = isempty(isinf(X.y));
                Huge_Flag = isempty(X.y>1e10);
                if Inf_Flag || Huge_Flag
                    J_final = Inf;
                    break
                end                
               
                % get input
                u_in = [zeros(Obj.Params.DimInput,1), Obj.Uhat(traj).val];

                % get measure
                Yhat_tmp = Obj.Functions.Measure(X.y,Obj.Params,tspan,u_in(:,(tspan_pos(1):tspan_pos(end))),Obj);
                                                                                                  
                % init cost function for Y
                J = zeros(Obj.Params.JNterm,length(Obj.Params.DimOutCompare));

                % get target index
                Target_pos = find(Obj.Params.YSpace ~= 0);

                % get the J - cycle in the output dimensions
                for dim=1:Obj.Params.DimOut

                    % get the time vector: get the index, remove the first
                    % to shift to indexes starting from 1. 
                    tspan_vals = tspan_pos(2:end) - tspan_pos(1) + 1;

                    % get the actual targets from Y
                    Target_tmp = y_Target(dim,Target_pos);

                    % get the values of interest from Yhat
                    hat_tmp = Yhat_tmp(dim,tspan_vals);

                    % compute the difference
                    diff_var = Target_tmp-hat_tmp;

                    % make it column vector
                    diff_var = reshape(diff_var,numel(diff_var),1);

                    % compute the dy^T*dy
                    J(dim) = transpose(diff_var)*Obj.Params.OutputWeightsScaled(dim)*diff_var;
                end            

                % wrap in JY
                JY = sum(J);

                % terminal cost
                % get the value from the last optimization propagated
                x0 = Obj.Params.TempX0(Obj.Params.traj).val;    

                % difference with the current xopt
                xterm = x0(Obj.Params.TerminalStates)-x(Obj.Params.TerminalStates);

                % make it column vector
                xterm = reshape(xterm,length(Obj.Params.TerminalStates),1);

                % compute the arrival cost with the scaled weights
                J_terminal = transpose(xterm)*diag(Obj.Params.TerminalWeightsScaled(Obj.Params.TerminalStates))*xterm;
                                             
                % barrier term init
                J_barr = zeros(numel(Obj.Params.BoundsPos),1);

                % cycle over the bound positions
                for bound=1:length(Obj.Params.BoundsPos)
                    
                    % barrier - low
                    % get the lowest val in the evolution and check it
                    err_low = min(X.y(Obj.Params.BoundsPos(bound),2:end) - Obj.Params.BoundsValLow(bound));
                    
                    % barrier - up
                    % get the highest val in the evolution and check it
                    err_up = max(Obj.Params.BoundsValUp(bound) - X.y(Obj.Params.BoundsPos(bound),2:end));
                    
                    % sum stuff
                    if (err_low > 0) && (err_up > 0)
                        J_barr(bound) = 0;
                    else
                        J_barr(bound) = 1e5;
                    end
                        
                end
                                
                % sum all contribution
                J_final = J_final + JY + sum(J_barr) + J_terminal;
                
            end
            
             % store J
             Obj.Params.JStory(Obj.Params.OptIterVal,Obj.Params.ActualTimeIndex) = J_final;
        end
        
        % Target function (observer or control design)
        function Obj = Target(Obj)          
            for i=1:Obj.Params.Ntraj
                Obj.Params.Target(i).val = Obj.Params.Ybuffer(i).val;
            end
        end               
        
        % observer function: this method wraps up all the afromentioned
        % methods and actually implements the observer. Check the reference
        % for more information. 
        function Obj = observer(Obj,xhat,y)
            
            % extract the estimated state from the provided struct
            for traj=1:Obj.Params.Ntraj

                % get current trajectory
                Obj.Params.traj = traj;

                % extract value
                xhat_tmp(traj).val = xhat(traj).val(:,Obj.Params.ActualTimeIndex);
            end

            % store in xhat (used from now on)
            xhat = xhat_tmp;
            
            %%% get estimated measures
            for traj=1:Obj.Params.Ntraj

                % set trajectory
                Obj.Params.traj = traj;

                % get ESTIMATED measure from ESTIMATED state (xhat)
                % remark: here "max(1,Obj.Params.ActualTimeIndex-1)" is
                % just to avoid to access element 0 of the array. 
                yhat(traj).val = Obj.Functions.Measure(xhat(traj).val,Obj.Params,Obj.T(Obj.Params.ActualTimeIndex),Obj.Uhat(traj).val(:,max(1,Obj.Params.ActualTimeIndex-1)),Obj);

                % save measure
                Obj.Yhat(traj).val(:,Obj.Params.ActualTimeIndex) = yhat(traj).val;
            end
            
            %%% get measure
            for traj=1:Obj.Params.Ntraj

                % set trajectory
                Obj.Params.traj = traj;

                % get measure
                Obj.Y(traj).val(:,Obj.Params.ActualTimeIndex) = y(traj).val;                 
                
            end                   

            % fisrt bunch of data - read Y every Nts and check if the signal is
            distance = Obj.Params.ActualTimeIndex-Obj.Params.YSpace(end);   

            % here add some guideline
            NtsPos = mod(Obj.Params.Nsaved,Obj.Params.N)+1;
            Obj.Params.NtsPos = NtsPos;

            %%%% observer %%%%
            if  (distance == Obj.Params.NtsVal(NtsPos))
               
                % OUTPUT measurements - buffer of N elements

                % measure
                for traj=1:Obj.Params.Ntraj

                    % set trajectory 
                    Obj.Params.traj = traj;

                    % shift buffer
                    Obj.Params.Ybuffer(traj).val(:,1:end-1) = Obj.Params.Ybuffer(traj).val(:,2:end);
                    Obj.Params.Ybuffer(traj).val(:,end) = y(traj).val;
                end

                % adaptive sampling                
                Obj.Params.YSpace(1:end-1) = Obj.Params.YSpace(2:end);
                Obj.Params.YSpace(end) = Obj.Params.ActualTimeIndex;
                Obj.Params.YSpaceFullStory(end+1) = Obj.Params.ActualTimeIndex;
                
                % i don't really know what's this for
                Obj.Params.Nsaved = Obj.Params.Nsaved + 1;
                
                % store measure times
                Obj.Params.SampleTime = [Obj.Params.SampleTime Obj.Params.ActualTimeIndex];

                % check only on the first traj as the sampling is coherent
                % on the 2.             
                % WaitAlBuffer == 1 --> start when all the buffer is full
                cols_nonzeros = length(find(Obj.Params.YSpace ~= 0));  

                % comment on this (waitbuffer)
                flag = Obj.Params.N;
                
                % real
                if cols_nonzeros >= flag      

                    % get the maximum distance (???)
                    first_nonzero = find(Obj.Params.YSpace,1,'first');
                    Y_space_nonzero = Obj.Params.YSpace(first_nonzero:end);
                    max_dist = max(diff(Y_space_nonzero));
                    if isempty(max_dist)
                        max_dist = 1;
                    end
                        
                    % setup buffers
                    n_samples = min(length(Obj.Params.YSpaceFullStory)-1,Obj.Params.N);
                    buf_Y_space_full_story = Obj.Params.YSpaceFullStory(end-n_samples:end);
                        
                    % back time index
                    buf_dist = diff(buf_Y_space_full_story);
                    Obj.Params.BackTimeIndex = max(Obj.Params.ActualTimeIndex-sum(buf_dist(1:end)),1); 
                    Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);
                   
                    % set of initial conditions
                    for traj=1:Obj.Params.Ntraj

                        % set trajetory 
                        Obj.Params.traj = traj;

                        % start from the correct one                             
                        Obj.Params.TempX0NonOpt(traj).val = Obj.Xhat(traj).val(Obj.Params.VarsNonOpt,Obj.Params.BackTimeIndex);

                        % change only the values which are to be optimized
                        % only 1 set of vars regardless to the number
                        % of trajectories used as we're not estimating
                        % the state or the model parameters
                        Obj.Params.TempX0Opt = Obj.Xhat(1).val(Obj.Params.VarsOpt,Obj.Params.BackTimeIndex);
                            
                        % reconstruct temp_x0 from opt/nonopt vars
                        Obj.Params.TempX0(traj).val = zeros(Obj.Params.DimState,1);
                        Obj.Params.TempX0(traj).val(Obj.Params.VarsOpt) = Obj.Params.TempX0Opt;
                        Obj.Params.TempX0(traj).val(Obj.Params.VarsNonOpt) = Obj.Params.TempX0NonOpt(traj).val;

                    end
                        
                    % set Target
                    Obj = Obj.Target();
                    for traj = 1:Obj.Params.Ntraj
                    
                        % set trajectory
                        Obj.Params.traj = traj;
                        nonzero_space = find(Obj.Params.YSpace ~= 0);
                        nonzero_pos = Obj.Params.YSpace(nonzero_space);
                        Obj.Params.TargetStory(traj).val(:,nonzero_pos) = Obj.Params.Target(traj).val(:,nonzero_space);

                    end
                        
                    %%% normalisation %%%
                    if (Obj.Params.JNormalize) && (~Obj.Params.Normalized) 

                        % get the time index until when you want to
                        % normalize. A buffer should work. 
                        range = 1:Obj.Params.ActualTimeIndex;
                        
                        % cycle over all the Y used for the cost function
                        for dim=1:length(Obj.Params.DimOutCompare)

                            % init max variable
                            Emax = zeros(Obj.Params.Ntraj);

                            % cycle over the trajectories
                            for traj=1:Obj.Params.Ntraj

                                % evaliate the output mismatch using the
                                % current measurements and Y estimation
                                % remark: if you have perfect measurements,
                                % this mismatch is zero. (noise free scenario)
                                E = (Obj.Yhat(traj).val(Obj.Params.DimOutCompare(dim),range) - ...
                                     Obj.Y(traj).val(Obj.Params.DimOutCompare(dim),range)).^2;

                                % get max
                                Emax(traj) = max(E);

                                % handle the case in which the max is too
                                % small. This would cause numerical errors
                                % as we will see in the next lines.
                                if abs(Emax(traj)) < 1e-15
                                    Emax(traj) = 1;
                                end                                        
                            end

                            % get the max among all the trajectories
                            Emax = max(Emax);

                            % rescale OutputWeights depending on the Emax
                            % value. Here you see that we can't have Emax
                            % zero, as previously remarked. 
                            Obj.Params.OutputWeightsScaled(Obj.Params.DimOutCompare(dim)) = Obj.Params.OutputWeights(Obj.Params.DimOutCompare(dim))/Emax;
                        end     
                            
                        % now we deal with a similar thing but for the
                        % terminal weights.

                        % get the norm of the terminal states in the range
                        E = vecnorm(Obj.Xhat(traj).val(Obj.Params.TerminalStates,range)',2,1);
                    
                        % cycle over the terminal states
                        for dim=1:length(Obj.Params.TerminalStates)  

                            % if the norm of that state in the range is not
                            % zero, scale it. Otherwise, keep it to 1. Same
                            % as before for the numerical errors having
                            % E(dim) zero.
                            if E(dim) ~= 0
                                Obj.Params.TerminalWeightsScaled(dim) = Obj.Params.TerminalWeights(dim)/E(dim);
                            else
                                Obj.Params.TerminalWeightsScaled(dim) = 1;
                            end
                        end    
                            
                        % the normalization is done only once in the whole
                        % simulation.
                        Obj.Params.Normalized = 1;
                        
                    end
                  
                    % let's proceed with the optimization
                    if (Obj.Params.Optimize)

                        % reset OptIterVal
                        Obj.Params.OptIterVal = 0;
                            
                        % save J before the optimization to confront it later 
                        [J_before, Obj_tmp] = Obj.cost_function(Obj.Params.TempX0Opt,Obj.Params.TempX0NonOpt,Obj.Params.Target);


                        % Optimization (only if distance_safe_flag == 1)
                        opt_time = tic;                        

                        %%%%% OPTIMISATION - NORMAL MODE %%%%%%                            
                        % run optimization
                                                                                            
                        % create problem
                        try
                            % create a problem for fminsearchcon
                            % remark: the structure is the same as fmincon
                            % without the inequality constraints.
                            % createOptimProblem does not accept
                            % fminsearchon (it's from FileExchange) so this
                            % is the only workaround I came up with 
                            problem = createOptimProblem('fmincon', ...
                                                         'objective',       @(x)Obj.cost_function(x,Obj.Params.TempX0NonOpt,Obj.Params.Target), ...
                                                         'x0',              Obj.Params.TempX0Opt, ...
                                                         'lb',              Obj.Params.LBcon, ...
                                                         'ub',              Obj.Params.UBcon, ...
                                                         'Aeq',             Obj.Params.AconEq, ...
                                                         'beq',             Obj.Params.BconEq, ...
                                                         'nonlcon',         @(x)Obj.NONCOLcon(x,Obj), ...
                                                         'options',         Obj.Params.Myoptioptions);

                            % no inequality constraints
                            problem = rmfield(problem,'bineq');
                            problem = rmfield(problem,'Aineq'); 

                            % set the 
                            problem.options = Obj.Params.Myoptioptions;
                            problem.solver = 'fminsearchcon';
                        catch ME
                            disp(ME);
                            error('WRONG OPTIMISATION SETUP');
                        end 
                                
                        [NewXopt, J] = Obj.Functions.Fmin(problem.objective, ...
                                                          problem.x0, ...
                                                          problem.lb, ...
                                                          problem.ub, ...
                                                          problem.Aeq, ...
                                                          problem.beq, ...
                                                          problem.nonlcon, ...
                                                          problem.options ...
                                                          );

                        % comment on this
                        Obj.Params.JStory(Obj.Params.OptIterVal:end,Obj.Params.ActualTimeIndex) = J;
                        
                        % reconstruct NewXopt from opt/nonopt vars
                        NewXopt_tmp = [];

                        % cycle over trajectories
                        for traj = 1:Obj.Params.Ntraj

                            % set trajectory
                            Obj.Params.traj = traj;

                            % stack vector
                            NewXopt_end = zeros(Obj.Params.DimState,1);
                            NewXopt_end(Obj.Params.VarsOpt) = NewXopt;
                            NewXopt_end(Obj.Params.VarsNonOpt) = Obj.Params.TempX0NonOpt(traj).val;                          
                            NewXopt_tmp(traj).val = NewXopt_end;  

                        end
                            
                        % set new state
                        NewXopt = NewXopt_tmp;                                                

                        % optimization counter
                        if traj == 1
                            Obj.Params.OptCounter = Obj.Params.OptCounter + 1;
                            Obj.Params.OptTime = [Obj.Params.OptTime Obj.Params.ActualTimeIndex];
                        end

                        % check J_dot condition
                        J_diff = (J/J_before);

                        if (J_diff <= Obj.Params.JdotThresh)                                                     

                            % on each trajectory
                            for traj=1:Obj.Params.Ntraj

                                % set traj
                                Obj.Params.traj = traj;

                                % update state
                                Obj.Xhat(traj).val(:,Obj.Params.BackTimeIndex) = NewXopt(traj).val;

                                % store measure times
                                if traj == 1
                                    Obj.Params.SelectCounter = Obj.Params.SelectCounter + 1;
                                    Obj.Params.SelectTime = [Obj.Params.SelectTime Obj.Params.ActualTimeIndex];
                                end

                                % state to repropagate 
                                x_propagate = NewXopt(traj).val;

                                % update Params
                                Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,x_propagate);

                                % FIRST MEASURE UPDATE 
                                % manage measurements
                                Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);

                                % ESTIMATED measurements
                                % measures       
                                % NB: the output storage has to be done in
                                % Obj.Params.BackTimeIndex+1 as the propagation has been
                                % performed 
                                Yhat_meas = Obj.Functions.Measure(x_propagate,Obj.Params,Obj.Params.BackTime,Obj.Uhat(traj).val(:,Obj.Params.BackTimeIndex),Obj);

                                % set value
                                Obj.Yhat(traj).val(:,Obj.Params.BackTimeIndex) = Yhat_meas;
                                    
                                % PROPAGATION 
                                n_iter_propagate = Obj.Params.ActualTimeIndex-Obj.Params.BackTimeIndex;
    
                                % cycle over the propagation range
                                for j=1:n_iter_propagate
                                            
                                    % back time
                                    Obj.Params.BackTimeIndex = Obj.Params.BackTimeIndex+1;

                                    % get time interval
                                    tspan = Obj.T(Obj.Params.BackTimeIndex-1:Obj.Params.BackTimeIndex);                                                                                                                     

                                    % integrate
                                    Xtmp = Obj.Functions.Ode(@(t,x)Obj.Params.Model(t, x, Obj.Params, Obj), tspan, x_propagate,Obj.Params.Odeset);                                    
                                    x_propagate = Xtmp.y(:,end);                      

                                    % set the evolution
                                    Obj.Xhat(traj).val(:,Obj.Params.BackTimeIndex) = x_propagate;
                                        
                                    % ESTIMATED measurements
                                    % measures       
                                    % NB: the output storage has to be done in
                                    % Obj.Params.BackTimeIndex+1 as the propagation has been
                                    % performed 
                                    Yhat_tmp = Obj.Params.Measure(x_propagate,Obj.Params,Obj.Params.BackTime,Obj.Uhat(traj).val(:,Obj.Params.BackTimeIndex-1),Obj);
                                        
                                    % get filters - yhat
                                    Obj.Yhat(traj).val(:,Obj.Params.BackTimeIndex) = Yhat_tmp;            
                                        
                                end
                             end
                        else
                                
                            % on each trajectory
                            for traj=1:Obj.Params.Ntraj
                                Obj.Params.traj = traj;
                                % keep the initial guess
                                Obj.Xhat(traj).val(Obj.Params.VarsOpt,Obj.Params.BackTimeIndex) = Obj.Params.TempX0Opt;
                            end
                                
                            % restore Params
                                
                            Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,Obj.Params.temp_x0(traj).val);                                                        
                        end

                        % stop time counter
                        Obj.Params.ExecutionTime(end+1) = toc(opt_time);
                    end
                end

                % clear the screen 
                if Obj.Params.Print
                    clc;
                end 
            else

                % keep the last J
                Obj.Params.JStory(:,Obj.Params.ActualTimeIndex) = Obj.Params.JStory(:,max(1,Obj.Params.ActualTimeIndex-1));

            end
        end
    end
end
