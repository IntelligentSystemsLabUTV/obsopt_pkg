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

        % obsopt: class constructor. Each of the class variables can be 
        % specifically set by calling the constructor with specific 
        % properties, i.e. by properly using the varargin parameter. Some 
        % of then are mandatory, others are optional. In this case a 
        % default value is used
        % INPUT: 
        %           varargin:   all the options as in simulation_general
        % OUTPUT:
        %           Obj: obsopt class (self)
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

                % get params update handle function from Params
                Obj.Functions.ParamsUpdate = Params.ParamsUpdate;
                
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
             
            % the class implements an MHE observer or a TBOD method. One
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
                warning('No "optimize" option provided: set default to: false');
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
            % MATLAB Exchange. You might want to change this. In this case 
            % a relevant rewriting of the obsopt class might be necessary. 
            % We have already done this on other branches of the repository
            % but the whole thing might be quite messy. So, feel free to
            % contact us for help. 
            Obj.Functions.Fmin = @fminsearchcon;
            
            % The he BounPos option allows the user to add a barrier
            % function term in the MHE cost function. This term can be used
            % to enforce upper and lower bounds on any state variable. Note
            % that, differently from fminsearchcon, the bounds can be
            % enforced also on state variables which are not optimization
            % variables. This comes in handy when you have, for instance,
            % constraints on parameters (see battery estimation). 
            % We suggest to use the fminsearchcon option to bound the
            % optimization variables, and the BounPos option to constrain
            % the rest of the state variables.

            % if present in the options, BoundPos is found. BoundPos shall
            % be a vector specifying which state variables the user wants
            % to constrain. For instance, if we have constraints on the 1st
            % and 4th state variable, BoundPos will be [1 4].
            if any(strcmp(varargin,'BoundsPos'))

                % find BoundPos position
                pos = find(strcmp(varargin,'BoundsPos'));

                % assign the value
                Obj.Params.BoundsPos = varargin{pos+1};     

            else

                % BoundPos is not mandatory. If not specificed, it is set
                % to "no bounds", namely an empty vector
                warning('No "BoundsPos" option provided: set default to: [].');
                Obj.Params.BoundsPos = [];

            end            
            
            % If BoundPos is specified, BoundsValLow is a vector with the
            % lower bound in the constraints. It shall agree with the
            % dimension of BoundsPos. Considering the case of before,
            % BoundsValLow here could be [-20.4 100.3]
            if any(strcmp(varargin,'BoundsValLow'))

                % find BoundsValLow position
                pos = find(strcmp(varargin,'BoundsValLow'));

                % assign the value
                Obj.Params.BoundsValLow = varargin{pos+1};                

            else

                % If not specified, same approach with BoundsPos, namely
                % empty vector
                warning('No "BoundsValLow" option provided: set default to: [].');
                Obj.Params.BoundsValLow = [];

            end
            
            % Same as BoundsValUp but with upper bounds, e.g., [45.2 1000]
            if any(strcmp(varargin,'BoundsValUp'))

                % find BoundsValUp position
                pos = find(strcmp(varargin,'BoundsValUp'));

                % assign the value
                Obj.Params.boundsValUp = varargin{pos+1};                
            else

                % If not specified, same approach with BoundsPos, namely
                % empty vector
                warning('No "BoundsValUp" option provided: set default to: [].');
                Obj.Params.BoundsValUp = [];

            end
                        
            % In this section all the constraints used by fminsearchcon are
            % assigned. See fminsearchcon reference for more information.
            % If none of them is specified, all are initialized as empty
            % vectors

            % Inequality constraint (A)
            if any(strcmp(varargin,'Acon'))

                % Find position
                pos = find(strcmp(varargin,'Acon'));

                % assign the value
                Obj.Params.Acon = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "Acon" option provided: set default to: [].');
                Obj.Params.Acon = [];                    

            end

            % Inequality constraint (b)
            if any(strcmp(varargin,'Bcon'))

                % Find position
                pos = find(strcmp(varargin,'Bcon'));

                % assign the value
                Obj.Params.Bcon = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "Bcon" option provided: set default to: [].');
                Obj.Params.Bcon = [];     

            end

            % Equality constraint (A)
            if any(strcmp(varargin,'AconEq'))

                % find position
                pos = find(strcmp(varargin,'AconEq'));

                % ssign the value
                Obj.Params.AconEq = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "AconEq" option provided: set default to: [].');
                Obj.Params.AconEq = [];    

            end

            % Equality constraint (b)
            if any(strcmp(varargin,'BconEq'))

                % find position
                pos = find(strcmp(varargin,'BconEq'));
                Obj.Params.BconEq = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "BconEq" option provided: set default to: [].');
                Obj.Params.BconEq = [];                

            end

            % Lower bounds on the optimization variables
            if any(strcmp(varargin,'LBcon'))

                % find position
                pos = find(strcmp(varargin,'LBcon'));

                % assign the value
                Obj.Params.LBcon = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "LBcon" option provided: set default to: -Inf.');
                Obj.Params.LBcon = -Inf*ones(1,length(Obj.Params.VarsOpt));  

            end

            % Upper bounds on the optimization variables
            if any(strcmp(varargin,'UBcon'))

                % find position
                pos = find(strcmp(varargin,'UBcon'));
                Obj.Params.UBcon = varargin{pos+1};

            else

                % If not specified, initialize to empty vector
                warning('No "UBcon" option provided: set default to: Inf.');
                Obj.Params.UBcon = Inf*ones(1,length(Obj.Params.VarsOpt));          

            end

            % general nonlìnear constraint function. This is a handle, see
            % ref from fminsearchcon for more info
            if any(strcmp(varargin,'NONCOLcon'))

                % find position
                pos = find(strcmp(varargin,'NONCOLcon'));

                % assign the handle
                Obj.Functions.NONCOLcon = varargin{pos+1}; 

            else

                % If not specified, initialize to the method fro the class.
                % Such a method returns zero constraints by default. In
                % case the user wants to add nonlinear constraints we
                % suggest to code them in an external function, as they
                % could greatly vary from a test case to another. You don't
                % want leftovers from previous projects within the obsopt
                % class itself. 
                warning('No "NONCOLcon" option provided: set default to @Obj.NONCOLcon.');
                Obj.Functions.NONCOLcon = @Obj.NONCOLcon;
            end
            
            % number of terms in the cost function, without considering the
            % arrival cost. It is used for storage. it is set to 1 as
            % currently only the output mismatch is considered. Future
            % developments might increase this value
            Obj.Params.JNterm = 1; 

            % Output weights from the Params file. These will be rescaled
            % by the normalization of the cost function, if enabled. 
            % Remark: if you want to manually set the cost function weights
            % specify the "OutputWeights" matrices in the "Params" file.
            try
                
                % get the values from Params
                Obj.Params.OutputWeights = Params.OutputWeights;

                % initialize the scaled values to the original
                % OutputWeights, they will be reassigned later
                Obj.Params.OutputWeightsScaled = Obj.Params.OutputWeights;

            catch

                % OutputWeights are mandatory
                error('No terminal weights found in "Params"');

            end
            
            % Terminal weights from the Params file. These will be rescaled
            % by the normalization of the cost function, if enabled. 
            % Remark: if you want to manually set the cost function weights
            % specify the "TerminalWeights" matrices in the "Params" file.
            try
                
                % get the values from Params
                Obj.Params.TerminalWeights = Params.TerminalWeights;

                % initialize the scaled values to the original
                % TerminalWeights, they will be reassigned later
                Obj.Params.TerminalWeightsScaled = Obj.Params.TerminalWeights;

            catch

                % TerminalWeights are mandatory
                error('No terminal weights found in "Params"');

            end
            
            % number of reference trajectories (see ref for TBOD)          
            Obj.Params.Ntraj = Params.Ntraj;

            % trajectory counter, initialized to 1
            Obj.Params.Traj = 1;

            % In the MHE the measurement buffer is updated every time a new
            % sample is available. In the standard version of the MHE, each
            % sample is spaced from the previous one by NTs samples. This
            % is not true anymore if the buffer is not equally spaced, as
            % it has been proposed in battery_estimation. For isntance,
            % assume that we are considering a N=4 buffer, with Nts
            % distribution [3 7 7 3]. In this case, before sampling a new
            % measurement, we need to wait either 3 or 7 samples depending
            % on which element of the buffer we are filling. Thus, we need
            % to keep track of where we are in the buffer filling. Nsaved
            % and NTsPos keep track of this, as it will be als recalled 
            % later in the code. 
            Obj.Params.Nsaved = 0;
            Obj.Params.NTsPos = 1;
            
            % this variable is a flag to check wether the cost function
            % normalization has already been done or not. Clearly, the
            % normalization shall be done only once, before the first
            % optimization, because the cost function weights must remain
            % the same for the entire simulation.
            Obj.Params.Normalized = 0;
            
            % BackTimeIndex: position in the time vector of the first
            % sample present in the buffer. It is used to know from where
            % start the model integration (k-N+1) in the ref
            Obj.Params.BackTimeIndex = 1; 

            % related time instant value from the time vector
            Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);

            % ActualTimeIndex: position of the current time instant in the
            % time vector (k) in ref
            Obj.Params.ActualTimeIndex = 1;

            % related time instant from the time vector
            Obj.Params.ActualTime = Obj.T(Obj.Params.ActualTimeIndex);
            
            % measure buffer (Y,Yhat): these buffers are used to store the 
            % measuredand estimated values on the observed states. There is 
            % a different buffer for each trajectory, mainly for the TBOD 
            % control buffer (U,Uhat): these buffers are used to store the
            % control signals. again, one buffer for each trajectory.
            for i=1:Obj.Params.Ntraj

                % output
                Obj.Y(i).val =  zeros(Obj.Params.DimOut,Obj.Params.N);
                Obj.Yhat(i).val = zeros(Obj.Params.DimOut,0);
                
                % control
                Obj.U(i).val(:,1) = zeros(Obj.Params.DimInput,1);
                Obj.Uhat(i).val(:,1) = zeros(Obj.Params.DimInput,1);

            end
            
            % this vector keeps track of the time indices when the output
            % has been sampled. In the standard MHE all these vaues will be
            % NTs samples distant. In the Adaptive and MultiScale MHE this
            % is in general not true (see ref)
            Obj.Params.YSpace = zeros(1,Obj.Params.N);
            
            % measurement buffer (Ybuffer): contains the actual
            % measurements sampled at the indices stored in Yspace.
            for i=1:Obj.Params.Ntraj

                % initialize the array
                Obj.Params.Ybuffer(i).val = zeros(Obj.Params.DimOut,Obj.Params.N);

            end

            % these variables are used to store the optimization values at
            % the beginning of the optimization process. This is done in
            % case that mid-steps or restore actions were needed.

            % cycle over the trajectories
            for i=1:Obj.Params.Ntraj

                % initialize the arrays
                Obj.Params.TempX0NonOpt(i).val = [];
                Obj.Params.TempX0Opt(i).val = [];
                Obj.Params.TempX0(i).val = [];

            end
         
            % observer cost function init
            Obj.Params.J = 0;

            % cost function storage init. We wan to store the cost function
            % evolution in time and in iterations
            Obj.Params.JStory = zeros(Obj.Params.MaxIter,Obj.Params.Niter);

            % this variable keeps track of the current iteration in the
            % cost function
            Obj.Params.OptIterVal = 0;
            
            % here we store all the sampling indices. Used in the plot to
            % select the sampled values during all the simulation.
            Obj.Params.SampleTime = [];

            % this variable colects all the time indices when an
            % optimization is run and accepted according to JDotThresh
            Obj.Params.OptTime = [];
            Obj.Params.SelectTime = [];

            % OptCounter: counts how many optimization are run
            Obj.Params.OptCounter = 0;

            % SelectCounter: counts how many optimization are accepted
            % following JDotThresh
            Obj.Params.SelectCounter = 0;
            
            % IterTime: used in simulation_general to get the execution
            % time of a single observer call
            Obj.Params.IterTime = [];

            % TotalTime: used in simulation_general to get the execution
            % time of the entire simulation
            Obj.Params.TotalTime = [];
            
            % here we start the optimization setup. Refer to optimset for
            % more details

            % tolerance on the minimum distance between the optimization
            % variables and cost function values. 
            Obj.Params.TolX = 0;
            Obj.Params.TolFun = 0;
            
            % set options for print depending on the Print option in the
            % initialization.
            if Obj.Params.Print 

                % display the iterations
                Obj.Params.Display = 'iter';

            else

                % do not display anything
                Obj.Params.Display = 'off'; 

            end
    
            
            % call the optimset function with the related values                    
            Obj.Params.Myoptioptions = optimset('MaxIter',              Obj.Params.MaxIter, ...     % max iterations 
                                                'display',              Obj.Params.Display, ...     % print options
                                                'MaxFunEvals',          Inf, ...                    % maximum cost function evaluations (feval)
                                                'OutputFcn',            @Obj.outfun, ...            % function to decide wether or not stop the optimization
                                                'TolFun',               Obj.Params.TolFun, ...      % tolerance on cost function improvement
                                                'TolX',                 Obj.Params.TolX, ...        % tolerance on optimization variables distance
                                                'TolCon',               1e-10 ...                   % tolerance on the constraints
                                                );
        end


        % NONCOLcon: method for the nonlinear constraints. 
        % It is used as a backup when no external function is provided in 
        % the constructor. It returns zero as it describes a problem 
        % without nonlinear constraints
        % INPUT: 
        %           varargin:   in general this is decided depending on the
        %                       test case. Here there are no inputs because
        %                       it returns zero
        % OUTPUT:
        %           c: inequality constraints identity outcome
        %           ceq: equality constraints identity outcome
        function [c, ceq] = NONCOLcon(varargin)

            % inequality constraints
            c = 0;

            % equality constraints
            ceq = 0;

        end
        
        % outfun: this method check wether or not the optimization process
        % shall be stopped or not. Specifically, it stops the optimization
        % when the maximum number of optimizations are reached or when the
        % tolerance on the cost function improvement is violated.
        % INPUT: 
        %           Obj: obsopt class (self)
        %           x: current optimization variables
        %           optimValues: cost function value
        %           state: current optimization state (idle, init, ...)
        %           check on matlab fmincon page
        % OUTPUT:
        %           stop: boolean to stop or not the optimization
        function stop = outfun(Obj,x, optimValues, state)

            % check on iterations and cost function tolerance
            if (optimValues.iteration == Obj.Params.MaxIter) || (optimValues.fval <= Obj.Params.TolFun)

                % stop the optimization
                stop = true;

            else

                % continue with the optimization
                stop = false;

            end

        end
        
        
        % cost function: this method computes the objective to be minimized 
        % by the MHE observer and the TBOD method
        % INPUT: 
        %           Obj: obsopt class (self)
        %           varargin:
        %                       x_opt: optimization variables
        %                       x_nonopt: non optimized variables
        %                       y_Target: measurement buffer used for the
        %                       output mismatch.
        % OUTPUT:
        %           J_final: cost function
        %           Obj: obsopt class (self)
        function [J_final,Obj] = cost_function(Obj,varargin) 

            % init the cost function
            J_final = 0;

            % update the iteration value
            Obj.Params.OptIterVal = Obj.Params.OptIterVal + 1;

            % get the part of the state vector building the optimization 
            % variables. These are the same for all the trajectories
            x_opt = varargin{1};

            % initialize the state vector and assign the opt vars values
            x = zeros(Obj.Params.DimState,1);
            x(Obj.Params.VarsOpt) = x_opt;
    
            % cycle over trajectories to build the initial condition for
            % each and any of them
            for traj = 1:Obj.Params.Ntraj

                % set the trajectory
                Obj.Params.traj = traj;
                
                % get non optimized variables - trajectory dependent
                x_nonopt = varargin{2}(traj).val;
                
                % add non optimized vars to state
                x(Obj.Params.VarsNonOpt) = x_nonopt;
                
                % update Params
                % remark: it is fundamental to update the parameters,
                % otherwise you compute the cost function with always the
                % same Theta values. See manual for more information.
                Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,x);
                
                % get desired measurement values from each trajectory
                y_Target = varargin{3}(traj).val;
                
                % model integration
                % define the time window [k-N+1;k]
                % both indices and time instants
                tspan_pos = [Obj.Params.BackTimeIndex, nonzeros(Obj.Params.YSpace)'];
                tspan = Obj.T(tspan_pos(1):tspan_pos(end));  
                
                % set the initial condition
                x_start = x(1:Obj.Params.DimState); 

                % get the model evolution with input. Note that the 
                % integration is called only if at least 2 instans are
                % considered in the time_span. 
                if length(tspan)>1

                    % integrate
                    Xtmp = Obj.Functions.Ode(@(t,x)Obj.Functions.Model(t, x, Obj.Params, Obj), tspan, x_start, Obj.Params.Odeset); 

                else

                    % keep the initial condition
                    Xtmp.y = x_start;
                end
                
                % there might be numerical instabilities during the 
                % integration. this could happen because the optimization 
                % problem is not well defned (which is bad) or because the 
                % optimizer is exploring values which are not feasible
                % (with fminserchcon this could happen because it uses a
                % simplex algorithm). here we check for NaN or Inf values
                % in the integration.
                NaN_Flag = find(isnan(Xtmp.y));

                % if NaN found, exit from the optimization
                if NaN_Flag

                    % the cost function is assigned to be NaN
                    J_final = NaN;
                    break

                end
                
                % Here we consider as Infinity both an actual Inf value for
                % the cost function but also a value which is clearly out
                % of scope. We hard-coded this value to be 1E10.
                Inf_Flag = isempty(isinf(Xtmp.y));      % Infinity value
                Huge_Flag = isempty(Xtmp.y>1e10);       % Huge value

                % If any of that is true, break and set the J to inf
                if Inf_Flag || Huge_Flag

                    % set cost function and exit
                    J_final = Inf;
                    break

                end                
               
                % if everything goes fine, we can proceed

                % get input from the storage arrays. The input might be
                % used in the output mapping if there is an algebraic
                % relation between y and u (D matrix in a linear framewirk,
                % for instance). In the scenarios considered so far this is
                % never happening, so the next line is in fact useless. We
                % suggest to leave it here for future developements.
                u_in = [zeros(Obj.Params.DimInput,1), Obj.Uhat(traj).val];

                % get measure from the state trajectory 
                Yhat_tmp = Obj.Functions.Measure(Xtmp.y,Obj.Params,tspan,u_in(:,(tspan_pos(1):tspan_pos(end))),Obj);
                
                % get the time vector: get the indices, remove the first
                % to shift the whole thing and have the indexes 
                % starting from 1. 
                tspan_vals = tspan_pos(2:end) - tspan_pos(1) + 1;

                % initialize cost function
                J = zeros(Obj.Params.JNterm,numel(tspan_vals));

                % compute the J. Cycle all the time interval and compute
                % the output mismatch.
                for timePos=1:numel(tspan_vals)

                    % get the actual targets from Y
                    Target_tmp = y_Target(Obj.Params.DimOutCompare,timePos);

                    % get the values of interest from Yhat
                    hat_tmp = Yhat_tmp(Obj.Params.DimOutCompare,tspan_vals(timePos));

                    % compute the difference
                    diff_var = Target_tmp-hat_tmp;

                    % make it column vector
                    diff_var = reshape(diff_var,numel(diff_var),1);

                    % compute the dy^T*W*dy using the OutputWeights of the
                    % specific dimension
                    J(timePos) = transpose(diff_var)*diag(Obj.Params.OutputWeightsScaled(Obj.Params.DimOutCompare))*diff_var;

                end            

                % wrap in JY: sum all the J values 
                JY = sum(J);

                % arrival cost: get the value from the last optimization
                x0 = Obj.Params.TempX0(Obj.Params.traj).val;    

                % difference with the current optimization variables
                xterm = x0(Obj.Params.TerminalStates)-x(Obj.Params.TerminalStates);

                % make it column vector
                xterm = reshape(xterm,length(Obj.Params.TerminalStates),1);

                % compute the arrival cost with the scaled weights
                J_terminal = transpose(xterm)*diag(Obj.Params.TerminalWeightsScaled(Obj.Params.TerminalStates))*xterm;
                                             
                % barrier term initialization
                J_barr = zeros(numel(Obj.Params.BoundsPos),1);

                % cycle over the bound positions
                for bound=1:length(Obj.Params.BoundsPos)
                    
                    % barrier - low
                    % get the lowest val in the evolution and check it
                    err_low = min(Xtmp.y(Obj.Params.BoundsPos(bound),2:end) - Obj.Params.BoundsValLow(bound));
                    
                    % barrier - up
                    % get the highest val in the evolution and check it
                    err_up = max(Obj.Params.BoundsValUp(bound) - Xtmp.y(Obj.Params.BoundsPos(bound),2:end));
                    
                    % if neither of these values is greater than zero then
                    % it means that the bounds are respected. In this case,
                    % the barrier function is zero. Otherwise, we set the
                    % barrier to 1E5, which is still lower than the
                    % Infinity check of before (1E10)
                    if (err_low > 0) && (err_up > 0)

                        % set cost function 
                        J_barr(bound) = 0;

                    else

                        % set cost function
                        J_barr(bound) = 1e5;

                    end
                        
                end
                                
                % sum all contribution into a final cost function 
                J_final = J_final + JY + sum(J_barr) + J_terminal;
                
            end
            
             % store J
             Obj.Params.JStory(Obj.Params.OptIterVal,Obj.Params.ActualTimeIndex) = J_final;
        end
        
        % observer function: this method implements a single call to the
        % observer. Check ref for more information 
        % INPUT: 
        %           Obj: obsopt class (self)
        %           xhat: the estimated plant trajectory
        %           y: the last measured output from the nominal plant
        % OUTPUT:
        %           Obj: obsopt class (self)
        function Obj = observer(Obj,xhat,y)

            % the function basically follows these steps:
            %   1:  unwrap/setup the state/measurements
            %   2:  decide whether or not to sample the measurement
            %       2.a:    if so store in the buffer
            %       2.b:    if needed, normalize the cost function
            %       2.c     decide whether to optimize
            %               2.c.a   if so optimize
            %               2.c.b   decide whether accept or not the new X
            %                       2.c.b.a if so, propagate the new X
            
            % STEP 1

            % cycle over the trajectories, and extract the estimated state 
            % from the provided struct
            for traj=1:Obj.Params.Ntraj

                % set current trajectory
                Obj.Params.traj = traj;

                % extract value
                xhat_tmp(traj).val = xhat(traj).val(:,Obj.Params.ActualTimeIndex);

            end

            % store in xhat (used from now on)
            xhat = xhat_tmp;
            
            % cycle over the trajectories, and get estimated measures
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
            
            % cycle over the trajectories, and get measure from 
            % teh nominal plant
            for traj=1:Obj.Params.Ntraj

                % set trajectory
                Obj.Params.traj = traj;

                % get measure
                Obj.Y(traj).val(:,Obj.Params.ActualTimeIndex) = y(traj).val;                 
                
            end                   

            % STEP 2

            % compute the sample-distance from the last sample
            distance = Obj.Params.ActualTimeIndex - Obj.Params.YSpace(end);   

            % As introduced in the constructor, if we have a buffer not
            % equally spaced, e.g., [3 7 7 3], we need to keep track where
            % we are in picking the next minimu distance between samples.
            % This is done by increasing any time the Nsaved counter (see
            % below) and then taking the module between nsaved and the
            % buffer length. That is the current index in the buffer,
            % namely NtsPos
            NtsPos = mod(Obj.Params.Nsaved,Obj.Params.N)+1;
            Obj.Params.NtsPos = NtsPos;

            % check if the down-sampling distance is met. if so, proceed
            if  (distance == Obj.Params.NtsVal(NtsPos))
               
                % STEP 2.a

                % here we increase the Nsaved counter, which will be used
                % as previously introduced
                Obj.Params.Nsaved = Obj.Params.Nsaved + 1;
                
                % Now we can store in the buffer
                for traj=1:Obj.Params.Ntraj

                    % set trajectory 
                    Obj.Params.traj = traj;

                    % shift buffer
                    Obj.Params.Ybuffer(traj).val(:,1:end-1) = Obj.Params.Ybuffer(traj).val(:,2:end);

                    % assign new measurement
                    Obj.Params.Ybuffer(traj).val(:,end) = y(traj).val;

                end

                % shift buffer positions memory                
                Obj.Params.YSpace(1:end-1) = Obj.Params.YSpace(2:end);

                % update buffer position memory
                Obj.Params.YSpace(end) = Obj.Params.ActualTimeIndex;
                
                % store measure times. See the constructor for more info on
                % SampleTime
                Obj.Params.SampleTime = [Obj.Params.SampleTime Obj.Params.ActualTimeIndex];

                % now we check if in the buffer we have stored enough data
                % to slve the oprimization problem. Refer to the ref for
                % the necessary condition. Independently from that we wait
                % until the entire buffer is filled before starting with
                % the optimization. So, if we have a buffer of 30 elements,
                % but the necessary condition for the MHE is N >= 10, we
                % don't care. We wait until we have 30 saples. As for the
                % TBOD, this approach is consistent, as we wait for the
                % whole trajectory to be sampled.
                cols_nonzeros = length(find(Obj.Params.YSpace ~= 0));  

                % indeed, this flag is how much we want to wait. It comes
                % without saying that if we wanted to wait for just 5
                % samples, we would set flag = 5. So, the user can try
                % different fill&wait strategies by changing this variable
                flag = Obj.Params.N;
                
                % we check the condition and if it is satisfied we proceed
                % with the following steps
                if cols_nonzeros >= flag      
    
                    % setup buffers
                    % here we count how many samples have been collected
                    % up to now. If flag = N before, this line is useless,
                    % but if the user is using a different wait&fill
                    % strategy, there might be less samples than N
                    n_samples = min(length(Obj.Params.SampleTime)-1,Obj.Params.N);

                    % colect the last n_samples time indices
                    buf_Y_space_full_story = Obj.Params.SampleTime(end-n_samples:end);
                        
                    % compute the sample-distances between all these last
                    % n_samples
                    buf_dist = diff(buf_Y_space_full_story);

                    % the current index minus the sum of all these
                    % differences gives us the BackTimeIndex. 
                    Obj.Params.BackTimeIndex = max(Obj.Params.ActualTimeIndex-sum(buf_dist(1:end)),1); 
                    Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);

                    % remark: one could say that instead of this last
                    % calculations one could just use the distances in
                    % NtsVal. This is true if the samplings are
                    % deterministic, but this might not be the case if the
                    % Adaptive MHE was used. Thus, we prefer to leave this
                    % approach to better adapt to future refinements of
                    % the code. 
                   
                    % set the initial conditions for the optimization

                    % optimization variabless. here we assign ro tempX0Opt
                    % only the values of the first trajectory because the
                    % optimization variables are the same for all the
                    % trajctories. Thus, initializing with the first or
                    % other trajectories makes no difference
                    Obj.Params.TempX0Opt = Obj.Xhat(1).val(Obj.Params.VarsOpt,Obj.Params.BackTimeIndex);

                    % cycle over the trajectories
                    for traj=1:Obj.Params.Ntraj

                        % set trajetory 
                        Obj.Params.traj = traj;

                        % start from the non optimied variables                            
                        Obj.Params.TempX0NonOpt(traj).val = Obj.Xhat(traj).val(Obj.Params.VarsNonOpt,Obj.Params.BackTimeIndex);
    
                        % reconstruct temp_x0 from opt/nonopt vars
                        Obj.Params.TempX0(traj).val = zeros(Obj.Params.DimState,1);
                        Obj.Params.TempX0(traj).val(Obj.Params.VarsOpt) = Obj.Params.TempX0Opt;
                        Obj.Params.TempX0(traj).val(Obj.Params.VarsNonOpt) = Obj.Params.TempX0NonOpt(traj).val;

                    end

                    % STEP 2.b
                        
                    % if setup in the options, do the one-time
                    % normalization of the cost function. The Normalized
                    % flag will be set to true once this process is done
                    if (Obj.Params.JNormalize) && (~Obj.Params.Normalized) 

                        % get the time index until when you want to
                        % normalize. Our default choice is up until the
                        % current index, which is a full measurement buffer
                        range = 1:Obj.Params.ActualTimeIndex;
                        
                        % cycle over all the output dimensions
                        for dim=1:length(Obj.Params.DimOutCompare)

                            % init max variable. we want to store here the
                            % maximum output mismatch value. 
                            % We will normalize over this maximum value,
                            % if it does not result zero of course
                            Emax = zeros(Obj.Params.Ntraj);

                            % cycle over the trajectories
                            for traj=1:Obj.Params.Ntraj

                                % evaluate the output mismatch using the
                                % current measurements and Y estimation
                                % remark: if you have perfect measurements,
                                % this mismatch is zero. (noise free scenario)
                                % we take the square because this is what
                                % is done in the cost function. remember
                                % that we are normalizing on the cost
                                % function elements, not on the simple
                                % output measurement.
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
                            
                        % now we do a similar process but for the
                        % terminal weights.

                        % get the norm of the terminal states in the range.
                        % We do this because the trajectories could swing
                        % from very different values in the time range.
                        % Thus, for each element, we take the norm over the
                        % entire range and we assume it as a good estimate
                        % of its order of magnitude. We use this estimation
                        % to copute the scaling factor for every state.
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

                    % STEP 2.c
                  
                    % let's proceed with the optimization
                    if (Obj.Params.Optimize)

                        % reset OptIterVal (each time we perform MaxIter
                        % optimization runs)
                        Obj.Params.OptIterVal = 0;
                            
                        % save J before the optimization to confront it
                        % later with JdotThresh
                        [J_before, ~] = Obj.cost_function(Obj.Params.TempX0Opt,Obj.Params.TempX0NonOpt,Obj.Params.Ybuffer);
                       

                        % OPTIMISATION      
                                                                                            
                        % create the optimization problem
                        try

                            % create a problem for fminsearchcon
                            % remark: the structure is the same as fmincon
                            % without the inequality constraints.
                            % createOptimProblem does not accept
                            % fminsearchon (it's from FileExchange) so this
                            % is the only workaround I came up with: first
                            % create the problem with fmincon and then
                            % replace the constraints and the solver name
                            problem = createOptimProblem('fmincon', ...                                                                                     % solver
                                                         'objective',       @(x)Obj.cost_function(x,Obj.Params.TempX0NonOpt,Obj.Params.Ybuffer), ...        % cost function handle
                                                         'x0',              Obj.Params.TempX0Opt, ...                                                       % initial condition
                                                         'lb',              Obj.Params.LBcon, ...                                                           % lower bound constraints                                                       
                                                         'ub',              Obj.Params.UBcon, ...                                                           % upper bound constraints
                                                         'Aeq',             Obj.Params.AconEq, ...                                                          % equality constraints (A)
                                                         'beq',             Obj.Params.BconEq, ...                                                          % equality constraints (b)
                                                         'nonlcon',         @(x)Obj.NONCOLcon(x,Obj), ...                                                   % nonlinear constraints                            
                                                         'options',         Obj.Params.Myoptioptions);                                                      % solver options

                            % no inequality constraints
                            problem = rmfield(problem,'bineq');
                            problem = rmfield(problem,'Aineq'); 

                            % set the options and the solver
                            problem.options = Obj.Params.Myoptioptions;
                            problem.solver = 'fminsearchcon';

                        catch ME

                            % there are a thousand of thing that could go
                            % wrong here, so we put this to leave a space
                            % for a smart breakpoint to test the setup
                            % without rerunning everything. 
                            disp(ME);
                            error('WRONG OPTIMISATION SETUP');

                        end 

                        % STEP 2.c.a
                                
                        % here we run the optimization (and pray)
                        [NewXopt, J] = Obj.Functions.Fmin(problem.objective, ...    % cost function handle
                                                          problem.x0, ...           % initial condition
                                                          problem.lb, ...           % lower bound constraints
                                                          problem.ub, ...           % upper bound constraints
                                                          problem.Aeq, ...          % equality constraints (A)
                                                          problem.beq, ...          % equality constraints (b)
                                                          problem.nonlcon, ...      % nonlinear constraints
                                                          problem.options ...       % solver options
                                                          );

                        % store the cost function 
                        Obj.Params.JStory(Obj.Params.OptIterVal:end,Obj.Params.ActualTimeIndex) = J;
                        
                        % reconstruct NewXopt from opt/nonopt vars
                        NewXopt_tmp = [];

                        % cycle over trajectories
                        for traj = 1:Obj.Params.Ntraj

                            % set trajectory
                            Obj.Params.traj = traj;

                            % stack vector with  opt and nonopt variables
                            NewXopt_end = zeros(Obj.Params.DimState,1);
                            NewXopt_end(Obj.Params.VarsOpt) = NewXopt;
                            NewXopt_end(Obj.Params.VarsNonOpt) = Obj.Params.TempX0NonOpt(traj).val;     

                            % assign to the struct
                            NewXopt_tmp(traj).val = NewXopt_end;  

                        end
                            
                        % set new state
                        NewXopt = NewXopt_tmp;                                                

                        % optimization counter
                        if traj == 1

                            % number of optimization runs
                            Obj.Params.OptCounter = Obj.Params.OptCounter + 1;

                            % store time index of the optimization
                            Obj.Params.OptTime = [Obj.Params.OptTime Obj.Params.ActualTimeIndex];

                        end

                        % STEP 2.c.b

                        % check JDotThresh condition 
                        J_diff = (J/J_before);

                        % if this works, then proceed to propagate the new
                        % state foward
                        if (J_diff <= Obj.Params.JdotThresh)     

                            % update counter of how many times the
                            % optimization has been accepted and the
                            % relative time index
                            Obj.Params.SelectCounter = Obj.Params.SelectCounter + 1;
                            Obj.Params.SelectTime = [Obj.Params.SelectTime Obj.Params.ActualTimeIndex];

                            % cycle over the trajectories
                            for traj=1:Obj.Params.Ntraj

                                % set traj
                                Obj.Params.traj = traj;

                                % update state
                                Obj.Xhat(traj).val(:,Obj.Params.BackTimeIndex) = NewXopt(traj).val;

                                % state to repropagate 
                                x_propagate = NewXopt(traj).val;

                                % update Params. FUNDAMENTAL, otherwise the
                                % model parameters might not be the correct
                                % ones
                                Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,x_propagate);

                                % set initial time, namely k-N+1
                                Obj.Params.BackTime = Obj.T(Obj.Params.BackTimeIndex);

                                % State and measure update of the first 
                                % time index, namely k-N+1
                                % here there is no model integration
                                % because we are considerng the initial
                                % state

                                % ESTIMATED measurement
                                Yhat_meas = Obj.Functions.Measure(x_propagate,Obj.Params,Obj.Params.BackTime,Obj.Uhat(traj).val(:,Obj.Params.BackTimeIndex),Obj);

                                % store the obtained value
                                Obj.Yhat(traj).val(:,Obj.Params.BackTimeIndex) = Yhat_meas;
                                    
                                % compute how many integrations we need to
                                % compute from backTime to ActualTime. This
                                % variable defines the length of the
                                % propagation range
                                n_iter_propagate = Obj.Params.ActualTimeIndex-Obj.Params.BackTimeIndex;
    
                                % cycle over the propagation range
                                for j=1:n_iter_propagate
                                            
                                    % back time
                                    Obj.Params.BackTimeIndex = Obj.Params.BackTimeIndex + 1;

                                    % get time interval (span)
                                    tspan = Obj.T(Obj.Params.BackTimeIndex-1:Obj.Params.BackTimeIndex);                                                                                                                     

                                    % integrate
                                    Xtmp = Obj.Functions.Ode(@(t,x)Obj.Params.Model(t, x, Obj.Params, Obj), tspan, x_propagate,Obj.Params.Odeset);                                    
                                    x_propagate = Xtmp.y(:,end);                      

                                    % set the evolution in the estimated
                                    % trajectory
                                    Obj.Xhat(traj).val(:,Obj.Params.BackTimeIndex) = x_propagate;
                                        
                                    % ESTIMATED measurements
                                    Yhat_tmp = Obj.Params.Measure(x_propagate,Obj.Params,Obj.Params.BackTime,Obj.Uhat(traj).val(:,Obj.Params.BackTimeIndex-1),Obj);
                                        
                                    % store the measurement
                                    Obj.Yhat(traj).val(:,Obj.Params.BackTimeIndex) = Yhat_tmp;            
                                        
                                end

                            end

                        else

                            % remark: we finish here not only when the
                            % JDotThresh condition is not met but also when
                            % the cost function is NaN or Inf. So, if you
                            % pyt JDotThresh = 1 and you still finish here,
                            % you migt want to check the cost function
                                
                            % here we restore the initial consition to the
                            % initial conditio before the optimization. The
                            % rest is not affected because we never
                            % re-propaated forward.

                            % cycle over trajectories
                            for traj=1:Obj.Params.Ntraj
                                    
                                % reset the initial condition
                                Obj.Xhat(traj).val(Obj.Params.VarsOpt,Obj.Params.BackTimeIndex) = Obj.Params.TempX0Opt;

                            end
                                
                            % we simply restore the parameters to the
                            % non-optimized values
                            Obj.Params = Obj.Functions.ParamsUpdate(Obj.Params,Obj.Params.TempX0(traj).val);     

                        end

                    end

                end

                % clear the screen 
                if Obj.Params.Print

                    clc;

                end 

            else

                % if no sampling is done, keep the last J
                Obj.Params.JStory(:,Obj.Params.ActualTimeIndex) = Obj.Params.JStory(:,max(1,Obj.Params.ActualTimeIndex-1));

            end

        end

    end

end
