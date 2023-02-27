%%%%%%% LICENSING %%%%%%%
% Copyright 2020-2021 Federico Oliva
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.

% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.

% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% CODE %%%%%%%%%%%%

%% class definition - obsopt observer
% This class allows to apply a newton-like observer to a specified model,
% within a model integration setup in the form:
% 1) initial setup
% ---- for loop ----
% 2) model integration 
% 3) measurements (with noise)
% 4) newton-like observer (obsopt)
% -----> back to 2
% 5) plot section (optional)
classdef obsopt < handle
    %%% class properties
    properties
        % setup: set of flags defining the main characteristics. These
        % flags can be set by properly calling the class constructor.
        setup;
        
        % init: set of fags initialised depending on setup and set of
        % variables that will be used and stored during the observation
        % process. 
        init;
    end
    
    %%% class methods %%%
    methods
        % class constructor. Each of these variables can be specifically
        % set by calling the constructor with specific properties, i.e. by
        % properly using the varargin parameter. Otherwise a default system
        % is set up
        function obj = obsopt(varargin)
            
            if any(strcmp(varargin,'params'))
                pos = find(strcmp(varargin,'params'));
                params = varargin{pos+1};
                
                % get model from params
                obj.setup.model = params.model;
                
                % get measure from params
                obj.setup.measure = params.measure;
                
                % get the integration algorithm
                obj.setup.ode = params.ode;
                obj.setup.odeset = params.odeset;
                
                % get the samling time from params
                obj.setup.Ts = params.Ts;
                
                % get the time instants vector
                obj.setup.time = params.time;
                
                % output dimension
                obj.setup.dim_out = params.OutDim;
                obj.setup.dim_out_compare = params.OutDim_compare;
            
                % state dimension
                obj.setup.dim_state = params.StateDim;
                
                % noise info
                obj.setup.noise = params.noise;
                obj.setup.noise_mu = params.noise_mu;
                obj.setup.noise_std = params.noise_std;
                
                % observed state
                obj.setup.observed_state = params.observed_state;
                
                % get state init
                obj.setup.X = params.X;
                obj.setup.X_est = params.X_est;
                
                % estimated param
                obj.setup.estimated_params = params.estimated_params;
                obj.setup.opt_vars = params.opt_vars;
                obj.setup.nonopt_vars = params.nonopt_vars;
                obj.setup.plot_vars = params.plot_vars;
                obj.setup.plot_params = params.plot_params;
                                
            else
                obj.setup.model = @(t,x,params) -2*x;
                obj.setup.measure = @(x,params) x;
                obj.setup.ode = @ode45;
                obj.setup.Ts = 1e-1;
                
                % get the time instants vector. Default is from 0 to 1s, with
                % the sample time previously set.
                obj.setup.time = 0:obj.setup.Ts:10;
                
                obj.setup.dim_out = 1;
                obj.setup.dim_state = 1;
                
                % set empty params
                obj.setup.X = 5;
                
                % noise info
                obj.setup.noise = 1;
                obj.setup.noise_mu = 0;
                obj.setup.noise_std = 1e-1;
                
                % observed state
                obj.setup.observed_state = 1;
                
                % input
                params.input = @(t,x,params) 0;
                
                % estimated param
                obj.setup.estimated_params = [];
            end
            
            % get number of iterations
            obj.setup.Niter = length(obj.setup.time);
            
            % define the time span from the sample time. This will be used
            % in the integration procedure (e.g. ode45)
            obj.setup.tspan = [0, obj.setup.Ts];
            
            % get if data are simulated or from real measurements (used in)
            % plot section
            if any(strcmp(varargin,'DataType'))
                pos = find(strcmp(varargin,'DataType'));
                obj.setup.DataType = varargin{pos+1};
            else
                obj.setup.DataType = 'simulated';
            end
            
            % run or no the optimisation
            if any(strcmp(varargin,'optimise'))
                pos = find(strcmp(varargin,'optimise'));
                obj.setup.optimise = varargin{pos+1};
            else
                obj.setup.optimise = 1;
            end
                       
            % option to print out the optimisation process or not. Default
            % is not (0).
            if any(strcmp(varargin,'print'))
                pos = find(strcmp(varargin,'print'));
                obj.setup.print = varargin{pos+1};
            else
                obj.setup.print = 0;
            end
            
            % option to define the safety interval in adaptive sampling
            if any(strcmp(varargin,'SafetyDensity'))
                pos = find(strcmp(varargin,'SafetyDensity'));
                obj.setup.safety_density = varargin{pos+1};
            else
                obj.setup.safety_density = 2;
            end
            
            % enable or not the adaptive sampling
            if any(strcmp(varargin,'AdaptiveSampling'))
                pos = find(strcmp(varargin,'AdaptiveSampling'));
                obj.setup.AdaptiveSampling = varargin{pos+1};
            else
                obj.setup.AdaptiveSampling = 0;
            end
            obj.init.nfreqs = 2;            
            obj.init.freqs = zeros(obj.init.nfreqs,1);
            obj.init.wvname = 'amor';
            obj.init.Nv = 48;
            obj.init.PLIMITS = [1e0*obj.setup.Ts 2e2*obj.setup.Ts];
            obj.init.FLIMITS = fliplr(1./obj.init.PLIMITS);            
            obj.init.NtsChanged = 0;
            obj.init.break = 0;
                        
            
            % option to define the safety interval in adaptive sampling. If
            % AdaptiveSampling flag is zero the histeresis is set to zero,
            % disabling the adaptive sampling
            if any(strcmp(varargin,'AdaptiveParams')) && (obj.setup.AdaptiveSampling)
                pos = find(strcmp(varargin,'AdaptiveParams'));
                tmp = varargin{pos+1};                         
                obj.init.Fnyq = tmp(1);     % integer
                obj.init.Fbuflen = tmp(2);  % integer < Nw
                obj.init.Fselect = tmp(3);  % 1 = min 2 = max 
                obj.init.FNts = tmp(4);
                obj.init.Fmin = tmp(5);
                obj.init.wavelet_output_dim = tmp(6:end);
            else
                obj.init.FNts = 1;
                obj.init.Fbuflen = 20;
                obj.init.Fselect = 2;
                obj.init.Fnyq = 2;
            end
            
            % enable or not the buffer flush on the adaptive sampling
            if any(strcmp(varargin,'FlushBuffer')) && (obj.setup.AdaptiveSampling)
                pos = find(strcmp(varargin,'FlushBuffer'));
                obj.setup.FlushBuffer = (obj.setup.AdaptiveSampling && varargin{pos+1});
            else
                obj.setup.FlushBuffer = 0;
            end
            
            % get the maximum number of iterations in the optimisation
            % process. Default is 100.
            if any(strcmp(varargin,'MaxIter'))
                pos = find(strcmp(varargin,'MaxIter'));
                obj.setup.max_iterVal = varargin{pos+1};
                if length(obj.setup.max_iterVal) == 1
                    obj.setup.max_iter = obj.setup.max_iterVal;                    
                else
                    obj.setup.max_iter = max(obj.setup.max_iterVal);
                end
            else
                obj.setup.max_iter = 100;
            end
            obj.init.MaxIter_story = [];
            
            % get the maximum number of iterations in the optimisation
            % process. Default is 100.
            if any(strcmp(varargin,'MaxOptTime'))
                pos = find(strcmp(varargin,'MaxOptTime'));
                obj.setup.MaxOptTime = varargin{pos+1};
                obj.setup.MaxOptTimeFlag = 0;
            else
                obj.setup.MaxOptTime = Inf;
                obj.setup.MaxOptTimeFlag = 0;
            end
            % init story
            obj.init.MaxOptTime_story = [];
            
            % check if the optimised value shall always be accepted. This
            % goes in contrast with the role of Jdot_thresh
            if any(strcmp(varargin,'AlwaysOpt'))
                pos = find(strcmp(varargin,'AlwaysOpt'));
                obj.setup.AlwaysOpt = varargin{pos+1};
            else
                obj.setup.AlwaysOpt = 1;
            end

            % get the Nw buffer length. See reference on the algorithm for
            % more detailed information. Default is 5.
            if any(strcmp(varargin,'Nw'))
                pos = find(strcmp(varargin,'Nw'));
                obj.setup.w = varargin{pos+1};
            else
                obj.setup.w = 5;
            end
            
            % get the Nts down sampling period. See reference on the algorithm for
            % more detailed information. Default is 3.
            if any(strcmp(varargin,'Nts'))
                pos = find(strcmp(varargin,'Nts'));
                obj.setup.NtsVal = varargin{pos+1};
                if length(obj.setup.NtsVal) == 1
                    obj.setup.Nts = obj.setup.NtsVal;
                    obj.setup.NtsVal = ones(1,obj.setup.w)*obj.setup.Nts;
                else
                    obj.setup.Nts = min(obj.setup.NtsVal);
                end
            else
                obj.setup.Nts = 3;
            end
            obj.init.Nsaved = 0;
            
            % Discrete sampling time (for filters)
            obj.setup.DTs = obj.setup.Nts*obj.setup.Ts;

            % get the conditions to consider the optimisation result. See 
            % reference on the algorithm for more detailed information. 
            % Default is 90%.
            if any(strcmp(varargin,'Jdot_thresh'))
                pos = find(strcmp(varargin,'Jdot_thresh'));
                obj.setup.Jdot_thresh = varargin{pos+1};
            else
                obj.setup.Jdot_thresh = 0.9;
            end

            % get the conditions for the optimisation to stop. Default are
            % 1e-10 and 1e3. Check out outfun options for optimisation
            % methods like fminsearch or fminunc in MATLAB
            if any(strcmp(varargin,'J_thresh'))
                pos = find(strcmp(varargin,'J_thresh'));
                obj.setup.J_thresh = varargin{pos+1};
            else
                obj.setup.J_thresh = [1e-10, 1e3];
            end
            
            % normalise the cost function
            if any(strcmp(varargin,'J_normalise'))
                pos = find(strcmp(varargin,'J_normalise'));
                obj.setup.J_normalise = varargin{pos+1};
            else
                obj.setup.J_normalise = 1;
            end
            
            % change max iterations depending on PE 
            if any(strcmp(varargin,'PE_maxiter'))
                pos = find(strcmp(varargin,'PE_maxiter'));
                obj.setup.PE_maxiter = varargin{pos+1};
            else
                obj.setup.PE_maxiter = 0;
            end
            
            % PE position
            if any(strcmp(varargin,'PEPos'))
                pos = find(strcmp(varargin,'PEPos'));
                obj.setup.PEPos = varargin{pos+1};
            else
                obj.setup.PEPos = [1 1];
            end

            % get the optimisation method. Default is fminsearch from
            % MATLAB
            if any(strcmp(varargin,'opt'))
                pos = find(strcmp(varargin,'opt'));
                obj.setup.fmin = varargin{pos+1};
            else
                obj.setup.fmin = @fminsearch;
            end                        
            
            % get the multistart option
            if any(strcmp(varargin,'MultiStart'))
                pos = find(strcmp(varargin,'MultiStart'));
                obj.setup.MultiStart = varargin{pos+1};
            else
                obj.setup.MultiStart = 0;
            end
            
            % get the multistart option
            test = func2str(obj.setup.fmin);
            if any(strcmp(varargin,'Bounds')) && ~strcmp(test,'fmincon')
                pos = find(strcmp(varargin,'Bounds'));
                obj.setup.bounds = varargin{pos+1};                
            else
                obj.setup.bounds = 0;
            end
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsPos'))
                pos = find(strcmp(varargin,'BoundsPos'));
                obj.setup.boundsPos = varargin{pos+1};                
            else
                obj.setup.boundsPos = [];
            end            
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsValLow'))
                pos = find(strcmp(varargin,'BoundsValLow'));
                obj.setup.boundsValLow = varargin{pos+1};                
            else
                obj.setup.boundsValLow = [];
            end
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsValUp'))
                pos = find(strcmp(varargin,'BoundsValUp'));
                obj.setup.boundsValUp = varargin{pos+1};                
            else
                obj.setup.boundsValUp = [];
            end
            
            % get the multistart option
            if any(strcmp(varargin,'BoundsWeight'))
                pos = find(strcmp(varargin,'BoundsWeight'));
                obj.setup.boundsWeight = varargin{pos+1};                
            else
                obj.setup.boundsWeight = ones(1,numel(obj.setup.boundsPos));
            end
            
            % constraint pos
            if any(strcmp(varargin,'ConPos'))
                pos = find(strcmp(varargin,'ConPos'));
                obj.setup.ConPos = varargin{pos+1};
            else
                obj.setup.ConPos = [];
            end
            
            
            % handle fmincon
            try
                test = func2str(obj.setup.fmin);
            catch
                test = 'null';
            end
            if strcmp(test,'fmincon') || strcmp(test,'fminsearchbnd') || any(strcmp(varargin,'Bounds'))
                if any(strcmp(varargin,'Acon'))
                    pos = find(strcmp(varargin,'Acon'));
                    obj.setup.Acon = varargin{pos+1};
                else
                    obj.setup.Acon = [];                    
                end
                if any(strcmp(varargin,'Bcon'))
                    pos = find(strcmp(varargin,'Bcon'));
                    obj.setup.Bcon = varargin{pos+1};
                else
                    obj.setup.Bcon = [];                
                end
                if any(strcmp(varargin,'Acon_eq'))
                    pos = find(strcmp(varargin,'Acon_eq'));
                    obj.setup.Acon_eq = varargin{pos+1};
                else
                    obj.setup.Acon_eq = [];                
                end
                if any(strcmp(varargin,'Bcon_eq'))
                    pos = find(strcmp(varargin,'Bcon_eq'));
                    obj.setup.Bcon_eq = varargin{pos+1};
                else
                    obj.setup.Bcon_eq = [];                
                end
                if any(strcmp(varargin,'LBcon'))
                    obj.setup.LBcon = double(-Inf*ones(1,length(obj.setup.opt_vars)));
                    pos = find(strcmp(varargin,'LBcon'));
                    obj.setup.LBcon(obj.setup.ConPos) = varargin{pos+1};
                else
                    obj.setup.LBcon = zeros(1,length(obj.setup.opt_vars));                
                end
                if any(strcmp(varargin,'UBcon'))
                    obj.setup.UBcon = double(Inf*ones(1,length(obj.setup.opt_vars)));
                    pos = find(strcmp(varargin,'UBcon'));
                    obj.setup.UBcon(obj.setup.ConPos) = varargin{pos+1};
                else
                    obj.setup.UBcon = zeros(1,length(obj.setup.opt_vars));          
                end
                if any(strcmp(varargin,'NONCOLcon'))
                    pos = find(strcmp(varargin,'NONCOLcon'));
                    obj.setup.NONCOLcon = varargin{pos+1}; 
                else
		    obj.setup.NONCOLcon = @obj.NONCOLcon;
                end
            else
                obj.setup.Acon = [];    
                obj.setup.Bcon = [];
                obj.setup.Acon_eq = [];
                obj.setup.Bcon_eq = [];
                obj.setup.LBcon = [];
                obj.setup.UBcon = [];
                obj.setup.NONCOLcon = [];
            end
            
            % store J terms
            if any(strcmp(varargin,'Jterm_store')) && (~strcmp(obj.setup.fmin,'broyden'))
                pos = find(strcmp(varargin,'Jterm_store'));
                obj.setup.Jterm_store = varargin{pos+1};
            else
                if (~strcmp(obj.setup.fmin,'broyden'))
                    obj.setup.Jterm_store = 1;
                else
                    obj.setup.Jterm_store = 0;
                end
            end

            % get the algorithm direction (forward or backward). V1.1 has
            % only forward implemented
            if any(strcmp(varargin,'forward'))
                pos = find(strcmp(varargin,'forward'));
                obj.setup.forward = varargin{pos+1};
            else
                obj.setup.forward = 1;
            end            
            
            % wait all buffer to be filled
            if any(strcmp(varargin,'WaitAllBuffer'))
                pos = find(strcmp(varargin,'WaitAllBuffer'));
                obj.setup.WaitAllBuffer = varargin{pos+1};
            else
                obj.setup.WaitAllBuffer = 0;
            end
            
            % reference model
            if any(strcmp(varargin,'model_reference'))
                pos = find(strcmp(varargin,'model_reference'));
                obj.setup.model_reference = varargin{pos+1};
            else
                obj.setup.model_reference = obj.setup.model;
            end
            
            % reference measure
            if any(strcmp(varargin,'measure_reference'))
                pos = find(strcmp(varargin,'measure_reference'));
                obj.setup.measure_reference = varargin{pos+1};
            else
                obj.setup.measure_reference = obj.setup.measure;
            end
            
            
            % get the cost function terms. Default uses measures only, 
            % without any additional filter (e.g. [1 0 0])
            if any(strcmp(varargin,'filters'))
                pos = find(strcmp(varargin,'filters'));
                temp_scale = varargin{pos+1};
            else
                temp_scale = [1];
            end
            
            
            % filters
            obj.setup.J_temp_scale = temp_scale;
            obj.setup.J_nterm = length(temp_scale);
            obj.setup.J_nterm_total = length(temp_scale);             
            
            % get the terminal like term in the cost function
            if any(strcmp(varargin,'terminal'))
                pos = find(strcmp(varargin,'terminal'));
                if varargin{pos+1} ~= 0
                    obj.setup.J_term_terminal = 1;
                    obj.setup.J_temp_scale = [obj.setup.J_temp_scale, varargin{pos+1}];
                    obj.setup.J_term_terminal_position = length(obj.setup.J_temp_scale);
                    obj.setup.J_nterm_total = obj.setup.J_nterm_total+1;
                else
                    obj.setup.J_term_terminal = 0;
                end
            else
                obj.setup.J_term_terminal = 0;
            end     
            
            if any(strcmp(varargin,'terminal_states')) && obj.setup.J_term_terminal
                pos = find(strcmp(varargin,'terminal_states'));
                obj.setup.terminal_states = varargin{pos+1};
            else
                obj.setup.terminal_states = [];
            end
            
            if any(strcmp(varargin,'terminal_weights')) && obj.setup.J_term_terminal
                pos = find(strcmp(varargin,'terminal_weights'));
                obj.setup.terminal_weights = varargin{pos+1};
            else
                obj.setup.terminal_weights = ones(size(obj.setup.terminal_states));
            end
            
            if any(strcmp(varargin,'terminal_normalise')) && obj.setup.J_term_terminal
                pos = find(strcmp(varargin,'terminal_normalise'));
                obj.setup.terminal_normalise = varargin{pos+1};
            else
                obj.setup.terminal_normalise = 0;
            end
            
            obj.setup.Nfilt = obj.setup.J_nterm-1;
            
            % option to define the safety interval in adaptive sampling
            if any(strcmp(varargin,'filterTF'))
                pos = find(strcmp(varargin,'filterTF'));
                obj.setup.filterTF = varargin{pos+1};
            else
                obj.setup.filterTF = [];
            end            
            
            % set of the integral resets or not
            if any(strcmp(varargin,'int_reset'))
                pos = find(strcmp(varargin,'int_reset'));
                obj.setup.J_int_reset = varargin{pos+1};
            else
                obj.setup.J_int_reset = 0;
            end
            
            if ~any(strcmp(varargin,'params'))
                % set initial condition perturbed
                obj.setup.X_est(:,1) = obj.setup.X(:,1) + 1e1*obj.setup.noise*(obj.setup.noise_mu + obj.setup.noise_std.*randn(obj.setup.dim_state,1));
                
                obj.setup.Ntraj = 1;
            end
            % complete the params update in .setup
            obj.setup.params = params;
            
            % number of reference trajectories            
            obj.setup.Ntraj = params.Ntraj;
            
            % set the actual cost function (J)
            obj.setup.cost_run = @obj.cost_function;
            
            % initialise the class
            obj = obj.obs_init();
        end
        
        % init function. This function defines all the intenal variables
        % that will be used during the optimisation process. Observed state
        % trajectories will also be stored in these vars. 
        function obj = obs_init(obj)
            
            % get params structure. It's redundant but by doing so we save
            % the initial values of the params
            obj.init.params = obj.setup.params;
            obj.init.params = obj.setup.params.params_update(obj.init.params,obj.setup.X_est(1).val(:,1));
            
            % get initial state
            obj.init.X = obj.setup.X;
            obj.init.X_est = obj.setup.X_est;
            obj.init.X_est_runtime = obj.setup.X_est;
            obj.init.X_wrong = obj.setup.X_est;            
            for traj=1:obj.setup.Ntraj
                obj.init.X_filter(traj).val = cell(obj.setup.Nfilt,1);
                obj.init.X_filter_est(traj).val = cell(obj.setup.Nfilt,1);
                obj.init.X_filter_buffer_control(traj).val = cell(obj.setup.Nfilt,1);
                for i=1:obj.setup.Nfilt
                    for dim=1:obj.setup.dim_out
                        obj.init.X_filter(traj).val{i,dim} = zeros(obj.setup.filterTF(i).dim,obj.setup.Niter);
                        obj.init.X_filter_est(traj).val{i,dim} = 0*randn(obj.setup.filterTF(i).dim,obj.setup.Niter);
                        obj.init.X_filter_buffer_control(traj).val{i,dim} = zeros(obj.setup.filterTF(i).dim,obj.setup.Niter);
                    end
                end
            end
            
            % create filter matrix string
            Nfilt = length(obj.setup.filterTF);
            tmp_str = [];
            for filt=1:Nfilt
                tmp_str = [tmp_str,'obj.setup.filterTF(',num2str(filt),').A,obj.setup.filterTF(',num2str(filt),...
                          ').B,obj.setup.filterTF(',num2str(filt),').C,obj.setup.filterTF(',num2str(filt),').D,'];
            end
            tmp_str = tmp_str(1:end-1);
            if isempty(tmp_str)
                obj.init.matrix_str = '0';
            else
                obj.init.matrix_str = tmp_str;
            end
            
            % create scale factor, namely the weight over time for all the
            % cost function terms. In V1.1 no forgetting factor is
            % implemented. 
            obj = obj.scale_factor();
            obj.init.normalised = 0;
            obj.init.ConstrNormalised = 0;
            
            % Window Samples: maximum number of time instants considered in
            % the optimisation if fixed sampling is used. For more
            % information check the reference. 
            obj.init.WindowSamples = max(2,obj.setup.Nts*(obj.setup.w-1)+1);
            
            % get safety interval for the adaptive sampling
            obj.init.safety_interval = int32(obj.setup.safety_density*obj.init.WindowSamples);
            
            % BackIterIndex: t0 index (starting from 1). For more
            % information check the reference. 
            obj.init.BackIterIndex = 1;            
            
            % persistenct excitation setup
            obj.init.c1_PE = 5;
            obj.init.d1_PE = 20;
            obj.init.PE_pos_array = [];
            for i=1:obj.setup.Ntraj
                obj.init.buf_PE(i).val = zeros(1,obj.init.d1_PE);
            end

            % measure buffer: these buffers are used to store the measured
            % and estimated values on the observed states.           
            for i=1:obj.setup.Ntraj
                obj.init.Y(i).val =  zeros(obj.setup.J_nterm,obj.setup.dim_out,obj.setup.w);
                obj.init.Ytrue_full_story(i).val = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
                obj.init.Y_full_story(i).val = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
                obj.init.Yhat_full_story(i).val = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
                obj.init.target_story(i).val = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
                % buffer for Y during drive computation (control design)
                obj.init.Y_buffer_control(i).val = [];
                obj.init.drive_out(i).val = [];
                obj.init.input_story(i).val(:,1) = zeros(obj.setup.params.dim_input,1);
                obj.init.input_story_ref(i).val(:,1) = zeros(obj.setup.params.dim_input,1);
            end
            
            % input dimension
            obj.setup.dim_input = obj.setup.params.dim_input;

            % buffer adaptive sampling: these buffers keep track of the
            % time instants in which the measured data have been stored. 
            obj.init.Y_space = zeros(1,obj.setup.w);
            obj.init.Y_space_full_story = 0;            

            % dJ condition buffer (adaptive sampling): numerical condition
            % to be thresholded in order to trigger the adaptive sampling.
            % Not implemented in V1.1
            obj.init.dJ_cond_story = [];
            
            % observer cost function init
            obj.init.J = 0;
            obj.init.Jstory = obj.init.J;
            obj.init.Jterm_store = zeros(obj.setup.J_nterm_total,1);
            obj.init.Jterm_story = obj.init.Jterm_store;
            obj.init.Jdot_story = 0;
            % J_components is used to keep track of the different cost
            % function terms amplitude. Not implemented in V1.1
            obj.init.J_components = ones(obj.setup.dim_out,obj.setup.J_nterm);
            obj.init.just_optimised = 0;
            obj.init.FirstOpt = 0;
            
            % time instants in which the optimisation is run
            obj.init.temp_time = [];
            % time instand in which the optimisation is accepted 
            % (see J_dot_thresh for more information)
            obj.init.opt_chosen_time = [];
            
            % single integration time
            obj.init.t_ode = 0;
            obj.init.t_meas = 0;
            obj.init.t_filt = 0;
            obj.init.t_J = 0;
            obj.init.t_ode_start = 0;
            obj.init.t_meas_start = 0;
            obj.init.t_filt_start = 0;
            obj.init.t_J_start = 0;
            
            % cost function gradient memory buffer. Not implemented in V1.1
            obj.init.grad_story = zeros(obj.setup.dim_state,1);
            
            % optimisation counters. Related to temp_time and chosen_time
            obj.init.opt_counter = 0;
            obj.init.select_counter = 0;
            
            %%% start of optimisation setup %%%
            % optimset: check documentation for fminsearch or fminunc
            obj.init.TolX = 0;
            obj.init.TolFun = 0;
            obj.init.DiffMinChange = 1e-3;
            obj.init.last_opt_time = 0;
            obj.init.opt_time = 0;
            
            % set options
            if obj.setup.print 
                obj.init.display = 'iter';
            else
                obj.init.display = 'off'; 
            end
    
            
            % optimset                    
            if strcmp(func2str(obj.setup.fmin),'fminsearchcon')
                obj.init.myoptioptions = optimset('MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                  'MaxFunEvals',Inf,'OutputFcn',@obj.outfun,'TolFun',obj.init.TolFun,'TolX',obj.init.TolX);   
            elseif strcmp(func2str(obj.setup.fmin),'patternsearch')                              
                obj.init.myoptioptions = optimoptions(func2str(obj.setup.fmin), 'MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                      'Cache', 'on', 'UseParallel', false, 'StepTolerance', 0,'MaxFunEvals',Inf,'Algorithm','nups');            
            else
                obj.init.myoptioptions = optimoptions(func2str(obj.setup.fmin), 'MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                      'OptimalityTolerance', 0, 'StepTolerance', 0,'MaxFunEvals',Inf, 'GradObj', 'off',...
                                                      'OutputFcn',@obj.outfun,'TolFun',obj.init.TolFun,'TolX',obj.init.TolX, ...
                                                      'FiniteDifferenceStepSize', obj.init.DiffMinChange, 'FiniteDifferenceType','central');                          
            end
            %%% end of optimisation setup %%%

        end

        % nonlinear constraing default
        function [c, ceq] = NONCOLcon(varargin)
            c = 0;
            ceq = 0;
        end
        
        % scale factor: this method defines the cost function weights
        % accordingly to the selected filters (see setup.temp_scale).
        function obj = scale_factor(obj)
            for dim=1:obj.setup.dim_out
                obj.init.scale_factor(dim,:) = obj.setup.J_temp_scale.*ones(1,obj.setup.J_nterm_total);
            end
        end
        
        % outfun: this method check wether or not the optimisation process
        % shall be stopped or not. Check setup.J_thresh for more
        % information. 
        function stop = outfun(obj,x, optimValues, state)
            if (optimValues.iteration == obj.setup.max_iter) || (optimValues.fval <= obj.init.TolFun) || obj.setup.MaxOptTimeFlag
                stop = true;
            else
                stop = false;
            end
        end
        
        % convex conjugate: this function computes the convex conjugate of
        % the cost function
        function [J_final,x_final,obj] = convex_conjugate(obj,varargin)
            
            % get opt state
            x_opt = varargin{1};           
            
            % define the function handle for the conjugate
            y = ones(size(x_opt));
            f_cc = @(x)-(transpose(y)*x - obj.cost_function(x,varargin{2},varargin{3},varargin{4}));
            
            % initial condition
            x_0 = x_opt;
            
            % solve the sup problem to find the conjugate fcn
            [x_final,J_final] = fminunc(f_cc,x_0,obj.init.myoptioptions);
            
            obj.init.convex_conjugate(end+1) = J_final;
            
        end                
        
        % cost function: objective to be minimised by the MHE observer
        function [J_final,obj] = cost_function(obj,varargin) 
            
            obj.init.t_J_start = tic;
            
            % above ntraj init
            J_final = 0;
            J_input = 0;
                
            for traj = 1:obj.setup.Ntraj
                
                obj.init.params.optimising = 1;
                
                obj.init.traj = traj;
                obj.init.params.traj = traj;
                
                % cost function init
                Jtot = 0;
                
                % get opt state
                x_opt = varargin{1};

                % non optimised vals
                x_nonopt = varargin{2}(traj).val;
                
                % x filter vals
                x_filters = varargin{3}(traj).val;
                Lfilt = length(x_filters);

                % create state
                x = zeros(obj.setup.dim_state+Lfilt,1);
                x(obj.setup.opt_vars) = x_opt;
                x(obj.setup.nonopt_vars) = x_nonopt;
                x(obj.setup.dim_state+1:end) = x_filters;
                
                % update params
                obj.init.params = obj.setup.params.params_update(obj.init.params,x);
                
                % get desired trajectory
                y_target = varargin{4}(traj).val;

                % init Jterm store
                obj.init.Jterm_store = zeros(obj.setup.J_nterm_total,1);
                
                %%% integrate %%%
                % define time array
                tspan_pos = [obj.init.BackIterIndex, nonzeros(obj.init.Y_space)'];
                tspan = obj.setup.time(tspan_pos(1):tspan_pos(end));  
                buf_dist = diff(tspan_pos);
                n_iter = sum(buf_dist);
                % initial condition
                x_start = x(1:obj.setup.dim_state);      
                % reset buffer for Y during drive computation (control design)
                obj.init.Y_buffer_control(traj).val = [];
                for i=1:obj.setup.Nfilt
                    for dim=1:obj.setup.dim_out
                        obj.init.X_filter_buffer_control(traj).val{i,dim} = 0*obj.init.X_filter_buffer_control(traj).val{i,dim};
                    end
                end
                % get evolution with input only if at least 2 iinstans are
                % considered
                if length(tspan)>1
                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params, obj), tspan , x_start, obj.setup.odeset);
                else
                    X.y = x_start;
                end
                
                obj.init.params.optimising = 0;
                
                % check for NaN or Inf
                NaN_Flag = find(isnan(X.y));
                if NaN_Flag
                    J_final = NaN;
                    break
                end
                
                Inf_Flag = isempty(isinf(X.y));
                Huge_Flag = isempty(X.y>1e10);
                if Inf_Flag || Huge_Flag
                    J_final = Inf;
                    break
                end                
                
                %%% get measure  %%               
                Yhat = zeros(obj.setup.Nfilt+1,obj.setup.dim_out,size(X.y,2));                
                u_in = [zeros(size(obj.init.input_story(traj).val,1),1), obj.init.input_story(traj).val];
                Yhat(1,:,:) = obj.setup.measure(X.y,obj.init.params,tspan,u_in(:,(tspan_pos(1):tspan_pos(end))),obj);
                
                %%% compute filters %%%
                if obj.setup.Nfilt > 0                     
                    
                    % how long iterate
                    % odeDD
%                     tspan_filt = 1+(0:tspan_pos(end)-tspan_pos(1));
                    % Lsim
                    tspan_filt = 1+(0:tspan_pos(end)-tspan_pos(1));
                    
                    % filter state
                    x0_filter.val = cell(obj.setup.Nfilt,obj.setup.dim_out);
                    startpos = 0;
                    for filt=1:obj.setup.Nfilt 
                       for dim=1:obj.setup.dim_out
                           x0_filter.val{filt,dim}(:,max(1,tspan_filt(1)-1)) = x_filters(startpos+1:startpos+obj.setup.filterTF(filt).dim);
                           startpos = startpos + obj.setup.filterTF(filt).dim;
                       end
                    end
                             
                    try
                        [Y, ~] = obj.measure_filter(Yhat(:,:,tspan_filt),tspan_filt,x0_filter);                         
                    catch                        
                        J_final = NaN;
                        break
                    end                    
                    
                    for filt=1:obj.setup.Nfilt
                        for dim=1:obj.setup.dim_out
                            % odeDD
%                             Yhat(filt+1,dim,tspan_filt(2:end)) = Y{filt,dim}.val;
                            % Lsim                            
                            Yhat(filt+1,dim,tspan_filt) = Y{filt,dim}.val(tspan_filt);                               
                        end
                    end
                end                                                                              
                % cost function
                J = zeros(obj.setup.J_nterm,length(obj.setup.dim_out_compare));
                target_pos = find(obj.init.Y_space ~= 0);

                for term=1:obj.setup.J_nterm
                    % get the J
                    for dim=1:obj.setup.dim_out
                        tspan_vals = tspan_pos(2:end) - tspan_pos(1) + 1;
                        target_tmp = reshape((y_target(term,dim,target_pos)),length(tspan_pos)-1,1);
                        hat_tmp = reshape(Yhat(term,dim,tspan_vals),length(tspan_pos)-1,1);
                        diff_var = target_tmp-hat_tmp;
                        J(term,dim) = transpose(diff_var)*diff_var;
                    end

                end                           

                % scaling
                J_scaled = zeros(size(J));
                for dim=1:length(obj.setup.dim_out_compare)
                    J_scaled(:,obj.setup.dim_out_compare(dim)) = transpose(obj.init.scale_factor_scaled(obj.setup.dim_out_compare(dim),1:obj.setup.J_nterm)).*J(:,obj.setup.dim_out_compare(dim));
                    Jtot = Jtot + sum(J_scaled(:,obj.setup.dim_out_compare(dim))); 
                end

                % store terms
                obj.init.Jterm_store(1:obj.setup.J_nterm) = obj.init.Jterm_store(1:obj.setup.J_nterm) + sum(J_scaled,2);                
                
                %%% terminal cost %%%
                if obj.setup.J_term_terminal
                    x0 = obj.init.temp_x0(obj.init.traj).val;                    
                    xterm = x0(obj.setup.terminal_states)-x(obj.setup.terminal_states);
                    paramsDiff = reshape(xterm,1,length(obj.setup.terminal_states));
                    J_terminal = paramsDiff*diag(obj.init.scale_factor_scaled_terminal)*transpose(paramsDiff);
                    
                    % store terms
                    obj.init.Jterm_store(end) = J_terminal; 
                else
                    J_terminal = 0;
                end
                                             
                % non opt vars barrier term                
                if obj.setup.bounds                    
                    for bound=1:length(obj.setup.boundsPos)
                        % init value
                        init_value = obj.init.temp_x0(obj.init.traj).val(obj.setup.boundsPos(bound));
                        % barrier - low
                        err_low = min(X.y(obj.setup.boundsPos(bound),2:end)-obj.setup.boundsValLow(bound));
                        % barrier - up
                        err_up = max(obj.setup.boundsValUp(bound)-X.y(obj.setup.boundsPos(bound),2:end));
                        % terminal
                        err_terminal = norm(X.y(obj.setup.boundsPos(bound),:)-init_value);
                        % sum stuff
                        if (err_low > 0) && (err_up > 0)
                            J_barr(bound) = 0*obj.setup.boundsWeight(bound)*err_terminal;
                        else
                            J_barr(bound) = 1e5;
                        end
                        
                    end
                    
                else
                    J_barr = 0;
                end
                                
                if any(isinf(J_barr))
                    J_final = Inf;
                    break
                end 
                
                %%% Allocation cost fucntion %%%
                if 0
                    u_diff = obj.init.input_story(traj).val;
                    u_diff_norm = obj.init.params.Ru*vecnorm(u_diff).^2;                
                    J_input = J_input + sum(u_diff_norm);
                else
                    J_input = 0;
                end
                                
                J_final = J_final + Jtot + sum(J_barr) + J_terminal + 0*J_input;

                %%% final stuff %%%                
                obj.init.Yhat_temp = Yhat;
                
                currentTime = toc(obj.setup.opt_temp_time);
                if currentTime > obj.setup.MaxOptTime
                   J_final = 0;
                   obj.setup.MaxOptTimeFlag = 1;
                   return
                end
            

            end
            
            obj.init.t_J(end+1) = toc(obj.init.t_J_start);  
        end
                 
        %%% LSIM filtering %%%
        function [Y, X] = measure_filter(varargin)
                                                  
            % get varargin
            obj = varargin{1};
            U = varargin{2};
            tspan_pos = varargin{3};
            X_filter = varargin{4};
            tspan = obj.setup.time(tspan_pos)-obj.setup.time(tspan_pos(1));
            
            obj.init.t_filt_start = tic;
            
            % n filters
            Nfilter = obj.setup.Nfilt;                      
            
            if (Nfilter > 0)
                
                % iter filters
                for nfilt=1:Nfilter
                                        
                    % iter dim
                    for dim=1:obj.setup.dim_out 
                        
                        % init condition
                        x0_filter = X_filter.val{nfilt,dim}(:,tspan_pos(1));                        

                        % inputs
                        u = reshape(U(1,dim,tspan_pos),1,length(tspan_pos));
                        
                        if (min(diff(tspan_pos)) ~= 0)                                 
                            [Y{nfilt,dim}.val, ~, tmp_xtraj] = lsim(obj.setup.filterTF(nfilt).TF,u',tspan,x0_filter);  
                            X{nfilt,dim}.val = transpose(tmp_xtraj);
                        else
                            Y{nfilt,dim}.val = 0;
                            X{nfilt,dim}.val = x0_filter;
                        end
                    end
                end
            else
                Y = 0;
                X = 0;
            end
            
            obj.init.t_filt(end+1) = toc(obj.init.t_filt_start);
            
        end
        
        % dJ_cond: function for the adaptive sampling
        % test: try to work with wavelets
        % it will simply return the main frequencies of the current signal
        function obj = dJ_cond_v5_function(obj)            
            
            buffer_ready = (obj.init.ActualTimeIndex > obj.init.FNts*obj.init.Fbuflen);

            if buffer_ready && obj.setup.AdaptiveSampling                

                % get current buffer - new
                pos_hat = obj.init.ActualTimeIndex:-obj.init.FNts:(obj.init.ActualTimeIndex-obj.init.FNts*obj.init.Fbuflen);                
                short_win_data = squeeze(obj.init.Y_full_story(obj.init.traj).val(1,obj.init.wavelet_output_dim,pos_hat));                

                % set data                
                short_win_data = reshape(short_win_data,numel(obj.init.wavelet_output_dim),numel(pos_hat));
                buffer = vecnorm(short_win_data,2,1);                 
                                                                                
                % frequency constraint                                
                %[WT, F] = cwt(buffer,obj.init.wvname,1/obj.setup.Ts,'VoicesPerOctave',obj.init.Nv,'FrequencyLimits',obj.init.FLIMITS);                
                [WT, ~] = cwt(buffer,obj.init.wvname,1/obj.setup.Ts,'VoicesPerOctave',obj.init.Nv);                


                % real values
                WT_real = real(WT);
                WT_norm = vecnorm(WT_real,2,1);   
                WT_norm_filt = movmean(WT_norm,5);                
                %F = scal2frq(WT_norm_filt,'morl',obj.setup.Ts);
                % find derivative
                [WT_norm_peaks,pos_peaks] = findpeaks(WT_norm_filt);                
                % set freqs
                if ~isempty(pos_peaks)
                    % pos max and min on the WT
                    [pos_max] = find(WT_norm_filt == max(WT_norm_peaks),1,'first');
                    [pos_min] = find(WT_norm_filt == min(WT_norm_peaks),1,'first');

                    % store max min F
                    %obj.init.Fcwt_story(:,obj.init.ActualTimeIndex) = [F(pos_max) F(pos_min)];

                    % pos max and min on the F
                    % new                    
                    F_def(1,1) = 2*pi*WT_norm_filt(pos_max);  %1 max
                    F_def(2,1) = 2*pi*WT_norm_filt(pos_min);  %2 min
                       
                    if min(F_def) < obj.init.Fmin
                        F_def = zeros(obj.init.nfreqs,1);
                    end

                    % select freqs
                    obj.init.freqs = [obj.init.freqs F_def];
                else
                    obj.init.freqs = [obj.init.freqs zeros(obj.init.nfreqs,1)];
                end                                                          
            else
                obj.init.freqs = [obj.init.freqs zeros(obj.init.nfreqs,1)];
            end                       
        end                
        
        % target function (observer or control design)
        function obj = target(obj)          
            for i=1:obj.setup.Ntraj
                obj.init.target(i).val = obj.init.Y(i).val;
            end
        end               
        
        % observer function: this method wraps up all the afromentioned
        % methods and actually implements the observer. Check the reference
        % for more information. 
        function obj = observer(obj,xhat,y)
            
            % get estimations
            for traj=1:obj.setup.Ntraj
                obj.init.traj = traj;
                xhat_tmp(traj).val = xhat(traj).val(:,obj.init.ActualTimeIndex);
            end
            xhat = xhat_tmp;
            
            obj.init.just_optimised = 0;
            
            for traj=1:obj.setup.Ntraj
                obj.init.traj = traj;
                % save runtime state
                obj.init.X_est_runtime(traj).val(:,obj.init.ActualTimeIndex) = obj.init.X_est(traj).val(:,obj.init.ActualTimeIndex);
                % get ESTIMATED measure from ESTIMATED state (xhat)
                yhat(traj).val = obj.setup.measure(xhat(traj).val,obj.init.params,obj.setup.time(obj.init.ActualTimeIndex),obj.init.input_story(traj).val(:,max(1,obj.init.ActualTimeIndex-1)),obj);
            end
            
            for traj=1:obj.setup.Ntraj
                obj.init.traj = traj;
                % get filters - y
                obj.init.Y_full_story(traj).val(1,:,obj.init.ActualTimeIndex) = y(traj).val;                 
                tspan_pos = [max(1,obj.init.ActualTimeIndex-1), obj.init.ActualTimeIndex];
                [dy, x_filter] = obj.measure_filter(obj.init.Y_full_story(traj).val,tspan_pos,obj.init.X_filter(obj.init.traj));                
                for filt=1:obj.setup.Nfilt
                    for dim=1:obj.setup.dim_out
                        y_tmp(dim,1) = dy{filt,dim}.val(end);
                        obj.init.X_filter(traj).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                    end   
                    y(traj).val = [y(traj).val, y_tmp];
                end                
                % get filters - yhat
                obj.init.Yhat_full_story(traj).val(1,:,obj.init.ActualTimeIndex) = yhat(traj).val;
                tspan_pos = [max(1,obj.init.ActualTimeIndex-1), obj.init.ActualTimeIndex];
                [dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,tspan_pos,obj.init.X_filter_est(obj.init.traj));
                for filt=1:obj.setup.J_nterm-1
                    for dim=1:obj.setup.dim_out
                        y_tmp(dim,1) = dyhat{filt,dim}.val(end);                        
                        obj.init.X_filter_est(traj).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                    end
                    yhat(traj).val = [yhat(traj).val, y_tmp];                    
                end
            end            
            
            %%%%%%%%%%%%%%%%%%%%% INSERT OPT %%%%%%%%%%%%%%%%%%%%%
            for term=1:obj.setup.J_nterm
                for traj=1:obj.setup.Ntraj
                    obj.init.traj = traj;
                    obj.init.Y_full_story(traj).val(term,:,obj.init.ActualTimeIndex) = y(traj).val(:,term);                    
                    obj.init.Yhat_full_story(traj).val(term,:,obj.init.ActualTimeIndex) = yhat(traj).val(:,term);
                    obj.init.Yhat_full_story_runtime(traj).val(term,:,obj.init.ActualTimeIndex) = yhat(traj).val(:,term);
                end
            end                        

            % fisrt bunch of data - read Y every Nts and check if the signal is
            distance = obj.init.ActualTimeIndex-obj.init.Y_space(end);   
            NtsPos = mod(obj.init.Nsaved,obj.setup.w)+1;
            obj.init.NtsPos = NtsPos;

            % if the current Nts is different from the standard one, than
            % it measn that a complete cycle has been done, and you should
            % replace it again with the original one
            if obj.setup.NtsVal(NtsPos) ~= obj.setup.Nts
                obj.setup.NtsVal(NtsPos) = obj.setup.Nts;
            end
            
            % set optimization time and iterations depending on the current Nts
            if obj.setup.NtsVal(NtsPos) == min(obj.setup.NtsVal)
                obj.setup.max_iter = min(obj.setup.max_iterVal);
                if ~isinf(obj.setup.MaxOptTime)
                    obj.setup.MaxOptTime = 0.2*obj.setup.NtsVal(NtsPos)*obj.setup.Ts;
                end
            else
                obj.setup.max_iter = max(obj.setup.max_iterVal);
                if ~isinf(obj.setup.MaxOptTime)
                    obj.setup.MaxOptTime = 0.2*obj.setup.NtsVal(NtsPos)*obj.setup.Ts;
                end
            end            
            
            obj = obj.dJ_cond_v5_function();             
            % define selcted freq: freq_sel=1 MAX freq_sel=2 MIN
            freq_sel = obj.init.Fselect;
            % define bound on freq (at least 2 due to Nyquist)
            freq_bound = obj.init.Fnyq;
            % set NtsVal depending on freqs
            if any(obj.init.freqs(:,end))
                % define freq on which calibrate the sampling time
                freq = freq_bound*obj.init.freqs(freq_sel,end); % Hz
                Ts_wv = 1/(freq); % s
                distance_min = max(1,ceil(Ts_wv/obj.setup.Ts));
            else
                distance_min = obj.setup.NtsVal(NtsPos);                
            end

            % update the minimum distance                        
            obj.setup.NtsVal(NtsPos) = distance_min;            
            
            % safety flag
            obj.init.distance_safe_flag = (distance < obj.init.safety_interval);           
            
            %%%% observer %%%%
            if  ( ~( (distance < obj.setup.NtsVal(NtsPos)) && (obj.init.distance_safe_flag) ) )

                if obj.setup.print
                    % Display iteration slengthtep
                    disp(['n window: ', num2str(obj.setup.w),'  n samples: ', num2str(obj.setup.Nts)])                    
                    disp(['N. optimisations RUN: ',num2str(obj.init.opt_counter)]);
                    disp(['N. optimisations SELECTED: ',num2str(obj.init.select_counter)]);                    
                end                                
               
                %%%% OUTPUT measurements - buffer of w elements
                % measures
                for term=1:obj.setup.J_nterm
                    for traj=1:obj.setup.Ntraj
                        obj.init.traj = traj;
                        obj.init.Y(traj).val(term,:,1:end-1) = obj.init.Y(traj).val(term,:,2:end);
                        obj.init.Y(traj).val(term,:,end) = y(traj).val(:,term);
                    end
                end

                % adaptive sampling                
                obj.init.Y_space(1:end-1) = obj.init.Y_space(2:end);
                obj.init.Y_space(end) = obj.init.ActualTimeIndex;
                obj.init.Y_space_full_story(end+1) = obj.init.ActualTimeIndex;
                
                obj.init.Nsaved = obj.init.Nsaved + 1;
                
                % store measure times
                obj.init.temp_time = [obj.init.temp_time obj.init.ActualTimeIndex];

                % check only on the first traj as the sampling is coherent
                % on the 2.
                % WaitAlBuffer == 0 --> start asap
                if obj.setup.WaitAllBuffer == 0
                    cols_nonzeros = length(find(obj.init.Y_space ~= 0))*obj.setup.dim_out*nnz(obj.setup.J_temp_scale);                
                % WaitAlBuffer == 1 --> start when all the buffer is full
                elseif obj.setup.WaitAllBuffer == 1
                    cols_nonzeros = length(find(obj.init.Y_space ~= 0));  
                % WaitAlBuffer == 2 --> start at the last step of the trajectory
                elseif obj.setup.WaitAllBuffer == 2
                    cols_nonzeros = obj.setup.Nts;
                else
                    error('Wrong Wait Buffer Option')
                end

                % flag
                % WaitAlBuffer == 0 --> start asap
                if obj.setup.WaitAllBuffer == 0
                    flag = 2*length(obj.setup.opt_vars)+1; % Aeyels condition (see https://doi.org/10.48550/arXiv.2204.09359)
                % WaitAlBuffer == 1 --> start when all the buffer is full
                elseif obj.setup.WaitAllBuffer == 1
                    flag = obj.setup.w;
                % WaitAlBuffer == 2 --> start at the last step of the trajectory
                elseif obj.setup.WaitAllBuffer == 2
                    flag = obj.setup.Niter-obj.init.ActualTimeIndex;
                else
                    error('Wrong Wait Buffer Option')
                end
                % real
                if cols_nonzeros >= flag                                        

                    if obj.setup.WaitAllBuffer == 2
                        obj.setup.Niter = obj.init.ActualTimeIndex;
                        obj.setup.time = obj.setup.time(1:obj.init.ActualTimeIndex);
                        obj.init.break = 1;
                        obj.setup.w = numel(obj.init.Y_space_full_story)-1;
                        obj.init.Y_space = nonzeros(obj.init.Y_space_full_story)';
                        for traj=1:obj.init.params.Ntraj
                            obj.init.Y(traj).val = obj.init.Y_full_story(traj).val(:,:,obj.init.Y_space);
                        end                        
                    end

                    if obj.setup.forward

                        % get the maximum distance
                        first_nonzero = find(obj.init.Y_space,1,'first');
                        Y_space_nonzero = obj.init.Y_space(first_nonzero:end);
                        max_dist = max(diff(Y_space_nonzero));
                        if isempty(max_dist)
                            max_dist = 1;
                        end
                        
                        % setup buffers
                        n_samples = min(length(obj.init.Y_space_full_story)-1,obj.setup.w);
                        buf_Y_space_full_story = obj.init.Y_space_full_story(end-n_samples:end);
                        
                        %%%% FLUSH THE BUFFER IF SAFETY FLAG %%%%
                        if (obj.setup.FlushBuffer) && (max_dist >= obj.init.safety_interval)
                            
                            % buffer adaptive sampling: these buffers keep track of the
                            % time instants in which the measured data have been stored. 
                            step = 1*obj.setup.Nts;
                            start = obj.init.Y_space(end)-step*(obj.setup.w-1); 
                            stop = obj.init.Y_space(end);
                            obj.init.Y_space = start:step:stop;
                            
                            % down samplingbuffer
                            for traj=1:obj.setup.Ntraj
                                obj.init.traj = traj;
                                obj.init.Y(traj).val =  obj.init.Y_full_story(traj).val(:,:,obj.init.Y_space);
                            end
                            
                            % update full story
                            obj.init.Y_space_full_story(end) = [];
                            obj.init.Y_space_full_story = [obj.init.Y_space_full_story, obj.init.Y_space];
                            
                            % set again (used for back_time)
                            buf_Y_space_full_story = obj.init.Y_space_full_story(end-1*obj.setup.w+1:end);
                            
                            % back time index
                            buf_dist = diff(buf_Y_space_full_story);                        
                            obj.init.BackTimeIndex = obj.setup.time(max(obj.init.ActualTimeIndex-sum(buf_dist(1:end)),1));
                        else                            
                            % back time index
                            buf_dist = diff(buf_Y_space_full_story);
                            obj.init.BackTimeIndex = obj.setup.time(max(obj.init.ActualTimeIndex-sum(buf_dist(1:end)),1)); 
                        end
                        
                        
                        obj.init.BackIterIndex = find(obj.setup.time==obj.init.BackTimeIndex);                        

                        % set of initial conditions
                        for traj=1:obj.setup.Ntraj
                            obj.init.traj = traj;
                            % start from the correct one                             
                            obj.init.temp_x0_nonopt(traj).val = obj.init.X_est(traj).val(obj.setup.nonopt_vars,obj.init.BackIterIndex);
                            % change only the values which are to be optimised
                            % only 1 set of vars regardless to the number
                            % of trajectories used as we're not estimating
                            % the state or the model parameters
                            obj.init.temp_x0_opt = obj.init.X_est(1).val(obj.setup.opt_vars,obj.init.BackIterIndex);
                            
                            % if filters are used, their state shall be
                            % treated as non optimised vars as well
                            x0_filters = [];
                            filterstartpos = max(1,obj.init.BackIterIndex);
                            for nfilt=1:obj.setup.Nfilt
                                for dim=1:obj.setup.dim_out
                                    tmp = reshape(obj.init.X_filter(traj).val{nfilt,dim}(:,filterstartpos),1,obj.setup.filterTF(nfilt).dim);                                    
                                    x0_filters = [x0_filters, tmp];
                                end
                            end
                            obj.init.temp_x0_filters(traj).val = x0_filters;
                            Lfilt = length(x0_filters);
                             
                            % reconstruct temp_x0 from opt/nonopt vars
                            obj.init.temp_x0(traj).val = zeros(obj.setup.dim_state,1);
                            obj.init.temp_x0(traj).val(obj.setup.opt_vars) = obj.init.temp_x0_opt;
                            obj.init.temp_x0(traj).val(obj.setup.nonopt_vars) = obj.init.temp_x0_nonopt(traj).val;
                            obj.init.temp_x0(traj).val(end+1:end+Lfilt) = obj.init.temp_x0_filters(traj).val;
                        end
                        
                        % set target
                        obj = obj.target();
                        for traj = 1:obj.setup.Ntraj
                            obj.init.traj = traj;
                            nonzero_space = find(obj.init.Y_space ~= 0);
                            nonzero_pos = obj.init.Y_space(nonzero_space);
                            obj.init.target_story(traj).val(:,:,nonzero_pos) = obj.init.target(traj).val(:,:,nonzero_space);
                        end
                        
                        %%% normalisation %%%
                        if (obj.setup.J_normalise) && (~obj.init.normalised) 
                            range = 1:obj.init.ActualTimeIndex;
                            for filt=1:obj.setup.J_nterm
                                for dim=1:length(obj.setup.dim_out_compare)
                                    for traj=1:obj.setup.Ntraj
                                        E = (obj.init.Yhat_full_story(traj).val(filt,obj.setup.dim_out_compare(dim),range) - ...
                                            obj.init.Y_full_story(traj).val(filt,obj.setup.dim_out_compare(dim),range)).^2;
                                        E = reshape(E,1,size(E,3));
                                        Emax(traj) = max(E);
                                        if Emax(traj) == 0 || (abs(Emax(traj)) < 1e-15)
                                            Emax(traj) = 1;
                                        end                                        
                                    end
                                    Emax = max(Emax);
                                    obj.init.scale_factor_scaled(obj.setup.dim_out_compare(dim),filt) = obj.init.scale_factor(obj.setup.dim_out_compare(dim),filt)/Emax;
                                end
                            end        
                            if (obj.setup.J_term_terminal) && (obj.setup.terminal_normalise)
                                E = vecnorm(obj.init.X_est(traj).val(obj.setup.terminal_states,range)');
                                E_scale = E/sum(E);
                                for dim=1:length(obj.setup.terminal_states)                                                                        
%                                     obj.init.scale_factor_scaled_terminal(dim) = obj.init.scale_factor(1,obj.setup.J_term_terminal_position)/(E_scale(dim));                                    
                                    obj.init.scale_factor_scaled_terminal(dim) = obj.init.scale_factor(1,obj.setup.J_term_terminal_position)/E(dim);
                                end    
                            elseif (obj.setup.J_term_terminal)
                                for dim=1:length(obj.setup.terminal_states)
                                    obj.init.scale_factor_scaled_terminal(dim) = obj.init.scale_factor(1,obj.setup.J_term_terminal_position);
                                end
                            end   
                            if (~isempty(obj.setup.terminal_weights)) && (obj.setup.J_term_terminal)
                                for dim=1:length(obj.setup.terminal_states)
                                    obj.init.scale_factor_scaled_terminal(dim) = obj.init.scale_factor_scaled_terminal(dim)*obj.setup.terminal_weights(dim);
                                end
                            end
                            obj.init.normalised = 1;
                        elseif (~obj.setup.J_normalise)
                            obj.init.scale_factor_scaled = obj.init.scale_factor;
                            if obj.setup.J_term_terminal
                                for dim=1:length(obj.setup.terminal_states)
                                    obj.init.scale_factor_scaled_terminal(dim) = obj.init.scale_factor(1,obj.setup.J_term_terminal_position)*obj.setup.terminal_weights(dim);
                                end
                            end
                        end
                        
                        % check fmin time (boundaries)
                        obj.setup.opt_temp_time = tic;
                        obj.setup.MaxOptTimeFlag = 0;
                        
                        % save max times
                        obj.init.MaxOptTime_story = [obj.init.MaxOptTime_story obj.setup.MaxOptTime];
                        obj.init.MaxIter_story = [obj.init.MaxIter_story obj.setup.max_iter];
                        
                        if (obj.setup.optimise)
                            
                            % flag for first opt
                            obj.init.FirstOpt = 1;
                            
                            % save J before the optimisation to confront it later 
                            [J_before, obj_tmp] = obj.setup.cost_run(obj.init.temp_x0_opt,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target);


                            % Optimisation (only if distance_safe_flag == 1)
                            opt_time = tic;                        

                            %%%%% OPTIMISATION - NORMAL MODE %%%%%%                            
                            % run optimization
                            if obj.setup.MultiStart                                                                 
                                % init multistart
                                ms = MultiStart('FunctionTolerance',obj.init.TolFun, 'XTolerance', obj.init.TolX, 'UseParallel',false);
                            end                                                                   
                            % create problem
                            try
                                problem = createOptimProblem(func2str(obj.setup.fmin),'objective',@(x)obj.setup.cost_run(x,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target,1),...
                                                                            'x0', obj.init.temp_x0_opt, 'Aineq', obj.setup.Acon, 'bineq', obj.setup.Bcon, 'Aeq', obj.setup.Acon_eq, 'beq', obj.setup.Bcon_eq, ...
                                                                            'lb', obj.setup.LBcon, 'ub', obj.setup.UBcon, 'nonlcon', @(x)obj.setup.NONCOLcon(x,obj.init.temp_x0_nonopt,obj), 'options', obj.init.myoptioptions);
                                if ~obj.setup.MultiStart
                                    [NewXopt, J] = obj.setup.fmin(problem);
                                else
                                    [NewXopt, J] = run(ms,problem,obj.init.params.tpoints);                                                                
                                end
                                
                            catch
                                problem = createOptimProblem('fmincon','objective',@(x)obj.setup.cost_run(x,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target,1),...
                                                                            'x0', obj.init.temp_x0_opt, 'Aineq', obj.setup.Acon, 'bineq', obj.setup.Bcon, 'Aeq', obj.setup.Acon_eq, 'beq', obj.setup.Bcon_eq, ...
                                                                            'lb', obj.setup.LBcon, 'ub', obj.setup.UBcon, 'nonlcon', @(x)obj.setup.NONCOLcon(x,obj.init.temp_x0_nonopt,obj), 'options', obj.init.myoptioptions);
                                                                        
                                if strcmp(func2str(obj.setup.fmin),'patternsearch')                                                             
                                    problem.solver = 'patternsearch';   
                                    obj.init.myoptioptions.ConstraintTolerance = 1e-10;
                                    obj.init.myoptioptions.ScaleMesh = 'off';
                                    obj.init.myoptioptions.MaxMeshSize = 100;
                                    obj.init.myoptioptions.InitialMeshSize = 100;
                                    obj.init.myoptioptions.Display = 'iter';
                                    obj.init.myoptioptions.Algorithm = 'nups-gps';
                                    obj.init.myoptioptions.UseParallel = true;                                    
                                    obj.init.myoptimoptions.UseCompletePoll = true;
                                elseif strcmp(func2str(obj.setup.fmin),'fminsearchcon')   
                                    problem = createOptimProblem('fmincon','objective',@(x)obj.setup.cost_run(x,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target,1),...
                                                                            'x0',  obj.init.temp_x0_opt, 'lb', obj.setup.LBcon, 'ub', obj.setup.UBcon, 'Aeq', obj.setup.Acon_eq, 'beq', obj.setup.Bcon_eq, ...
                                                                            'nonlcon', @(x)obj.setup.NONCOLcon(x,obj.init.temp_x0_nonopt,obj), 'options', obj.init.myoptioptions);
                                    problem = rmfield(problem,'bineq');
                                    problem = rmfield(problem,'Aineq');   
                                    obj.init.myoptioptions.ConstraintTolerance = 1e-10;
                                    problem.options = obj.init.myoptioptions;
                                    problem.solver = 'fminsearchcon';                                    
                                else    
                                    error('WRONG OPTIMISATION SETUP');
                                end
                                
                                problem.options = obj.init.myoptioptions;  
                                
                                if ~obj.setup.MultiStart
                                    try
                                        [NewXopt, J] = obj.setup.fmin(problem);
                                    catch ME
                                        [NewXopt, J] = obj.setup.fmin(problem.objective,problem.x0,problem.lb,problem.ub,problem.Aeq,problem.beq,problem.nonlcon,problem.options);
                                    end
                                else
                                    list = obj.init.params.tpoints.list;
                                    for pp = 1:obj.init.params.tpoints.NumStartPoints
                                        problem.x0 = list(pp,:);
                                        [J_before(pp), obj_tmp] = obj.setup.cost_run(problem.x0,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target);   
                                        try
                                            [NewXopt(pp,:), J(pp,:)] = obj.setup.fmin(problem);
                                        catch ME
                                            [NewXopt(pp,:), J(pp,:)] = obj.setup.fmin(problem.objective,problem.x0,problem.lb,problem.ub,problem.Aeq,problem.beq,problem.nonlcon,problem.options);
                                        end
                                    end
                                    %J_improve = J./J_before';
                                    [J,pos] = min(J);
                                    J_before = J_before(pos);
                                    NewXopt = NewXopt(pos,:);
                                end
                            end
                            
                            % reconstruct NewXopt from opt/nonopt vars
                            NewXopt_tmp = [];
                            for traj = 1:obj.setup.Ntraj
                                obj.init.traj = traj;
                                NewXopt_end = zeros(obj.setup.dim_state,1);
                                NewXopt_end(obj.setup.opt_vars) = NewXopt;
                                NewXopt_end(obj.setup.nonopt_vars) = obj.init.temp_x0_nonopt(traj).val;                          
                                NewXopt_tmp(traj).val = NewXopt_end;                           
                            end
                            % set new state
                            NewXopt = NewXopt_tmp;                                                

                            % opt counter
                            if traj == 1
                                obj.init.opt_counter = obj.init.opt_counter + 1;
                            end

                            % check J_dot condition
                            J_diff = (J/J_before);

                            if (obj.setup.AlwaysOpt) || ( (J_diff <= obj.setup.Jdot_thresh) || (distance > obj.init.safety_interval) )                                                        

                                % on each trajectory
                                for traj=1:obj.setup.Ntraj

                                    % set traj
                                    obj.init.traj = traj;

                                    % update state
                                    obj.init.X_est(traj).val(:,obj.init.BackIterIndex) = NewXopt(traj).val;

                                    % store measure times
                                    obj.init.opt_chosen_time = [obj.init.opt_chosen_time obj.init.ActualTimeIndex];
                                    obj.init.just_optimised = 1;

                                    % counters
                                    obj.init.jump_flag = 0;
                                    obj.init.select_counter = obj.init.select_counter + 1;

                                    x_propagate = NewXopt(traj).val;

                                    % update params
                                    obj.init.params = obj.setup.params.params_update(obj.init.params,x_propagate);

                                    %%%%%%%%%%%%%%%%% FIRST MEASURE UPDATE %%%%%%%%
                                    % manage measurements
                                    back_time = obj.init.BackIterIndex;

                                    %%%% ESTIMATED measurements
                                    % measures       
                                    % NB: the output storage has to be done in
                                    % back_time+1 as the propagation has been
                                    % performed 
                                    Yhat = obj.setup.measure(x_propagate,obj.init.params,obj.setup.time(back_time),obj.init.input_story(traj).val(:,back_time),obj);
                                    % get filters - yhat
                                    obj.init.Yhat_full_story(traj).val(1,:,back_time) = Yhat;  
                                    tspan_pos = [max(1,back_time-1), back_time];
                                    [dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,tspan_pos,obj.init.X_filter_est(obj.init.traj));                                
                                    yhat(traj).val = Yhat;
                                    for filt=1:obj.setup.Nfilt
                                        for dim=1:obj.setup.dim_out
                                            y_tmp(dim,1) = dyhat{filt,dim}.val(end);
                                            obj.init.X_filter_est(traj).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                                        end
                                        yhat(traj).val = [yhat(traj).val, y_tmp];
                                    end


                                    for term=1:obj.setup.J_nterm
                                        obj.init.Yhat_full_story(traj).val(term,:,back_time) = yhat(traj).val(:,term);
                                    end
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                                    %%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%
                                    n_iter_propagate = obj.init.ActualTimeIndex-back_time;
                                    if ~strcmp(func2str(obj.setup.ode),'odeLsim')
                                        
                                        for j=1:n_iter_propagate
                                            
                                            % back time
                                            back_time = obj.init.BackIterIndex+j;
                                            tspan = obj.setup.time(back_time-1:back_time);                                            

                                            % how do you handle the input?
                                            obj.init.params.ActualTimeIndex = back_time; % here you have the -1 because BackIterIndex is differently set up than in measure_function                                                                         

                                            % integrate
                                            obj.init.t_ode_start = tic;                     
                                            X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params, obj), tspan, x_propagate,obj.setup.odeset);                                    
                                            x_propagate = X.y(:,end);                      
                                            obj.init.X_est(traj).val(:,back_time) = x_propagate;
                                            obj.init.t_ode(end+1) = toc(obj.init.t_ode_start);
                                            
                                        end
                                        
                                    else
                                        
                                        tspan = obj.setup.time(back_time:obj.init.ActualTimeIndex);
                                        % integrate
                                        obj.init.t_ode_start = tic;                     
                                        X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params, obj), tspan, x_propagate,obj.setup.odeset);                                    
                                        x_propagate = X.y(:,end);                      
                                        obj.init.X_est(traj).val(:,back_time:obj.init.ActualTimeIndex) = X.y;
                                        obj.init.t_ode(end+1) = toc(obj.init.t_ode_start);
                                        
                                    end

                                    for j =1:n_iter_propagate                                                                                

                                        % back time
                                        back_time = obj.init.BackIterIndex+j;                                        

                                        % how do you handle the input?
                                        obj.init.params.ActualTimeIndex = back_time; % here you have the -1 because BackIterIndex is differently set up than in measure_function                                          
                                        
                                        if strcmp(func2str(obj.setup.ode),'odeLsim')                                        
                                            x_propagate = X.y(:,back_time);
                                        else
                                            x_propagate = obj.init.X_est(traj).val(:,back_time);
                                        end

                                        %%%% ESTIMATED measurements
                                        % measures       
                                        % NB: the output storage has to be done in
                                        % back_time+1 as the propagation has been
                                        % performed 
                                        Yhat = obj.setup.measure(x_propagate,obj.init.params,obj.setup.time(back_time),obj.init.input_story(traj).val(:,back_time-1),obj);
                                        % get filters - yhat
                                        obj.init.Yhat_full_story(traj).val(1,:,back_time) = Yhat;            
                                        tspan_pos = [max(1,back_time-1), back_time];
                                        [dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,tspan_pos,obj.init.X_filter_est(obj.init.traj));                                
                                        yhat(traj).val = Yhat;
                                        for filt=1:obj.setup.Nfilt
                                            for dim=1:obj.setup.dim_out
                                                y_tmp(dim,1) = dyhat{filt,dim}.val(end);
                                                obj.init.X_filter_est(traj).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                                            end
                                            yhat(traj).val = [yhat(traj).val, y_tmp]; 
                                        end                                    
                                        for term=1:obj.setup.J_nterm
                                            obj.init.Yhat_full_story(traj).val(term,:,back_time) = yhat(traj).val(:,term);                                        
                                        end
                                    end
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                                    obj.init.Jstory(1,end+1) = J;
                                    if obj.setup.Jterm_store
                                        obj.init.Jterm_story(:,end+1) = obj_tmp.init.Jterm_store;
                                    end                              
                                end
                            else
                                % on each trajectory
                                for traj=1:obj.setup.Ntraj
                                    obj.init.traj = traj;
                                    % keep the initial guess
                                    obj.init.X_est(traj).val(obj.setup.opt_vars,obj.init.BackIterIndex) = obj.init.temp_x0_opt;
                                end

                                % restore params
                                obj.init.params = obj.setup.params.params_update(obj.init.params,obj.init.temp_x0(traj).val);                                                        
                            end

                            % stop time counter
                            obj.init.opt_time(end+1) = toc(opt_time);

                        end
                    end

                else                    

                end

                if obj.setup.print
                    clc;
                end                
            else
            end            
        end
    end
end
