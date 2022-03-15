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
classdef obsopt_v1003 < handle
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
        function obj = obsopt_v1003(varargin)
            
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
            
            % enable or not the buffer flush on the adaptive sampling
            if any(strcmp(varargin,'FlushBuffer'))
                pos = find(strcmp(varargin,'FlushBuffer'));
                obj.setup.FlushBuffer = (obj.setup.AdaptiveSampling && varargin{pos+1});
            else
                obj.setup.FlushBuffer = 0;
            end
            
            % option to define the safety interval in adaptive sampling. If
            % AdaptiveSampling flag is zero the histeresis is set to zero,
            % disabling the adaptive sampling
            if any(strcmp(varargin,'AdaptiveHist')) && (obj.setup.AdaptiveSampling)
                pos = find(strcmp(varargin,'AdaptiveHist'));
                tmp = varargin{pos+1};
                obj.setup.dJ_low = tmp(1);
                obj.setup.dJ_high = tmp(2);
                obj.setup.dPE = tmp(3);
            else
                obj.setup.dJ_low = 0;
                obj.setup.dJ_high = 0;
                obj.setup.dPE = -1;
            end
            
            % get the maximum number of iterations in the optimisation
            % process. Default is 100.
            if any(strcmp(varargin,'MaxIter'))
                pos = find(strcmp(varargin,'MaxIter'));
                obj.setup.max_iter = varargin{pos+1};
            else
                obj.setup.max_iter = 100;
            end
            
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
                obj.setup.Nts = varargin{pos+1};
            else
                obj.setup.Nts = 3;
            end
            
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
            
            % change max iterations depending on PE 
            if any(strcmp(varargin,'PE_maxiter'))
                pos = find(strcmp(varargin,'PE_maxiter'));
                obj.setup.PE_maxiter = varargin{pos+1};
            else
                obj.setup.PE_maxiter = 0;
            end

            % get the optimisation method. Default is fminsearch from
            % MATLAB
            if any(strcmp(varargin,'opt'))
                pos = find(strcmp(varargin,'opt'));
                obj.setup.fmin = varargin{pos+1};
            else
                obj.setup.fmin = @fminsearch;
            end
            
            % handle fmincon
            try
                test = func2str(obj.setup.fmin);
            catch
                test = 'null';
            end
            if strcmp(test,'fmincon')
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
                    pos = find(strcmp(varargin,'LBcon'));
                    obj.setup.LBcon = varargin{pos+1};
                else
                    obj.setup.LBcon = [];
                end
                if any(strcmp(varargin,'UBcon'))
                    pos = find(strcmp(varargin,'UBcon'));
                    obj.setup.UBcon = varargin{pos+1};
                else
                    obj.setup.UBcon = [];
                end
                if any(strcmp(varargin,'NONCOLcon'))
                    pos = find(strcmp(varargin,'NONCOLcon'));
                    obj.setup.NONCOLcon = varargin{pos+1};
                else
                    obj.setup.NONCOLcon = [];
                end
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
            
            % number of reference trajectories (>1 for control design)
            if any(strcmp(varargin,'control_design'))
                pos = find(strcmp(varargin,'control_design'));
                obj.setup.control_design = varargin{pos+1};
            else
                obj.setup.control_design = 0;
            end
            
            % reference model
            if any(strcmp(varargin,'model_reference')) && (obj.setup.control_design)
                pos = find(strcmp(varargin,'model_reference'));
                obj.setup.model_reference = varargin{pos+1};
            else
                obj.setup.model_reference = obj.setup.model;
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
            
            % get the spring like term in the cost function
            if any(strcmp(varargin,'spring'))
                pos = find(strcmp(varargin,'spring'));
                obj.setup.J_term_spring = 1;
                obj.setup.J_temp_scale = [obj.setup.J_temp_scale, varargin{pos+1}];
                obj.setup.J_term_spring_position = length(obj.setup.J_temp_scale);
                obj.setup.J_nterm_total = obj.setup.J_nterm_total+1;
            else
                obj.setup.J_term_spring = 0;
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
            
            % number of reference trajectories (>1 for control design)
            if (obj.setup.control_design)
                obj.setup.Ntraj = params.Ntraj;
            else
                obj.setup.Ntraj = 1;
            end
            
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
            
            % get initial state
            obj.init.X = obj.setup.X;
            obj.init.X_est = obj.setup.X_est;
            obj.init.X_est_runtime = obj.setup.X_est;
            obj.init.X_wrong = obj.setup.X_est;            
            for traj=1:obj.setup.Ntraj
                obj.init.X_filter(traj).val = cell(obj.setup.Nfilt,1);
                obj.init.X_filter_est(traj).val = cell(obj.setup.Nfilt,1);
                for i=1:obj.setup.Nfilt
                    obj.init.X_filter(traj).val{i} = zeros(obj.setup.filterTF(i).dim,obj.setup.Niter);
                    obj.init.X_filter_est(traj).val{i} = zeros(obj.setup.filterTF(i).dim,obj.setup.Niter);
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
            obj.init.matrix_str = tmp_str;
            
            % create scale factor, namely the weight over time for all the
            % cost function terms. In V1.1 no forgetting factor is
            % implemented. 
            obj = obj.scale_factor();
            obj.init.normalised = 0;
            
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
            end

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
            obj.init.last_opt_time = 0;
            obj.init.opt_time = 0;
            
            % set options
            if obj.setup.print 
                obj.init.display = 'iter';
            else
                obj.init.display = 'off'; 
            end
    
            
            % optimset      
            if strcmp(func2str(obj.setup.fmin),'fmincon')
                obj.init.myoptioptions = optimoptions('fmincon', 'MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                      'OptimalityTolerance', 0, 'StepTolerance', 0,'MaxFunEvals',Inf, 'GradObj', 'off',...
                                                      'OutputFcn',@obj.outfun,'TolFun',obj.init.TolFun,'TolX',obj.init.TolX);  
            elseif strcmp(func2str(obj.setup.fmin),'fminunc')
                obj.init.myoptioptions = optimoptions('fminunc', 'MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                      'OptimalityTolerance', 0, 'StepTolerance', 0,'MaxFunEvals',Inf, 'GradObj', 'off',...
                                                      'OutputFcn',@obj.outfun,'TolFun',obj.init.TolFun,'TolX',obj.init.TolX);  
            elseif strcmp(func2str(obj.setup.fmin),'fminsearch')
                obj.init.myoptioptions = optimset('MaxIter', obj.setup.max_iter, 'display',obj.init.display, ...
                                                  'MaxFunEvals',Inf,...
                                                  'OutputFcn',@obj.outfun,'TolFun',obj.init.TolFun,'TolX',obj.init.TolX); 
            end                                              
            %%% end of optimisation setup %%%

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
            if (optimValues.iteration == obj.setup.max_iter) || (optimValues.fval <= obj.init.TolFun)
                stop = true;
            else
                stop = false;
            end
        end
        
        function [J_final,obj] = cost_function(obj,varargin) 
            
            obj.init.t_J_start = tic;  
            
            % above ntraj init
            J_final = 0;
                
            for Nstart = 1:obj.setup.Ntraj
                
                % cost function init
                Jtot = 0;
                
                % get opt state
                x_opt = varargin{1};

                % non optimised vals
                x_nonopt = varargin{2}(Nstart).val;
                
                % x filter vals
                x_filters = varargin{3}(Nstart).val;
                Lfilt = length(x_filters);

                % create state
                x = zeros(obj.setup.dim_state+Lfilt,1);
                x(obj.setup.opt_vars) = x_opt;
                x(obj.setup.nonopt_vars) = x_nonopt;
                x(obj.setup.dim_state+1:end) = x_filters;
                
                % update params
                obj.init.params = obj.setup.params.params_update(obj.init.params,x);
                
                % get desired trajectory
                y_target = varargin{4}(Nstart).val;
                
                %optimization vector  
                X = x;                

                n_item = length(find((obj.init.Y_space)));
                n_iter = n_item;

                % offset 
                shift = obj.setup.w-n_item;

                % init Jterm store
                obj.init.Jterm_store = zeros(obj.setup.J_nterm_total,1);

                for j=1:n_iter

                    %evaluate the weighted in time cost function at this iteration time
                    if(obj.setup.forward ~= 1) %backward        
                        %%%%%% TO BE DONE %%%%%%
                    else

                        % get measure                          
                        [Yhat, filter_end] = obj.measure_function(X,j,obj.init.Yhat_full_story(Nstart).val);

                        J = zeros(obj.setup.J_nterm,size(Yhat,1));

                        for term=1:obj.setup.J_nterm
                            % get the J
                            for i=1:obj.setup.dim_out
                                diff = (y_target(term,i,shift+j)-Yhat(i,term));
                                J(term,i) = (diff)^2;
                            end
                                                        
                        end                           
                                            
                        % scaling
                        J_scaled = zeros(size(J));
                        for dim=1:obj.setup.dim_out                            
                            J_scaled(:,dim) = transpose(obj.init.scale_factor_scaled(dim,1:obj.setup.J_nterm)).*J(:,dim);
                            Jtot = Jtot + sum(J_scaled(:,dim)); 
                        end

                        % store terms
                        obj.init.Jterm_store(1:obj.setup.J_nterm) = obj.init.Jterm_store(1:obj.setup.J_nterm) + sum(J_scaled,2);
                    end

                end

                %%% spring like term %%%
                if ~isempty(obj.setup.estimated_params) && obj.setup.J_term_spring
                    x0 = obj.init.temp_x0(1).val;
                    params0 = x0(obj.setup.estimated_params);
                    paramsNow = x(obj.setup.estimated_params);
                    paramsDiff = params0-paramsNow;
                    paramsDiff = reshape(paramsDiff,1,length(obj.setup.estimated_params));
                    Jspring = paramsDiff*obj.init.scale_factor(1,obj.setup.J_term_spring_position)*transpose(paramsDiff);
                    
                    % store terms
                    obj.init.Jterm_store(end+1) = Jspring; 
                else
                    Jspring = 0;
                end
                             
                
                % non opt vars barrier term
                diff = x(obj.setup.nonopt_vars) - x_nonopt(obj.setup.nonopt_vars);
                J_barr = 1e8*any(diff)*0;
                
                J_final = J_final + Jtot + Jspring + J_barr;

                %%% final stuff %%%
                if n_iter > 0
                    obj.init.Yhat_temp = Yhat;
                else
                    J_final = 1; 
                end
            end
            
            obj.init.t_J(end+1) = toc(obj.init.t_J_start);  
        end

        
        % get measure: from the setup.measure imported function, this
        % method computes the filters defined in setup.temp_scale as well
        function [y_read, x0_filter] = measure_function(obj,varargin)
            
            obj.init.t_meas_start = tic; 
            
            % get the state
            x_propagate = varargin{1};%(obj.setup.opt_vars);
            x_filters = varargin{1}(obj.setup.dim_state+1:end);
            W_buf_pos = varargin{2};
            
            % get buffers
            if nargin > 2
                buf_intY = varargin{3};
            end
                     
            %%%% sampling %%%%            
            % iterations
            buf_dist = (diff([obj.init.BackIterIndex, transpose(nonzeros(obj.init.Y_space))]));  
            n_iter = sum(buf_dist(1:W_buf_pos));
            
            % filter state
            x0_filter = cell(obj.setup.Nfilt,1);
            filterstartpos = max(1,obj.init.BackIterIndex-1);
            startpos = 0;
            for i=1:obj.setup.Nfilt            
               x0_filter{i}(:,filterstartpos) = x_filters(startpos+1:startpos+obj.setup.filterTF(i).dim);
               startpos = startpos + obj.setup.filterTF(i).dim;
            end
            
            %%% init measure in case of no iterations (starting from first instant with buffer (Schiller 2021))
            if 1 || n_iter == 0
                % get the measure
                y_read = obj.setup.measure(x_propagate,obj.init.params);

                % get filters - yhat
                buf_intY(1,:,obj.init.BackIterIndex) = y_read;                        
                tmp_str = ['[dy_read, x_filter] = obj.measure_filter(buf_intY,x0_filter,obj.init.BackIterIndex,',obj.init.matrix_str,');'];
                eval(tmp_str);

                % out
                for filt=1:obj.setup.J_nterm-1
                    y_read = [y_read, dy_read{filt}];
                    x0_filter{filt}(:,obj.init.BackIterIndex) = x_filter(filt).val;
                end
            end                        

            if obj.setup.forward                
                
                % the max has been added to at least save the current state if
                for i=1:n_iter  
                    
                    % integration
                    back_time = obj.init.BackIterIndex -1 + i;
                    t = obj.setup.time(back_time);

                    % how do you handle the input?
                    obj.init.params.ActualTimeIndex = back_time+1;

                    % propagation
                    obj.init.t_ode_start = tic;
                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params, obj), obj.setup.tspan, x_propagate, obj.setup.odeset);
                    x_propagate = X.y(:,end);
                    obj.init.t_ode(end+1) = toc(obj.init.t_ode_start);
                    
                    % get the measure
                    obj.init.t_ode_start = tic;
                    y_read = obj.setup.measure(x_propagate,obj.init.params);
                    obj.init.t_meas(end+1) = toc(obj.init.t_ode_start);
                    
                    % get filters - yhat
                    buf_intY(1,:,back_time+1) = y_read;   
                    tmp_str = ['[dy_read, x_filter] = obj.measure_filter(buf_intY,x0_filter,back_time+1,',obj.init.matrix_str,');'];
                    eval(tmp_str);
                
                    % out
                    for filt=1:obj.setup.J_nterm-1
                        y_read = [y_read, dy_read{filt}];
                        x0_filter{filt}(:,back_time+1) = x_filter(filt).val;
                    end
                end
            else
                %%%%% TO BE DONE %%%%
            end
            
            obj.init.t_meas(end+1) = toc(obj.init.t_meas_start);
                        
        end
        
        % measure_filter: from setup.measure and setup.model this method
        % gets the propagated measures and computes the related filters,
        % defined by setup.temp_scale
        function [y_filter, x_filter] = measure_filter(varargin)
                                                  
            % get varargin
            obj = varargin{1};
            buf_y = varargin{2};
            x0_filter = varargin{3};
            pos = varargin{4};
            
            obj.init.t_filt_start = tic;
            
            % n filters
            Nfilter = obj.setup.J_nterm-1;
            filterstartpos = max(1,pos-1);            
            
            
            if Nfilter > 0
                for nfilt=1:Nfilter
                                        
                    A = varargin{4*nfilt+1};
                    B = varargin{4*nfilt+2};
                    C = varargin{4*nfilt+3};
                    D = varargin{4*nfilt+4};
                    
                    % numeric derivative
                    for k=1:obj.setup.dim_out                                             
                        
                        if (pos > 1)                              
                            % input 
                            u = zeros(1,2);
                            u(1) = buf_y(1,k,pos-1);
                            u(2) = buf_y(1,k,pos);
                                                        
                            % initial condition                                 
                            x0 = x0_filter{nfilt}(:,filterstartpos);
                            
                            % step model
                            [yk,xk] = discreteSS(x0,u,A,B,C,D);                                
                        else                               
                            dim_filter = size(obj.setup.filterTF(nfilt).B,1); 
                            yk = buf_y(nfilt+1,k,pos);
                            xk = x0_filter{nfilt}(:,filterstartpos);
                        end
                        
                        y_filter{nfilt}(k,1) = yk;
                        x_filter(nfilt).val = xk;
                    end
                end
            else
                y_filter = -1;
                x_filter = -1;
            end
            
            obj.init.t_filt(end+1) = toc(obj.init.t_filt_start);
            
        end
        
        % dJ_cond: function for the adaptive sampling
        function obj = dJ_cond_v5_function(obj)

            buffer_ready = (nnz(obj.init.Y_space) >= 2);

            if buffer_ready
                
                % init
                temp_e = 0;
                
                for traj = 1:obj.setup.Ntraj

                    % where to check
                    [~, pos_hat] = find(obj.init.Y_space > 1);
                    pos_hat = obj.init.Y_space(pos_hat)-1;

                    % Y measure - hat
                    Yhat = obj.init.Yhat_full_story(traj).val(1,:,pos_hat);
                    Yhat(1,:,end+1) = obj.init.Yhat_full_story(traj).val(1,:,obj.init.ActualTimeIndex);
                    Yhat = reshape(Yhat,size(Yhat,2),size(Yhat,3));
                                        
                    % Y measure - real
                    Y_buf = obj.init.Y_full_story(traj).val(1,:,pos_hat);
                    Y_buf(1,:,end+1) = obj.init.Y_full_story(traj).val(1,:,obj.init.ActualTimeIndex);
                    Y_buf = reshape(Y_buf,size(Y_buf,2),size(Y_buf,3));

                    % build the condition
                    n_sum = size(Y_buf,1);
                    
                    for i=1:n_sum
                        temp_e = temp_e + norm(Y_buf(i,:)-Yhat(i,:)); 
                    end                 
                end

                obj.init.dJ_cond = norm(temp_e);
            else
                obj.init.dJ_cond = 1.1*obj.setup.dJ_high;
            end
            
            obj.init.dJ_cond_story(:,obj.init.ActualTimeIndex) = obj.init.dJ_cond;
        end
        
        % Get Persistent Excitation and its derivative
        function obj = PE(obj)
            
            for traj = 1:obj.setup.Ntraj
                
                nonzero_meas = nnz(obj.init.Y(traj).val(1,:,:));
                
                if (obj.setup.w > 1) 
                    % Y measure
                    Y_buf = obj.init.Y(traj).val(1,:,:);
                    Y_buf(1,:,end+1) = obj.init.Y_full_story(traj).val(1,:,obj.init.ActualTimeIndex);
                    Y_buf = reshape(Y_buf,size(Y_buf,2),size(Y_buf,3));

                     %%% compute signal richness %%%             
                     obj.init.PE(traj).val(obj.init.ActualTimeIndex) = sum(abs(diff(sum(Y_buf,1).^2)));
                else
                     obj.init.PE(traj).val(obj.init.ActualTimeIndex) = 0;
                end
             
            end
             
             obj.init.maxIterStory(obj.init.ActualTimeIndex) = obj.setup.max_iter;
        end
        
        % target function (observer or control design)
        function obj = target(obj)          
            for i=1:obj.setup.Ntraj
                obj.init.target(i).val = obj.init.Y(i).val;
            end
        end
        
        % target function (observer or control design)
        function drive = drive(varargin) 
            
            obj = varargin{1};
            
            % y - varargin
%             y_star = varargin{2};
%             y_star = [y_star(1,:),y_star(2,:)]';
%             y = varargin{3};
%             y = [y(1,:),y(2,:)]';
%             drive = y_star-y;
            
            % x - varargin
            x_star = varargin{2}(1:obj.init.params.dim_state);            
            x = varargin{3}(1:obj.init.params.dim_state);            
            drive = x_star-x;
            
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
                % get measure
                yhat(traj).val = obj.setup.measure(xhat(traj).val,obj.init.params);
            end
            
            for traj=1:obj.setup.Ntraj
                obj.init.traj = traj;
                % get filters - y
                obj.init.Y_full_story(traj).val(1,:,obj.init.ActualTimeIndex) = y(traj).val;                       
                tmp_str = ['[dy, x_filter] = obj.measure_filter(obj.init.Y_full_story(traj).val,obj.init.X_filter(traj).val,obj.init.ActualTimeIndex,',obj.init.matrix_str,');'];
                eval(tmp_str);
                
                for filt=1:obj.setup.J_nterm-1
                    y(traj).val = [y(traj).val, dy{filt}];
                    obj.init.X_filter(traj).val{filt}(:,obj.init.ActualTimeIndex) = x_filter(filt).val;
                end
                
                % get filters - yhat
                obj.init.Yhat_full_story(traj).val(1,:,obj.init.ActualTimeIndex) = yhat(traj).val;
                tmp_str = ['[dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,obj.init.X_filter_est(traj).val,obj.init.ActualTimeIndex,',obj.init.matrix_str,');'];
                eval(tmp_str);
                for filt=1:obj.setup.J_nterm-1
                    yhat(traj).val = [yhat(traj).val, dyhat{filt}];
                    obj.init.X_filter_est(traj).val{filt}(:,obj.init.ActualTimeIndex) = x_filter(filt).val;
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
            
            obj = obj.dJ_cond_v5_function();
            obj = obj.PE();
            obj.init.distance_safe_flag = (distance < obj.init.safety_interval);
            %%%% select optimisation with hystheresis - dJcond %%%%%
            hyst_low = (obj.init.dJ_cond_story(max(1,obj.init.ActualTimeIndex-1)) < obj.setup.dJ_low) && (obj.init.dJ_cond >= obj.setup.dJ_low);
            hyst_high = (obj.init.dJ_cond >= obj.setup.dJ_high);
            % flag = all good, no sampling
            obj.init.hyst_flag = ~(hyst_low || hyst_high);
            %%%% select optimisation with hystheresis - PE %%%%%
            % flag = all good, no sampling
            obj.init.PE_flag = obj.init.PE(traj).val(obj.init.ActualTimeIndex) <= obj.setup.dPE;
            %%%% observer %%%%
            if  ( ~( ( (distance < obj.setup.Nts) || obj.init.hyst_flag || obj.init.PE_flag ) && (obj.init.distance_safe_flag) ) ) && (obj.setup.optimise)

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

                % store measure times
                obj.init.temp_time = [obj.init.temp_time obj.init.ActualTimeIndex];

                % check only on the first traj as the sampling is coherent
                % on the 2.
                if obj.setup.control_design == 0
                    cols_nonzeros = length(find(obj.init.Y_space ~= 0))*obj.setup.dim_out*nnz(obj.setup.J_temp_scale);                
                else
                    cols_nonzeros = length(find(obj.init.Y_space ~= 0));  
                end

                % flag
                if obj.setup.control_design == 0
                    flag = 2*length(obj.setup.opt_vars)+1;
                else
                    flag = obj.setup.w;
                end
                % real
                if cols_nonzeros >= flag

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
                            step = 2*obj.setup.Nts;
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
                            buf_Y_space_full_story = obj.init.Y_space_full_story(end-obj.setup.w+1:end);
                        end
                        
                        % back time index
                        buf_dist = diff(buf_Y_space_full_story);
                        obj.init.BackTimeIndex = obj.setup.time(max(obj.init.ActualTimeIndex-sum(buf_dist),1)); 
                        obj.init.BackIterIndex = find(obj.setup.time==obj.init.BackTimeIndex);

                        % set of initial conditions
                        for traj=1:obj.setup.Ntraj
                            obj.init.traj = traj;
                            % start from the correct one
                            if obj.setup.control_design 
                                obj.init.temp_x0_nonopt(traj).val = obj.init.X_est(traj).val(obj.setup.nonopt_vars,obj.init.BackIterIndex);
                            else
                                obj.init.temp_x0_nonopt(traj).val = obj.init.X(traj).val(obj.setup.nonopt_vars,obj.init.BackIterIndex);
                            end
                            % change only the values which are to be optimised
                            % only 1 set of vars regardless to the number
                            % of trajectories used as we're not estimating
                            % the state or the model parameters
                            obj.init.temp_x0_opt = obj.init.X_est(1).val(obj.setup.opt_vars,obj.init.BackIterIndex);
                            
                            % if filters are used, their state shall be
                            % treated as non optimised vars as well
                            x0_filters = [];
                            filterstartpos = max(1,obj.init.BackIterIndex-1);
                            for nfilt=1:obj.setup.Nfilt
                                tmp = reshape(obj.init.X_filter(traj).val{nfilt}(:,filterstartpos),1,obj.setup.filterTF(nfilt).dim);
                                x0_filters = [x0_filters, tmp];
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
                        if (~obj.init.normalised) 
                            range = 1:obj.init.ActualTimeIndex;
                            for filt=1:obj.setup.J_nterm
                                for dim=1:obj.setup.dim_out
                                    E = (obj.init.Yhat_full_story(traj).val(filt,dim,range) - obj.init.Y_full_story(traj).val(filt,dim,range)).^2;
                                    E = reshape(E,1,size(E,3));
                                    Emax = max(E);
                                    if Emax == 0
                                        Emax = 1;
                                    end
                                    obj.init.scale_factor_scaled(dim,filt) = obj.init.scale_factor(dim,filt)/Emax;
                                end
                            end                            
                            obj.init.normalised = 1;
                        end                       
                        
                        % save J before the optimisation to confront it later 
                        [J_before, obj_tmp] = obj.cost_function(obj.init.temp_x0_opt,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target);  

                        
                        % Optimisation (only if distance_safe_flag == 1)
                        opt_time = tic;
                        
                        %%%%% OPTIMISATION - NORMAL MODE %%%%%%
                        if strcmp(func2str(obj.setup.fmin),'fmincon')                                                       
                            [NewXopt, J, obj.init.exitflag,output] = obj.setup.fmin(@(x)obj.cost_function(x,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target,1),...
                                                                     obj.init.temp_x0_opt, obj.setup.Acon, obj.setup.Bcon,obj.setup.Acon_eq, obj.setup.Bcon_eq, obj.setup.LBcon,...
                                                                     obj.setup.UBcon, obj.setup.NONCOLcon, obj.init.myoptioptions);
                        else
                            [NewXopt, J, obj.init.exitflag,output] = obj.setup.fmin(@(x)obj.cost_function(x,obj.init.temp_x0_nonopt,obj.init.temp_x0_filters,obj.init.target,1),...
                                                                     obj.init.temp_x0_opt, obj.init.myoptioptions);
                        end
                        
                        % save numer of iterations
                        obj.init.NiterFmin(obj.init.ActualTimeIndex) = output.iterations;
                        obj.init.exitflag_story(obj.init.ActualTimeIndex) = obj.init.exitflag;
                        
                        % reconstruct NewXopt from opt/nonopt vars
                        NewXopt_tmp = [];
                        for traj = 1:obj.setup.Ntraj
                            obj.init.traj = traj;
                            NewXopt_end = zeros(obj.setup.dim_state,1);
                            NewXopt_end(obj.setup.opt_vars) = NewXopt;
                            NewXopt_end(obj.setup.nonopt_vars) = obj.init.temp_x0_nonopt(traj).val;                          
                            NewXopt_tmp(traj).val = NewXopt_end;
                             % wrap for pendulum
                            NewXopt_tmp(traj).val(1:2) = wrapToPi(NewXopt_tmp(traj).val(1:2));
                        end
                        NewXopt = NewXopt_tmp;

                        % opt counter
                        if traj == 1
                            obj.init.opt_counter = obj.init.opt_counter + 1;
                        end

                        % check J_dot condition
                        J_diff = (J/J_before);

                        if (obj.setup.AlwaysOpt) || ( (J_diff <= obj.setup.Jdot_thresh) || (distance > obj.init.safety_interval) )
                            
                            % update params
                            obj.init.params = obj.setup.params.params_update(obj.init.params,NewXopt(1).val);

                            % on each trajectory
                            for traj=1:obj.setup.Ntraj
                                obj.init.traj = traj;
                                % update
                                obj.init.X_est(traj).val(:,obj.init.BackIterIndex) = NewXopt(traj).val;
                                % store measure times
                                obj.init.opt_chosen_time = [obj.init.opt_chosen_time obj.init.ActualTimeIndex];
                                obj.init.just_optimised = 1;

                                % counters
                                obj.init.jump_flag = 0;
                                obj.init.select_counter = obj.init.select_counter + 1;

                                x_propagate = NewXopt(traj).val;

                                %%%%%%%%%%%%%%%%% FIRST MEASURE UPDATE %%%%%%%%
                                % manage measurements
                                back_time = obj.init.BackIterIndex;

                                %%%% ESTIMATED measurements
                                % measures       
                                % NB: the output storage has to be done in
                                % back_time+1 as the propagation has been
                                % performed 
                                Yhat = obj.setup.measure(x_propagate,obj.init.params);
                                % get filters - yhat
                                obj.init.Yhat_full_story(traj).val(1,:,back_time) = Yhat;     
                                tmp_str = ['[dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,obj.init.X_filter(traj).val,back_time,',obj.init.matrix_str,');'];
                                eval(tmp_str);
                                yhat(traj).val = Yhat;
                                for filt=1:obj.setup.J_nterm-1
                                    yhat(traj).val = [yhat(traj).val, dyhat{filt}];
                                    obj.init.X_filter_est(traj).val{filt}(:,back_time) = x_filter(filt).val;
                                end
                                for term=1:obj.setup.J_nterm
                                    obj.init.Yhat_full_story(traj).val(term,:,back_time) = yhat(traj).val(:,term);
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                                %%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%
                                n_iter_propagate = obj.init.ActualTimeIndex-obj.init.BackIterIndex;

                                for j =1:n_iter_propagate                                    
                                    
                                    % back time
                                    back_time = obj.init.BackIterIndex+j;
                                    t = obj.setup.time(back_time);                                    
                                    
                                    % how do you handle the input?
                                    obj.init.params.ActualTimeIndex = back_time; % here you have the -1 because BackIterIndex is differently set up than in measure_function

                                    % integrate
                                    obj.init.t_ode_start = tic;                     
                                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params, obj), obj.setup.tspan, x_propagate,obj.setup.odeset);
                                    x_propagate = X.y(:,end);                      
                                    obj.init.X_est(traj).val(:,back_time) = x_propagate;
                                    obj.init.t_ode(end+1) = toc(obj.init.t_ode_start);

                                    %%%% ESTIMATED measurements
                                    % measures       
                                    % NB: the output storage has to be done in
                                    % back_time+1 as the propagation has been
                                    % performed 
                                    Yhat = obj.setup.measure(x_propagate,obj.init.params);
                                    % get filters - yhat
                                    obj.init.Yhat_full_story(traj).val(1,:,back_time) = Yhat;            
                                    tmp_str = ['[dyhat, x_filter] = obj.measure_filter(obj.init.Yhat_full_story(traj).val,obj.init.X_filter_est(traj).val,back_time,',obj.init.matrix_str,');'];
                                    eval(tmp_str);
                                    yhat(traj).val = Yhat;
                                    for filt=1:obj.setup.J_nterm-1
                                        yhat(traj).val = [yhat(traj).val, dyhat{filt}];
                                        obj.init.X_filter_est(traj).val{filt}(:,back_time) = x_filter(filt).val;
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

                if obj.setup.print
                    clc;
                end

            end

            % on each trajectory
            for traj=1:obj.setup.Ntraj
                obj.init.traj = traj;
                % save measures
                obj.init.Y_est(traj).val = obj.init.X_est(traj).val(obj.setup.observed_state,:);
                obj.init.Y_est_runtime(traj).val = obj.init.X_est_runtime(traj).val(obj.setup.observed_state,:);
            end
        end        
        
        % plot results for control design
        function plot_section_control(obj,varargin)
            
            %%%% plot state estimation %%%
            figure(1)
            for i=1:obj.setup.plot_vars
                subplot(obj.setup.plot_vars,1,i);
                hold on
                grid on
                box on
                
                for traj=1:obj.setup.Ntraj
                    if strcat(obj.setup.DataType,'simulated')
                        plot(obj.setup.time,obj.init.X(traj).val(i,:),'b--');
                    end
                    plot(obj.setup.time,obj.init.X_est(traj).val(i,:),'r--');
%                     plot(obj.setup.time,obj.init.X_est_runtime(traj).val(i,:),'g');

                    if strcat(obj.setup.DataType,'simulated')
                        legend('True','Est')
%                         legend('Stored','Est','Runtime')
                    else
                        legend('Stored','Est','Runtime')
                    end
                end
            end
            
            %%%% plot state estimation error %%%
            figure(2)
            for i=1:obj.setup.plot_vars
                subplot(obj.setup.plot_vars,1,i);
                hold on
                grid on
                box on
                
                % plot
                est_error = obj.init.X(1).val(i,:) - obj.init.X_est_runtime(1).val(i,:);
                
                log_flag = 1;
                if ~log_flag
                    plot(obj.setup.time,est_error,'k','LineWidth',2);
                else
                    % log 
%                     set(gca, 'XScale', 'log')
                    set(gca, 'YScale', 'log')
                    plot(obj.setup.time,abs(est_error),'k','LineWidth',2);
                end
                
                xlabel('time [s]')
                ylabel(['\delta x_',num2str(i)])
            end
            
            %%%% plot state estimation error - norm%%%
            figure(3)
            hold on
            grid on
            box on

            % plot
            for iter=1:obj.setup.Niter
                est_error_norm(iter) = norm(obj.init.X(1).val(1:obj.init.params.dim_state,iter) - obj.init.X_est_runtime(1).val(1:obj.init.params.dim_state,iter));
            end

            log_flag = 1;
            if ~log_flag
                plot(obj.setup.time,est_error_norm,'k','LineWidth',2);
            else
                % log 
%                     set(gca, 'XScale', 'log')
                set(gca, 'YScale', 'log')
                plot(obj.setup.time,abs(est_error_norm),'k--','LineWidth',2);
            end

            xlabel('time [s]')
            ylabel('\delta x_norm')            
            
            
            %%%% plot windowed data %%%%
            figure(4)
            title('Sampled measured')
            ax = zeros(1,3);
            for k=1:obj.setup.dim_out
                
                % number fo subplots depending on the output dimension
                n_subplot = obj.setup.dim_out;
                
                % indicize axes
                ax_index = k;
                ax(ax_index)=subplot(n_subplot,1,ax_index);
                
                % hold on on plots
                hold on
                
                % dpwn sampling instants
                WindowTime = obj.setup.time(obj.init.temp_time);
                
                for traj=1:obj.setup.Ntraj
                    % plot true values
                    y_meas = reshape(obj.init.Y_full_story(traj).val(1,k,:),size(obj.setup.time));
                    plot(obj.setup.time,y_meas,'m--')
                    
                    % plot down sampling
                    data = reshape(obj.init.Yhat_full_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
%                     plot(WindowTime,data,'s','MarkerSize',5);

                    % plot target values
                    data = reshape(obj.init.target_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'bo','MarkerSize',5);

                    ylabel(strcat('y_',num2str(k)));
                    xlabel('simulation time [s]');
%                     legend('true','estimation','target')
                    legend('trajectory','measures')
                end
            end
            linkaxes(ax,'x');
            
            %%%% plot filters %%%%%
            figure(5)
            title('Filters on measures')
            ax = zeros(1,3);
            for k=1:obj.setup.J_nterm
                
                % number fo subplots depending on the Nterm
                n_subplot = obj.setup.J_nterm;
                
                % indicize axes
                ax_index = k;
                ax(ax_index)=subplot(n_subplot,1,ax_index);
                
                % plot
                hold on
                for traj=1:obj.setup.Ntraj
                    for dim=1:obj.setup.dim_out
                        y_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Y_full_story(traj).val(k,dim,:),size(obj.setup.time));
                        yhat_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Yhat_full_story(traj).val(k,dim,:),size(obj.setup.time));
                        plot(obj.setup.time,y_plot,'b--');
                        plot(obj.setup.time,yhat_plot,'r--');
                    end
                    
                    ylabel(strcat('y_filter',num2str(k)));
                    xlabel('simulation time [s]');
                end            
                
            end
            linkaxes(ax,'x');
            
        end
    end
end
