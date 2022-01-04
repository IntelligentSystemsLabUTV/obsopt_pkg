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
classdef obsopt_general_adaptive_flush
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
        function obj = obsopt_general_adaptive_flush(varargin)
            
            if any(strcmp(varargin,'params'))
                pos = find(strcmp(varargin,'params'));
                params = varargin{pos+1};
                
                % get model from params
                obj.setup.model = params.model;
                
                % get measure from params
                obj.setup.measure = params.measure;
                
                % get the integration algorithm
                obj.setup.ode = params.ode;
                
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
                params.input = @(x,params) 0;
                
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
            else
                obj.setup.dJ_low = 0;
                obj.setup.dJ_high = 0;
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
                temp_scale = [1, 0, 0, 0];
            end
            % get if integral is used
            obj.setup.J_term_integral = (temp_scale(3) ~= 0);
            % this further computation gets the actual filters present. 
            obj.setup.J_temp_scale = transpose(nonzeros(temp_scale));
            % compute the actual number of terms in the cost function.
            % Maximum is 3 as only the y-filters terms are considered.
            % Other terms as the spring-like aren't taken into account here
            obj.setup.J_nterm = min(3,size(nonzeros(temp_scale(1:3)),1));
            % This variable instead taske into account the whole set of
            % terms. It's used to create the scale factor (see obs.scale_factor())
            obj.setup.J_nterm_total = size(nonzeros(temp_scale),1);
            
            % get spring like term in cost function
            obj.setup.J_term_spring = (temp_scale(4) ~= 0);
            obj.setup.J_term_spring_position = length(obj.setup.J_temp_scale);
            
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
            
            % create scale factor, namely the weight over time for all the
            % cost function terms. In V1.1 no forgetting factor is
            % implemented. 
            obj = obj.scale_factor();
            
            % Window Samples: maximum number of time instants considered in
            % the optimisation if fixed sampling is used. For more
            % information check the reference. 
            obj.init.WindowSamples = max(2,obj.setup.Nts*(obj.setup.w-1)+1);
            
            % get safety interval for the adaptive sampling
            obj.init.safety_interval = int32(obj.setup.safety_density*obj.init.WindowSamples);
            
            % BackIterIndex: t0 index (starting from 1). For more
            % information check the reference. 
            obj.init.BackIterIndex = 1;
            
            % derivative setup: buffer dimensions for numeric derivative
            % computation. Default is [1,3], which is kinda quick. To
            % numerically differentiate slow systems check
            % obj.derivative method documentation. 
            obj.init.c1_derivative = 1;
            obj.init.d1_derivative = 3;
            
            % buffer used for measured and estimated variables derivarive. 
            for i=1:obj.setup.Ntraj
                obj.init.buf_dY(i).val = zeros(obj.setup.dim_out,obj.init.d1_derivative);
                obj.init.buf_dYhat(i).val = zeros(obj.setup.dim_out,obj.init.d1_derivative);
            end
            
            
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
            
            % Broyden
            obj.init.A_fin = 1e-2*rand(obj.setup.w*obj.setup.dim_out*obj.setup.J_nterm,obj.setup.dim_state);
            obj.init.alpha = 1e-3;
            
            % time instants in which the optimisation is run
            obj.init.temp_time = [];
            % time instand in which the optimisation is accepted 
            % (see J_dot_thresh for more information)
            obj.init.opt_chosen_time = [];
            
            % cost function gradient memory buffer. Not implemented in V1.1
            obj.init.grad_story = zeros(obj.setup.dim_state,1);
            
            % optimisation counters. Related to temp_time and chosen_time
            obj.init.opt_counter = 0;
            obj.init.select_counter = 0;
            
            %%% start of optimisation setup %%%
            % optimset: check documentation for fminsearch or fminunc
            obj.init.TolX = 1e-10;
            obj.init.TolFun = 1e-7;
            obj.init.last_opt_time = 0;
            obj.init.opt_time = 0;
            
            % set options
            if obj.setup.print 
                obj.init.display = 'iter';
            else
                obj.init.display = 'off'; 
            end
    
            % optimset 
            obj.init.myoptioptions = optimset('MaxIter', obj.setup.max_iter,'display',obj.init.display,...
                                            'TolFun',obj.init.TolFun,'TolX',obj.init.TolX,'MaxFunEvals',Inf,'OutputFcn',@obj.outfun, 'GradObj', 'off');
            %%% end of optimisation setup %%%

        end
        
        % scale factor: this method defines the cost function weights
        % accordingly to the selected filters (see setup.temp_scale).
        function obj = scale_factor(obj)
            obj.init.scale_factor = obj.setup.J_temp_scale.*ones(obj.setup.dim_out,obj.setup.J_nterm_total);
        end
        
        % numeric derivative: this method computes the numeric derivative
        % of a signal. Open the method for more detailed information.
        function [buffer_out, dy] = derivative(obj,T,y,c,d,mediana,buffer)

        %T: sampling time (equally spaced samples)
        %y: actual data
        %c  the length of the two subwindows where the mean/median is evaluated
        %d  is total number of data considered to evaluate the pseudo-derivative, d>=2c
        %reset: =1 if the data buffer has to be discarded, 0 otherwise
        %mediana = 1 the median is evaluated on the burst of the c samples,
        %otherwise the mean
        % buffer = 1xd vector

        % (------d------)
        % (-c-)-----(-c-)
        % [---]-----[---]

        %FUNZIONE PER IL CALCOLO DELLA PSEUDODERIVATA
        %----------------------------------

        % global DynOpt

        %update the buffer
        for k=1:d-1
            buffer(k) = buffer(k+1);
        end
        buffer(d) = y;


        if(mediana==1)
            temp1 = median(buffer(1:c));
            temp2 = median(buffer(d-c+1:d));
        else
            temp1 = 0;
            for k=1:c
                temp1 = temp1 + buffer(k);
            end
            temp1 = temp1/c;

            temp2 = 0;
            for k=d-c+1:d
                temp2 = temp2 + buffer(k);
            end
            temp2 = temp2/c;
        end

        dy = (temp2-temp1) / (T*(d-c));
        buffer_out = buffer;

        end
        
        % outfun: this method check wether or not the optimisation process
        % shall be stopped or not. Check setup.J_thresh for more
        % information. 
        function stop = outfun(obj,x, optimValues, state)
            if (abs(optimValues.fval) < 1e-8)
                stop = 1;
            else
                stop = 0;
            end
        end
        
        % cost function: actual cost function. Check the reference for more 
        % information 
        function [Jtot,obj] = cost_function_measure(obj,varargin) 

            % get state
            x = varargin{1};
            
            % set the derivative buffer as before the optimisation process (multiple f computation)
            back_time = obj.init.BackIterIndex;
            % set the derivative buffer as before the optimisation process (multiple f computation)
            if back_time >= obj.init.d1_derivative
                temp_buf_dyhat = obj.init.Yhat_full_story(1,:,back_time-(obj.init.d1_derivative-1):back_time);
            else
                init_pos = obj.init.d1_derivative-back_time;
                stack = reshape(obj.init.Yhat_full_story(1,:,1:back_time),obj.setup.dim_out,back_time);
                temp_buf_dyhat = [zeros(obj.setup.dim_out,init_pos), stack];
            end
            
            % cost function init
            Jtot = 0;

            %optimization vector  
            X = x; 

            n_item = length(find((obj.init.Y_space)));
            n_iter = n_item;

            shift = obj.setup.w-n_item;
            
            % init Jterm store
            obj.init.Jterm_store = zeros(obj.setup.J_nterm_total,1);

            for j=1:n_iter

                %evaluate the weighted in time cost function at this iteration time
                if(obj.setup.forward ~= 1) %backward        
                    %%%%%% TO BE DONE %%%%%%
                else

                    % get measure
                    [temp_buf_dyhat, Yhat] = obj.measure_function(X,j,temp_buf_dyhat,obj.init.Yhat_full_story);

                    J = zeros(obj.setup.J_nterm,size(Yhat,1));

                    for term=1:obj.setup.J_nterm
                        % get the J
                        for i=1:obj.setup.dim_out
                            diff = (obj.init.Y(term,i,shift+j)-Yhat(i,term));
                            J(term,i) = (diff)^2;
                        end
                    end

                    for term=1:obj.setup.dim_out
                        tmp = obj.init.scale_factor(term,1:obj.setup.J_nterm)*J(:,term);
                        Jtot = Jtot + tmp; 
                    end

                    % store terms
                    obj.init.Jterm_store(1:obj.setup.J_nterm) = obj.init.Jterm_store(1:obj.setup.J_nterm) + J(:,term);
                end

            end
            
            %%% spring like term %%%
            if ~isempty(obj.setup.estimated_params) && obj.setup.J_term_spring
                x0 = obj.init.temp_x0;
                params0 = x0(obj.setup.estimated_params);
                paramsNow = x(obj.setup.estimated_params);
                paramsDiff = params0-paramsNow;
                paramsDiff = reshape(paramsDiff,1,length(obj.setup.estimated_params));
                Jspring = paramsDiff*obj.init.scale_factor(1,obj.setup.J_term_spring_position)*transpose(paramsDiff);
            else
                Jspring = 0;
            end
            
            % store terms
            obj.init.Jterm_store(end) = Jspring;
            
            Jtot = Jtot+Jspring;

            %%% final stuff %%%
            if n_iter > 0
                obj.init.Yhat_temp = Yhat;
            else
                Jtot = 1; 
            end
        end
        
        function [J_final,obj] = cost_function(obj,varargin) 
            
            % above ntraj init
            J_final = 0;
                
            for Nstart = 1:obj.setup.Ntraj
                
                % cost function init
                Jtot = 0;
                
                % get opt state
                x_opt = varargin{1};

                % non optimised vals
                x_nonopt = varargin{2}(Nstart).val;

                % create state
                x = zeros(obj.setup.dim_state,1);
                x(obj.setup.opt_vars) = x_opt;
                x(obj.setup.nonopt_vars) = x_nonopt;  
                
                % update params
                obj.init.params = obj.setup.params.params_update(obj.init.params,x);
                
                % get desired trajectory
                y_target = varargin{3}(Nstart).val;
            
                % set the derivative buffer as before the optimisation process (multiple f computation)
                back_time = obj.init.BackIterIndex;
                % set the derivative buffer as before the optimisation process (multiple f computation)
                if back_time >= obj.init.d1_derivative
                    temp_buf_dyhat = obj.init.Yhat_full_story(Nstart).val(1,:,back_time-(obj.init.d1_derivative-1):back_time);
                else
                    init_pos = obj.init.d1_derivative-back_time;
                    stack = reshape(obj.init.Yhat_full_story(Nstart).val(1,:,1:back_time),obj.setup.dim_out,back_time);
                    temp_buf_dyhat = [zeros(obj.setup.dim_out,init_pos), stack];
                end
              

                %optimization vector  
                X = x;                

                n_item = length(find((obj.init.Y_space)));
                n_iter = n_item;

                shift = obj.setup.w-n_item;

                % init Jterm store
                obj.init.Jterm_store = zeros(obj.setup.J_nterm_total,1);

                for j=1:n_iter

                    %evaluate the weighted in time cost function at this iteration time
                    if(obj.setup.forward ~= 1) %backward        
                        %%%%%% TO BE DONE %%%%%%
                    else

                        % get measure
                        [temp_buf_dyhat, Yhat] = obj.measure_function(X,j,temp_buf_dyhat,obj.init.Yhat_full_story(Nstart).val);

                        J = zeros(obj.setup.J_nterm,size(Yhat,1));

                        for term=1:obj.setup.J_nterm
                            % get the J
                            for i=1:obj.setup.dim_out
                                diff = (y_target(term,i,shift+j)-Yhat(i,term));
                                J(term,i) = (diff)^2;
                            end
                        end

                        for term=1:obj.setup.dim_out
                            tmp = obj.init.scale_factor(term,1:obj.setup.J_nterm)*J(:,term);
                            Jtot = Jtot + tmp; 
                        end

                        % store terms
                        obj.init.Jterm_store(1:obj.setup.J_nterm) = obj.init.Jterm_store(1:obj.setup.J_nterm) + J(:,term);
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
                else
                    Jspring = 0;
                end

                % store terms
                obj.init.Jterm_store(end) = Jspring;              
                
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
        end

        
        % get measure: from the setup.measure imported function, this
        % method computes the filters defined in setup.temp_scale as well
        function [buf_dy, y_read] = measure_function(obj,varargin)
            
            % get the state
            x_propagate = varargin{1};
            W_buf_pos = varargin{2};
            
            % get buffers
            if nargin > 2
                buf_dy = varargin{3};
                buf_intY = varargin{4};
            end
            
                        
            %%%% sampling %%%%
            buf_dist = diff([obj.init.BackIterIndex, transpose(nonzeros(obj.init.Y_space))]);
            n_iter = sum(buf_dist(1:W_buf_pos));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if obj.setup.forward
                % the max has been added to at least save the current state if
                for i=1:n_iter
                    
                    % integration
                    back_time = obj.init.BackIterIndex -1 + i;
                    
                    % how do you handle the input?
                    if obj.setup.control_design
                        obj.init.params.u = obj.setup.params.input(x_propagate,obj.init.params);
                    else
                        obj.init.params.u = obj.setup.params.input(obj.init.X(1).val(:,back_time),obj.init.params);
                    end
                    
                    % propagation
                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params), obj.setup.tspan, x_propagate);
                    x_propagate = X.y(:,end);
                    
                    % get the measure
                    y_read = obj.setup.measure(x_propagate,obj.init.params);
                    
                    % get filters - yhat
                    [buf_dy, dy_read, y_read_int] = obj.measure_filter(y_read,buf_dy,buf_intY,obj.init.BackIterIndex+i);
                    y_read = [y_read, dy_read, y_read_int];
                end
            else
                %%%%% TO BE DONE %%%%
            end
            
            
            
        end
        
        % measure_filter: from setup.measure and setup.model this method
        % gets the propagated measures and computes the related filters,
        % defined by setup.temp_scale
        function [buf_der, dy, y_int] = measure_filter(obj,y,buf_der,buf_int,current_pos)
            
            % reshape buffer
            buf_der = reshape(buf_der,obj.setup.dim_out,obj.init.d1_derivative);
            
            % numeric derivative
            dy = zeros(obj.setup.dim_out,1);
            for k=1:obj.setup.dim_out
                [buf_der(k,:), dy(k)] = obj.derivative(obj.setup.Ts,y(k),obj.init.c1_derivative,obj.init.d1_derivative,0,buf_der(k,:));
            end
           
            % integral initial condition
            temp_pos = max(1,current_pos-1);
            if (obj.setup.J_term_integral) && (temp_pos > 1)
                % reset the integral
                if obj.setup.J_int_reset
                    % restart after Nw*Nts
                    modulus = cast(floor(current_pos/obj.init.WindowSamples),'uint32')-1;
                    start = max(1,modulus*obj.init.WindowSamples);
                else
                    % start from the beginning
                    start = 1;
                end
                temp = buf_int(1,:,start:temp_pos-1); 
                y_int = reshape(temp,obj.setup.dim_out,size(temp,3));
                
                % integral
                for i = 1:obj.setup.dim_out
                    time = obj.setup.time(start:temp_pos);
                    y_int(i) = trapz(time,[y_int(i,:), y(i)]);
                end
            else
                y_int = zeros(obj.setup.dim_out,1); 
            end
            
        end
        
        % dJ_cond: function for the adaptive sampling
        function obj = dJ_cond_v5_function(obj)

%             buffer_ready = (nnz(obj.init.Y_space) >= 1) && (size(obj.init.Yhat_full_story(1).val,3) > obj.setup.Nts);
            buffer_ready = (nnz(obj.init.Y_space) > obj.setup.w);

            if buffer_ready
                
                % init
                temp_e = 0;
                
                for traj = 1:obj.setup.Ntraj

                    % where to check
                    [~, pos_hat] = find(obj.init.Y_space ~= 0);
                    pos_hat = obj.init.Y_space(pos_hat)-1;

                    % Y measure
                    Yhat = obj.init.Yhat_full_story(traj).val(1,:,pos_hat);
                    Y_buf = obj.init.Y_full_story(traj).val(1,:,pos_hat);

                    % Y derivative measure
                    if obj.setup.J_nterm > 1
                        Yhat_der = obj.init.Yhat_full_story(traj).val(2,:,pos_hat);
                        Y_buf_der = obj.init.Y_full_story(traj).val(2,:,pos_hat);
                    end

                    % build the condition
                    n_sum = size(Y_buf,2);
                    
                    for i=1:n_sum
                        temp_e = temp_e + norm(Y_buf(:,i)-(Yhat(:,i))); 
                        if obj.setup.J_nterm > 1
                           temp_e = temp_e + + norm(Y_buf_der(:,i)-(Yhat_der(:,i))); 
                        end
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
            
             %%% compute signal richness %%%
             obj.init.PE_num(traj).val(obj.init.ActualTimeIndex) = trapz(obj.setup.Ts,obj.init.Y(traj).val(1,:).^2);
             
             [obj.init.buf_PE(traj).val, dPE] = obj.derivative(obj.setup.Ts,obj.init.PE_num(traj).val(obj.init.ActualTimeIndex),obj.init.c1_PE,obj.init.d1_PE,0,obj.init.buf_PE(traj).val);
             obj.init.PE_num_dot(traj).val(obj.init.ActualTimeIndex) = dPE;
             
             if obj.setup.PE_maxiter
                 if abs(dPE) > 1e0 || (obj.setup.AdaptiveSampling && obj.init.hyst_flag)
                     if dPE > 0
                        obj.setup.max_iter = max(10,floor(0.9*obj.setup.max_iter)); 
                     else
                        obj.setup.max_iter = min(200,ceil(1.1*obj.setup.max_iter));  
                     end
                 end
             end
             
            end
             
             obj.init.maxIterStory(obj.init.ActualTimeIndex) = obj.setup.max_iter;
        end
        
        % Broyden gradient method
        function [x_fin, obj] = Broyden(obj,x0,A0)          
            
            % init optimisation
            Ai = A0;
            xi = x0;
            d = 0;
            delta = Inf;
            c1 = 1e-3;
            alpha_min = 1e-5;
            % init alpha
            alpha = obj.init.alpha;
            % init J
            J = -8;
            
            % iterate Broyden's method
            while (d < obj.setup.max_iter) && (delta > obj.init.TolFun)

                %%%%% CREATE Pi %%%%%
                [Pi, Hrank] = Pi_Broyden(obj,xi);
                
                % Solve for the gradient step
                si = -pinv(Ai)*Pi;
                
                %%% armijo conditions for stepsize %%%
                % compute result
                xii = xi + alpha*si;
                Pii = Pi_Broyden(obj,xii);
                
                % check condition 1
                c1_cond = norm(Pii) > c1*norm(Pi);
                
                if (c1_cond) && (0.5*alpha > (alpha_min))
                    alpha = 0.5*alpha;
                else
                    alpha = 2*alpha;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % compute result
                xii = xi + alpha*si;
                Pii = Pi_Broyden(obj,xii);
                
                vi = Pii - Pi;
                A_update = (vi-Ai*si)*transpose(si)/(transpose(si)*si);
                check = isnan(A_update);
                if ~check
                    Ai = Ai + A_update;
                else
                    break;
                end
                
                delta = norm(xii-xi);
                d = d + 1;
                xi = xii;
                
                J = norm(Pi);
                if obj.setup.print
                    disp(['J: ',num2str(J)]);
                end
                
            end
            
            x_fin = xi;
            obj.init.J_last = J;
            obj.init.A_fin = Ai;
            obj.init.alpha = alpha;
            obj.init.Hrank(obj.init.ActualTimeIndex) = Hrank;
            
            
        end
        
        function [Pi, Hrank] = Pi_Broyden(obj,x0)
            
            % init H matrix and Y
            Y = zeros(obj.setup.w*obj.setup.dim_out*obj.setup.J_nterm,1);
            H = zeros(obj.setup.w*obj.setup.dim_out*obj.setup.J_nterm,1);
            
            % set the derivative buffer as before the optimisation process (multiple f computation)
            back_time = obj.init.BackIterIndex;
            % set the derivative buffer as before the optimisation process (multiple f computation)
            if back_time >= obj.init.d1_derivative
                temp_buf_dyhat = obj.init.Yhat_full_story(1,:,back_time-(obj.init.d1_derivative-1):back_time);
            else
                init_pos = obj.init.d1_derivative-back_time;
                stack = reshape(obj.init.Yhat_full_story(1,:,1:back_time),obj.setup.dim_out,back_time);
                temp_buf_dyhat = [zeros(obj.setup.dim_out,init_pos), stack];
            end

            n_item = length(find((obj.init.Y_space)));
            n_iter = n_item;

            for i=1:n_iter

                % evaluate the weighted in time cost function at this iteration time
                if(obj.setup.forward ~= 1) %backward        
                    %%%%%% TO BE DONE %%%%%%
                else
                    % get measure - hat
                    [temp_buf_dyhat, Yhat] = obj.measure_function(x0,i,temp_buf_dyhat,obj.init.Yhat_full_story);
                    for j=1:obj.setup.dim_out  
                        shift = obj.setup.J_nterm*obj.setup.dim_out;
                        H(1+(i-1)*shift:i*shift) = repmat(obj.setup.J_temp_scale(1:obj.setup.J_nterm),1,obj.setup.dim_out).*reshape(Yhat(:,1:obj.setup.J_nterm),1,obj.setup.J_nterm*obj.setup.dim_out);
                    end
                    
                    % get measure - true 
                    shift = obj.setup.J_nterm*obj.setup.dim_out;
                    spaces = nonzeros(obj.init.Y_space);
                    Ytrue = transpose(obj.init.Y_full_story(:,:,spaces(i)));
                    Y(1+(i-1)*shift:i*shift) = repmat(obj.setup.J_temp_scale(1:obj.setup.J_nterm),1,obj.setup.dim_out).*reshape(Ytrue,1,obj.setup.J_nterm*obj.setup.dim_out);
                end
            end
            
            %%%%%%%%%%%%%%%%%%%%%

            % create Pi
            Pi = Y-H;
            
            % get rank H
            try
                Hrank = rank(H);
            catch
                Hrank = -1;
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
                xhat_tmp(traj).val = xhat(traj).val(:,obj.init.ActualTimeIndex);
            end
            xhat = xhat_tmp;
            
            obj.init.just_optimised = 0;
            
            for traj=1:obj.setup.Ntraj
                % save runtime state
                obj.init.X_est_runtime(traj).val(:,obj.init.ActualTimeIndex) = obj.init.X_est(traj).val(:,obj.init.ActualTimeIndex);
                % get measure
                yhat(traj).val = obj.setup.measure(xhat(traj).val,obj.init.params);
            end
            
            for traj=1:obj.setup.Ntraj
                % get filters - y
                [obj.init.buf_dY(traj).val, dy, y_int] = obj.measure_filter(y(traj).val,obj.init.buf_dY(traj).val,obj.init.Y_full_story(traj).val,obj.init.ActualTimeIndex);
                y(traj).val = [y(traj).val, dy, y_int];
                
                % get filters - yhat
                [obj.init.buf_dYhat(traj).val, dyhat, yhat_int] = obj.measure_filter(yhat(traj).val,obj.init.buf_dYhat(traj).val,obj.init.Yhat_full_story(traj).val,obj.init.ActualTimeIndex);
                yhat(traj).val = [yhat(traj).val, dyhat, yhat_int];
            end
            
            %%%%%%%%%%%%%%%%%%%%% INSERT OPT %%%%%%%%%%%%%%%%%%%%%
            for term=1:obj.setup.J_nterm
                for traj=1:obj.setup.Ntraj
                    obj.init.Y_full_story(traj).val(term,:,obj.init.ActualTimeIndex) = y(traj).val(:,term);
                    obj.init.Yhat_full_story(traj).val(term,:,obj.init.ActualTimeIndex) = yhat(traj).val(:,term);
                end
            end

            % fisrt bunch of data - read Y every Nts and check if the signal is
            distance = obj.init.ActualTimeIndex-obj.init.Y_space(end);
            
            obj = obj.dJ_cond_v5_function();
            obj.init.distance_safe_flag = (distance < obj.init.safety_interval);
            %%%% select optimisation with hystheresis %%%%%
            hyst_low = (obj.init.dJ_cond_story(max(1,obj.init.ActualTimeIndex-1)) < obj.setup.dJ_low) && (obj.init.dJ_cond >= obj.setup.dJ_low);
            hyst_high = (obj.init.dJ_cond >= obj.setup.dJ_high);
            obj.init.hyst_flag = ~(hyst_low || hyst_high);
            %%%% persistent excitation %%%%
            obj = obj.PE();
            %%%% observer %%%%
            if  (~(((distance < obj.setup.Nts) || obj.init.hyst_flag) && (obj.init.distance_safe_flag))) && (obj.setup.optimise)

                if obj.setup.print
                    % Display iteration slengthtep
                    disp(['n window: ', num2str(obj.setup.w),'  n samples: ', num2str(obj.setup.Nts)])
                    disp(['Last cost function: ', num2str(obj.init.Jstory(end))]);
                    disp(['N. optimisations RUN: ',num2str(obj.init.opt_counter)]);
                    disp(['N. optimisations SELECTED: ',num2str(obj.init.select_counter)]);
                end

                %%%% OUTPUT measurements - buffer of w elements
                % measures
                for term=1:obj.setup.J_nterm
                    for traj=1:obj.setup.Ntraj
                        obj.init.Y(traj).val(term,:,1:end-1) = obj.init.Y(traj).val(term,:,2:end);
                        obj.init.Y(traj).val(term,:,end) = y(traj).val(:,term);
                    end
                end

                % backup
                Y_space_backup = obj.init.Y_space;
                Y_space_full_story_backup = obj.init.Y_space_full_story;

                % adaptive sampling
                obj.init.Y_space(1:end-1) = obj.init.Y_space(2:end);
                obj.init.Y_space(end) = obj.init.ActualTimeIndex;
                obj.init.Y_space_full_story(end+1) = obj.init.ActualTimeIndex;

                % store measure times
                obj.init.temp_time = [obj.init.temp_time obj.init.ActualTimeIndex];

                % check only on the first traj as the sampling is coherent
                % on the 2.
                cols_nonzeros = length(find(obj.init.Y_space ~= 0));

                if cols_nonzeros >= obj.setup.w
%                 if cols_nonzeros*obj.setup.J_nterm >= 2*obj.setup.dim_state+1

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
                            obj.init.Y_space = buf_Y_space_full_story(2:end);
                            
                            % down samplingbuffer
                            for traj=1:obj.setup.Ntraj
                                obj.init.Y(traj).val =  obj.init.Y_full_story(traj).val(:,:,obj.init.Y_space);
                            end
                        end
                        
                        % back time index
                        buf_dist = diff(buf_Y_space_full_story);
                        obj.init.BackTimeIndex = obj.setup.time(max(obj.init.ActualTimeIndex-sum(buf_dist),1)); 
                        obj.init.BackIterIndex = find(obj.setup.time==obj.init.BackTimeIndex);

                        % set of initial conditions
                        for traj=1:obj.setup.Ntraj
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

                            % reconstruct temp_x0 from opt/nonopt vars
                            obj.init.temp_x0(traj).val = zeros(obj.setup.dim_state,1);
                            obj.init.temp_x0(traj).val(obj.setup.opt_vars) = obj.init.temp_x0_opt;
                            obj.init.temp_x0(traj).val(obj.setup.nonopt_vars) = obj.init.temp_x0_nonopt(traj).val;
                        end
                        
                        % set target
                        obj = obj.target();
                        for traj = 1:obj.setup.Ntraj
                            obj.init.target_story(traj).val(:,:,obj.init.Y_space) = obj.init.target(traj).val;
                        end
                        
                        % Optimisation (only if distance_safe_flag == 1)
                        opt_time = tic;
                        
                        %%%%% OPTIMISATION - NORMAL MODE %%%%%%
                        if strcmp(obj.setup.fmin,'broyden')
                            %%%%% OPTIMISATION - BROYDEN %%%%%%
                            % save J before the optimisation to confront it later
                            J_before = norm(Pi_Broyden(obj,obj.init.temp_x0));
                            [NewXopt, obj] = Broyden(obj,obj.init.temp_x0,obj.init.A_fin);
                            J = obj.init.J_last;
                        elseif strcmp(func2str(obj.setup.fmin),'fmincon')
                            % save J before the optimisation to confront it later
                            [J_before, obj_tmp] = obj.cost_function(obj.init.temp_x0_opt,obj.init.temp_x0_nonopt,obj.init.target);
                            [NewXopt, J, obj.init.exitflag] = obj.setup.fmin(@(x)obj.cost_function(x,obj.init.temp_x0_nonopt,obj.init.target,1),obj.init.temp_x0_opt, obj.setup.Acon, obj.setup.Bcon,...
                                                              obj.setup.Acon_eq, obj.setup.Bcon_eq, obj.setup.LBcon, obj.setup.UBcon, obj.setup.NONCOLcon, obj.init.myoptioptions);
                        else
                            % save J before the optimisation to confront it later
                            [J_before, obj_tmp] = obj.cost_function(obj.init.temp_x0_opt,obj.init.temp_x0_nonopt,obj.init.target);
                            [NewXopt, J, obj.init.exitflag] = obj.setup.fmin(@(x)obj.cost_function(x,obj.init.temp_x0_nonopt,obj.init.target,1),obj.init.temp_x0_opt, obj.init.myoptioptions);
                        end
                        
                        % reconstruct NewXopt from opt/nonopt vars
                        NewXopt_tmp = [];
                        for traj = 1:obj.setup.Ntraj
                            NewXopt_end = zeros(obj.setup.dim_state,1);
                            NewXopt_end(obj.setup.opt_vars) = NewXopt;
                            NewXopt_end(obj.setup.nonopt_vars) = obj.init.temp_x0_nonopt(traj).val;                          
                            NewXopt_tmp(traj).val = NewXopt_end;
                        end
                        NewXopt = NewXopt_tmp;

                        % opt counter
                        obj.init.opt_counter = obj.init.opt_counter + 1;

                        % adaptive buffer backup restore
                        obj.init.Y_space = Y_space_backup;
                        obj.init.Y_space_full_story = Y_space_full_story_backup;

                        % check J_dot condition
                        J_diff = (J/J_before);

                        if (obj.setup.AlwaysOpt) || ( (J_diff <= obj.setup.Jdot_thresh) || (distance > obj.init.safety_interval) )
                            
                            % update params
                            obj.init.params = obj.setup.params.params_update(obj.init.params,NewXopt(1).val);

                            % on each trajectory
                            for traj=1:obj.setup.Ntraj
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
                                % set the derivative buffer as before the optimisation process (multiple f computation)
                                if back_time > obj.init.d1_derivative
                                    temp_buf_dyhat = obj.init.Yhat_full_story(traj).val(1,:,obj.init.BackIterIndex-(obj.init.d1_derivative):back_time-1);
                                else
                                    temp_buf_dyhat = zeros(obj.setup.dim_out,obj.init.d1_derivative);
                                    range = 1:back_time-1;
                                    temp_buf_dyhat(:,end-length(range)+1:end) = obj.init.Yhat_full_story(traj).val(1,:,range);
                                end

                                %%%% ESTIMATED measurements
                                % measures       
                                % NB: the output storage has to be done in
                                % back_time+1 as the propagation has been
                                % performed 
                                Yhat = obj.setup.measure(x_propagate,obj.init.params);
                                % get filters - yhat
                                [temp_buf_dyhat, dyhat, yhat_int] = obj.measure_filter(Yhat,temp_buf_dyhat,obj.init.Yhat_full_story(traj).val,back_time);
                                yhat = [Yhat, dyhat, yhat_int];
                                for term=1:obj.setup.J_nterm
                                    obj.init.Yhat_full_story(traj).val(term,:,back_time) = yhat(:,term);
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

                                %%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%
                                n_iter_propagate = obj.init.ActualTimeIndex-obj.init.BackIterIndex;

                                for j =1:n_iter_propagate
                                    % back time
                                    back_time = obj.init.BackIterIndex+j;
                                    
                                    % how do you handle the input?
                                    if obj.setup.control_design
                                        obj.init.params.u = obj.init.params.input(x_propagate,obj.init.params);
                                    else
                                        % recall that if control_design == 0
                                        % there is at most 1 Ntraj and
                                        % therefore obj.init.X has only 1 field
                                        obj.init.params.u = obj.init.params.input(obj.init.X(1).val(:,back_time),obj.init.params);
                                    end

                                    % integrate
                                    X = obj.setup.ode(@(t,x)obj.setup.model(back_time, x, obj.init.params), obj.setup.tspan, x_propagate);
                                    x_propagate = X.y(:,end);                      
                                    obj.init.X_est(traj).val(:,back_time) = x_propagate;

                                    %%%% ESTIMATED measurements
                                    % measures       
                                    % NB: the output storage has to be done in
                                    % back_time+1 as the propagation has been
                                    % performed 
                                    Yhat = obj.setup.measure(x_propagate,obj.init.params);
                                    % get filters - yhat
                                    [temp_buf_dyhat, dyhat, yhat_int] = obj.measure_filter(Yhat,temp_buf_dyhat,obj.init.Yhat_full_story(traj).val,back_time);
                                    yhat = [Yhat, dyhat, yhat_int];
                                    for term=1:obj.setup.J_nterm
                                        obj.init.Yhat_full_story(traj).val(term,:,back_time) = yhat(:,term);
                                    end
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                                obj.init.Jstory(1,end+1) = J;
                                if obj.setup.Jterm_store
                                    obj.init.Jterm_story(:,end+1) = obj_tmp.init.Jterm_store;
                                end
                                obj.init.buf_dYhat(traj).val = obj.init.Yhat_full_story(traj).val(1,:,obj.init.ActualTimeIndex-obj.init.d1_derivative+1:obj.init.ActualTimeIndex);
                                obj.init.buf_dYhat(traj).val = reshape(obj.init.buf_dYhat(traj).val,obj.setup.dim_out,obj.init.d1_derivative);
                            end
                        else
                            % on each trajectory
                            for traj=1:obj.setup.Ntraj
                                % keep the initial guess
                                obj.init.X_est(traj).val(:,obj.init.BackIterIndex) = obj.init.temp_x0(traj).val;
                            end
                            
                            % restore params
                            obj.init.params = obj.setup.params.params_update(obj.init.params,obj.init.temp_x0(traj).val);
                        end                        

                        % adaptive sampling
                        obj.init.Y_space(1:end-1) = obj.init.Y_space(2:end);
                        obj.init.Y_space(end) = obj.init.ActualTimeIndex;
                        obj.init.Y_space_full_story(end+1) = obj.init.ActualTimeIndex;

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
                % save measures
                obj.init.Y_est(traj).val = obj.init.X_est(traj).val(obj.setup.observed_state,:);
                obj.init.Y_est_runtime(traj).val = obj.init.X_est_runtime(traj).val(obj.setup.observed_state,:);
            end
            
%             try
%                 X0 = obj.setup.ode(@(t,x)obj.setup.model(t, x, obj.init.params), [obj.setup.time(end):-obj.setup.Ts:obj.setup.time(1)], obj.init.X_est(:,end));
%                 obj.init.X0_est = X0.y(:,end);
%             catch
%                 obj.init.X0_est = -ones(obj.setup.dim_state,1);
%             end
%             obj.init.X0_true = obj.setup.params.X;
%             obj.init.X0 = obj.setup.params.X_est;

        end
        
        % plot results: this method simply plots the results of the
        % observtion process. 
        function plot_section(obj,varargin)
            
            %%%% plot state estimation %%%
            figure()
            for i=1:obj.setup.dim_state
                subplot(obj.setup.dim_state,1,i);
                hold on
                grid on
                box on
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(1).val(i,:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(1).val(i,:),'r.');
                plot(obj.setup.time,obj.init.X_est_runtime(1).val(i,:),'g');
                
                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Stored','Runtime')
                else
                    legend('Stored','Runtime')
                end
            end
            
            %%%% plot state estimation error %%%
            figure()
            for i=1:obj.setup.dim_state
                subplot(obj.setup.dim_state,1,i);
                hold on
                grid on
                box on
                
                est_error = obj.init.X(1).val(i,:) - obj.init.X_est_runtime(1).val(i,:);
                plot(obj.setup.time,est_error,'k','LineWidth',2);
                
                xlabel('time [s]')
                ylabel(['\delta x_',num2str(i)])
            end
            
            
            %%%% plot windowed data %%%%
            figure()
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
                OptTime = obj.setup.time(obj.init.opt_chosen_time);
                
                % plot down sampling
                data = reshape(obj.init.Y_full_story(1).val(1,k,obj.init.temp_time),1,length(WindowTime));
                plot(WindowTime,data,'s','MarkerSize',5);
                
                % plot accepted optimisations
                data = reshape(obj.init.Y_full_story(1).val(1,k,obj.init.opt_chosen_time),1,length(OptTime));
                plot(OptTime,data,'r+','MarkerSize',5);
                
                % plot measures
                plot(obj.setup.time,obj.init.Y_est(1).val(k,:),'k.')
                
                % plot true values
                y_meas = reshape(obj.init.Y_full_story(1).val(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,y_meas,'m--')
                
                % plot targe values
                data = reshape(obj.init.target_story(1).val(1,k,obj.init.temp_time),1,length(WindowTime));
                plot(WindowTime,data,'bo','MarkerSize',2);
                
                ylabel(strcat('y_',num2str(k)));
                xlabel('simulation time [s]');
                legend('measures','opt shots','estimation','true','target')
            end
            linkaxes(ax,'x');
            
            %%%% plot filters %%%%%
            figure()
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
                for dim=1:obj.setup.dim_out
                    y_plot = reshape(obj.init.Y_full_story(1).val(k,dim,:),size(obj.setup.time));
                    yhat_plot = reshape(obj.init.Yhat_full_story(1).val(k,dim,:),size(obj.setup.time));
                    plot(obj.setup.time,y_plot,'b--');
                    plot(obj.setup.time,yhat_plot,'r.');
                end
                
                ylabel(strcat('y_filter',num2str(k)));
                xlabel('simulation time [s]');
            end
            linkaxes(ax,'x');
            
            %%%% plot cost function %%%%
            figure()
            title('Cost function')
            hold on
            box on
            grid on
            obj.init.Jstory(1) = [];
            OptTime = obj.setup.time(obj.init.opt_chosen_time);
            plot(OptTime,obj.init.Jstory);
            ylabel('cost function J');
            xlabel('simulation time [s]');
            
            %%% plot cost function terms %%%
            if obj.setup.Jterm_store
                figure()
                title('Cost function terms')
                hold on
                box on
                grid on
                obj.init.Jterm_story(:,1) = [];
                OptTime = obj.setup.time(obj.init.opt_chosen_time);
                for i=1:obj.setup.J_nterm_total
                    plot(OptTime,obj.init.Jterm_story(i,:));
                end
                legend
                ylabel('cost function J');
                xlabel('simulation time [s]'); 
            end
            
            %%%% plot adaptive sampling %%%%
            if obj.setup.AdaptiveSampling
                figure()
                title('Adaptive sampling hysteresis')
                hold on
                box on
                grid on
                % down sampling instants
                WindowTime = obj.setup.time(obj.init.temp_time);
                OptTime = obj.setup.time(obj.init.opt_chosen_time);
                % plot the whole dJcond
                plot(obj.setup.time,obj.init.dJ_cond_story,'--');
                % plot the optimised and selected instants
                plot(WindowTime,obj.init.dJ_cond_story(obj.init.temp_time),'s','MarkerSize',5);
                plot(OptTime,obj.init.dJ_cond_story(obj.init.opt_chosen_time),'r+','MarkerSize',5);
                % plot the hysteresis
                plot(obj.setup.time,ones(size(obj.setup.time))*obj.setup.dJ_low,'r.');
                plot(obj.setup.time,ones(size(obj.setup.time))*obj.setup.dJ_high,'k.');
                ylabel('Accuracy level');
                xlabel('simulation time [s]');
            end
            
        end
        
        % plot results for control design
        function plot_section_control(obj,varargin)
            
            %%%% plot state estimation %%%
            figure()
            for i=1:obj.setup.dim_state
                subplot(obj.setup.dim_state,1,i);
                hold on
                grid on
                box on
                
                for traj=1:obj.setup.Ntraj
                    if strcat(obj.setup.DataType,'simulated')
                        plot(obj.setup.time,obj.init.X(traj).val(i,:),'b--');
                    end
                    plot(obj.setup.time,obj.init.X_est(traj).val(i,:),'r.');
%                     plot(obj.setup.time,obj.init.X_est_runtime(traj).val(i,:),'g');

                    if strcat(obj.setup.DataType,'simulated')
                        legend('True','Est')
%                         legend('Stored','Est','Runtime')
                    else
                        legend('Stored','Est','Runtime')
                    end
                end
            end
            
            
            %%%% plot windowed data %%%%
            figure()
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
                    % plot down sampling
                    data = reshape(obj.init.Yhat_full_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'s','MarkerSize',5);

                    % plot targe values
                    data = reshape(obj.init.target_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'bo','MarkerSize',2);

                    ylabel(strcat('y_',num2str(k)));
                    xlabel('simulation time [s]');
                    legend('estimation','target')
                end
            end
            linkaxes(ax,'x');
            
            %%%% plot filters %%%%%
            figure()
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
                        y_plot = reshape(obj.init.Y_full_story(traj).val(k,dim,:),size(obj.setup.time));
                        yhat_plot = reshape(obj.init.Yhat_full_story(traj).val(k,dim,:),size(obj.setup.time));
                        plot(obj.setup.time,y_plot,'b--');
                        plot(obj.setup.time,yhat_plot,'r.');
                    end
                    
                    ylabel(strcat('y_filter',num2str(k)));
                    xlabel('simulation time [s]');
                end            
                
            end
            linkaxes(ax,'x');
            
        end
    end
end
