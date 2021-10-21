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
classdef obsopt_general_adaptive
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
        function obj = obsopt_general_adaptive(varargin)
            
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
                params.input = zeros(1,length(obj.setup.time));
                
                % estimated param
                obj.setup.estimated_params = [];
            end
            
            % get number of iterations
            obj.setup.Niter = length(obj.setup.time);
            
            % define the time span from the sample time. This will be used
            % in the integration procedure (e.g. ode45)
            obj.setup.tspan = [0, obj.setup.Ts];
            
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

            % get the optimisation method. Default is fminsearch from
            % MATLAB
            if any(strcmp(varargin,'opt'))
                pos = find(strcmp(varargin,'opt'));
                obj.setup.fmin = varargin{pos+1};
            else
                obj.setup.fmin = @fminsearch;
            end

            % get the algorithm direction (forward or backward). V1.1 has
            % only forward implemented
            if any(strcmp(varargin,'forward'))
                pos = find(strcmp(varargin,'forward'));
                obj.setup.forward = varargin{pos+1};
            else
                obj.setup.forward = 1;
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
            obj.setup.J_nterm = min(3,size(nonzeros(obj.setup.J_temp_scale),1));
            % This variable instead taske into account the whole set of
            % terms. It's used to create the scale factor (see obs.scale_factor())
            obj.setup.J_nterm_total = size(nonzeros(obj.setup.J_temp_scale),1);
            
            % get spring like term in cost function
            obj.setup.J_term_spring = (temp_scale(4) ~= 0);
            obj.setup.J_term_spring_position = 4;
            
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
            end
            % complete the params update in .setup
            obj.setup.params = params;
            
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
            obj.init.buf_dY = zeros(obj.setup.dim_out,obj.init.d1_derivative);
            obj.init.buf_dYhat = zeros(obj.setup.dim_out,obj.init.d1_derivative);

            % measure buffer: these buffers are used to store the measured
            % and estimated values on the observed states. 
            obj.init.Y =  zeros(obj.setup.J_nterm,obj.setup.dim_out,obj.setup.w);
            obj.init.Ytrue_full_story = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
            obj.init.Y_full_story = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);
            obj.init.Yhat_full_story = zeros(obj.setup.J_nterm,obj.setup.dim_out,0);

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
            obj.init.Jdot_story = 0;
            % J_components is used to keep track of the different cost
            % function terms amplitude. Not implemented in V1.1
            obj.init.J_components = ones(obj.setup.dim_out,obj.setup.J_nterm);
            
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
                                            'TolFun',obj.init.TolFun,'TolX',obj.init.TolX,'MaxFunEvals',Inf,'OutputFcn',@obj.outfun);
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
        function [Jtot,obj] = cost_function(obj,varargin)

            % get state
            x = varargin{1};
            
            % set the derivative buffer as before the optimisation process (multiple f computation)
            back_time = obj.init.BackIterIndex;
            % set the derivative buffer as before the optimisation process (multiple f computation)
            if back_time >= obj.init.d1_derivative
                temp_buf_dyhat = obj.init.Yhat_full_story(1,:,back_time-(obj.init.d1_derivative-1):back_time);
            else
                init_pos = obj.init.d1_derivative-back_time;
                temp_buf_dyhat = [zeros(obj.setup.dim_out,init_pos), obj.init.Yhat_full_story(1,:,1:back_time)];
            end
            
            % cost function init
            Jtot = 0;

            %optimization vector  
            X = x; 

            n_item = length(find((obj.init.Y_space)));
            n_iter = n_item;

            shift = obj.setup.w-n_item;

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
                            J(term,i) = obj.init.scale_factor(i,term)*(diff)^2;
                        end
                    end

                    for term=1:obj.setup.J_nterm
                        tmp = sum(J(term,:));
                        Jtot = Jtot + tmp; 
                    end

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
            
            Jtot = Jtot+Jspring;

            %%% final stuff %%%
            if n_iter > 0
                obj.init.Yhat_temp = Yhat;
            else
                Jtot = 1; 
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
            
            % get params
            params = obj.init.params;
                        
            %%%% sampling %%%%
            if obj.init.BackIterIndex > 1
                zero_flag = 0;
            else
                zero_flag = 1;
            end
            n_iter = W_buf_pos*obj.setup.Nts-zero_flag;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if obj.setup.forward
                % the max has been added to at least save the current state if
                for i=1:n_iter
                    
                    % integration
                    params.u = params.input(obj.init.BackIterIndex+i-1);
                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, params), obj.setup.tspan, x_propagate);
                    x_propagate = X.y(:,end);
                    
                    % get the measure
                    y_read = obj.setup.measure(x_propagate,params);
                    
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

            buffer_ready = (nnz(obj.init.Y_space) >= 1) || (size(obj.init.Yhat_full_story,3) > obj.setup.Nts);

            if buffer_ready

                [~, pos_hat] = find(obj.init.Y_space ~= 0);
                pos_hat = obj.init.Y_space(pos_hat)-1;
                Yhat = obj.init.Yhat_full_story(1,:,pos_hat);
                Y_buf = obj.init.Y_full_story(1,:,pos_hat);

                n_sum = size(Y_buf,2);
                temp_e = 0;
                for i=1:n_sum
                    temp_e = temp_e + norm(Y_buf(:,i)-(Yhat(:,i)));
                end

                obj.init.dJ_cond = norm(temp_e);
            else
                obj.init.dJ_cond = 1.1*obj.setup.dJ_high;
            end
            
            obj.init.dJ_cond_story(:,obj.init.ActualTimeIndex) = obj.init.dJ_cond;
        end
        
        % observer function: this method wraps up all the afromentioned
        % methods and actually implements the observer. Check the reference
        % for more information. 
        function obj = observer(obj,xhat,y)
            
            params = obj.init.params;
            
            % save runtime state
            obj.init.X_est_runtime(:,obj.init.ActualTimeIndex) = obj.init.X_est(:,obj.init.ActualTimeIndex);

            yhat = obj.setup.measure(xhat,params);
            
            % get filters - y
            [obj.init.buf_dY, dy, y_int] = obj.measure_filter(y,obj.init.buf_dY,obj.init.Y_full_story,obj.init.ActualTimeIndex);
            y = [y, dy, y_int];
            
            % get filters - yhat
            [obj.init.buf_dYhat, dyhat, yhat_int] = obj.measure_filter(yhat,obj.init.buf_dYhat,obj.init.Yhat_full_story,obj.init.ActualTimeIndex);
            yhat = [yhat, dyhat, yhat_int];

            %%%%%%%%%%%%%%%%%%%%% INSERT OPT %%%%%%%%%%%%%%%%%%%%%
            for term=1:obj.setup.J_nterm
                obj.init.Y_full_story(term,:,obj.init.ActualTimeIndex) = y(:,term);
                obj.init.Yhat_full_story(term,:,obj.init.ActualTimeIndex) = yhat(:,term);
            end

            % fisrt bunch of data - read Y every Nts and check if the signal is
            distance = obj.init.ActualTimeIndex-obj.init.Y_space(end);
            
            obj = obj.dJ_cond_v5_function();
            obj.init.distance_safe_flag = (distance < obj.init.safety_interval);
            %%%% select optimisation with hystheresis %%%%%
            hyst_low = (obj.init.dJ_cond_story(max(1,obj.init.ActualTimeIndex-1)) < obj.setup.dJ_low) && (obj.init.dJ_cond >= obj.setup.dJ_low);
            hyst_high = (obj.init.dJ_cond >= obj.setup.dJ_high);
            obj.init.hyst_flag = ~(hyst_low || hyst_high);
            if  ~(((distance < obj.setup.Nts) || obj.init.hyst_flag) && (obj.init.distance_safe_flag))

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
                    obj.init.Y(term,:,1:end-1) = obj.init.Y(term,:,2:end);
                    obj.init.Y(term,:,end) = y(:,term);
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

                cols_nonzeros = length(find(sum(obj.init.Y ~= 0,1)));

                if cols_nonzeros >= obj.setup.w

                    if obj.setup.forward

                        %%%% FLUSH THE BUFFER IF SAFETY FLAG %%%%
                        first_nonzero = find(obj.init.Y_space,1,'first');
                        Y_space_nonzero = obj.init.Y_space(first_nonzero:end);
                        max_dist = max(diff(Y_space_nonzero));
                        if isempty(max_dist)
                            max_dist = 1;
                        end
                        n_samples = min(length(obj.init.Y_space_full_story)-1,obj.setup.w);
                        buf_Y_space_full_story = obj.init.Y_space_full_story(end-n_samples:end);

                        % back time index
                        buf_dist = diff(buf_Y_space_full_story);
                        obj.init.BackTimeIndex = obj.setup.time(max(obj.init.ActualTimeIndex-sum(buf_dist),1)); 
                        obj.init.BackIterIndex = find(obj.setup.time==obj.init.BackTimeIndex);

                        % set of initial conditions
                        obj.init.temp_x0 = obj.init.X_est(:,obj.init.BackIterIndex);

                        % Optimisation

                        % Optimisation (only if distance_safe_flag == 1)
                        opt_time = tic;
                        % save J before the optimisation to confront it later
                        J_before = obj.cost_function(obj.init.temp_x0);


                        %%%%% OPTIMISATION - NORMAL MODE %%%%%%
                        [NewXopt, J, obj.init.exitflag] = obj.setup.fmin(@(x)obj.cost_function(x),obj.init.temp_x0,obj.init.myoptioptions);

                        % opt counter
                        obj.init.opt_counter = obj.init.opt_counter + 1;

                        % adaptive buffer backup restore
                        obj.init.Y_space = Y_space_backup;
                        obj.init.Y_space_full_story = Y_space_full_story_backup;

                        % check J_dot condition
                        J_diff = (J/J_before);

                        if (obj.setup.AlwaysOpt) || ( (J_diff <= obj.setup.Jdot_thresh) || (distance > obj.init.safety_interval) )

                            % update
                            obj.init.X_est(:,obj.init.BackIterIndex) = NewXopt;

                            % store measure times
                            obj.init.opt_chosen_time = [obj.init.opt_chosen_time obj.init.ActualTimeIndex];

                            % counters
                            obj.init.jump_flag = 0;
                            obj.init.select_counter = obj.init.select_counter + 1;

                            x_propagate = NewXopt;

                            %%%%%%%%%%%%%%%%% FIRST MEASURE UPDATE %%%%%%%%
                            % manage measurements
                            back_time = obj.init.BackIterIndex;
                            % set the derivative buffer as before the optimisation process (multiple f computation)
                            if back_time > obj.init.d1_derivative
                                temp_buf_dyhat = obj.init.Yhat_full_story(1,:,obj.init.BackIterIndex-(obj.init.d1_derivative):back_time-1);
                            else
                                temp_buf_dyhat = zeros(obj.setup.dim_out,obj.init.d1_derivative);
                                range = 1:back_time-1;
                                temp_buf_dyhat(:,end-length(range)+1:end) = obj.init.Yhat_full_story(1,:,range);
                            end

                            %%%% ESTIMATED measurements
                            % measures       
                            % NB: the output storage has to be done in
                            % back_time+1 as the propagation has been
                            % performed 
                            Yhat = obj.setup.measure(x_propagate,params);
                            % get filters - yhat
                            [temp_buf_dyhat, dyhat, yhat_int] = obj.measure_filter(Yhat,temp_buf_dyhat,obj.init.Yhat_full_story,back_time);
                            yhat = [Yhat, dyhat, yhat_int];
                            for term=1:obj.setup.J_nterm
                                obj.init.Yhat_full_story(term,:,back_time) = yhat(:,term);
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

                            %%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%
                            n_iter_propagate = obj.init.ActualTimeIndex-obj.init.BackIterIndex;

                            for j =1:n_iter_propagate
                                % back time
                                back_time = obj.init.BackIterIndex+j;

                                % integrate
                                params.u = params.input(back_time-1);
                                X = obj.setup.ode(@(t,x)obj.setup.model(back_time, x, params), obj.setup.tspan, x_propagate);
                                x_propagate = X.y(:,end);                      
                                obj.init.X_est(:,back_time) = x_propagate;

                                %%%% ESTIMATED measurements
                                % measures       
                                % NB: the output storage has to be done in
                                % back_time+1 as the propagation has been
                                % performed 
                                Yhat = obj.setup.measure(x_propagate,params);
                                % get filters - yhat
                                [temp_buf_dyhat, dyhat, yhat_int] = obj.measure_filter(Yhat,temp_buf_dyhat,obj.init.Yhat_full_story,back_time);
                                yhat = [Yhat, dyhat, yhat_int];
                                for term=1:obj.setup.J_nterm
                                    obj.init.Yhat_full_story(term,:,back_time) = yhat(:,term);
                                end
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                            obj.init.Jstory(1,end+1) = J;
                            obj.init.buf_dYhat = obj.init.Yhat_full_story(1,obj.init.ActualTimeIndex-obj.init.d1_derivative+1:obj.init.ActualTimeIndex);
                        else
                            % keep the initial guess
                            obj.init.X_est(:,obj.init.BackIterIndex) = obj.init.temp_x0;
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

            % save measures
            obj.init.Y_est = obj.init.X_est(obj.setup.observed_state,:);
            obj.init.Y_est_runtime = obj.init.X_est_runtime(obj.setup.observed_state,:);

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
                plot(obj.setup.time,obj.init.X(i,:),'b--');
                plot(obj.setup.time,obj.init.X_est(i,:),'r.');
                plot(obj.setup.time,obj.init.X_est_runtime(i,:),'g');
                legend('True','Stored','Runtime')
            end
            
            
            %%%% plot windowed data %%%%
            figure()
            title('Sampled measured')
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
                plot(WindowTime,obj.init.Y_full_story(k,obj.init.temp_time),'s','MarkerSize',5);
                
                % plot accepted optimisations
                plot(OptTime,obj.init.Y_full_story(k,obj.init.opt_chosen_time),'r+','MarkerSize',5);
                
                % plot measures
                plot(obj.setup.time,obj.init.Y_est_runtime(k,:),'k.')
                
                % plot true values
                y_meas = reshape(obj.init.Y_full_story(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,y_meas,'m--')
                
                ylabel(strcat('y_',num2str(k)));
                xlabel('simulation time [s]');
                legend('measures','opt shots','estimation','true')
            end
            linkaxes(ax,'x');
            
            %%%% plot filters %%%%%
            figure()
            title('Filters on measures')
            for k=1:obj.setup.J_nterm
                
                % number fo subplots depending on the Nterm
                n_subplot = obj.setup.J_nterm;
                
                % indicize axes
                ax_index = k;
                ax(ax_index)=subplot(n_subplot,1,ax_index);
                
                % plot
                hold on
                for dim=1:obj.setup.dim_out
                    y_plot = reshape(obj.init.Y_full_story(k,dim,:),size(obj.setup.time));
                    yhat_plot = reshape(obj.init.Yhat_full_story(k,dim,:),size(obj.setup.time));
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
    end
end