%% class definition - obsopt observer
classdef obsopt_cubli
    %%% class properties
    properties
        % setup: set of flags defining the main characteristics
        setup;
        
        % init: set of fags initialised depending on setup
        init;
    end
    
    %%% class methods %%%
    % dynamic methods
    methods
        % constructor
        function obj = obsopt_cubli(Ts,Nw,Nts)
            % simulation data init
            obj.setup.Ts = Ts;
            obj.setup.tspan = [0, Ts];

            % plot and graphics
            obj.setup.plot = 0;
            obj.setup.print = 1;

            % sampling 
            obj.setup.w = Nw;
            obj.setup.Nts = Nts;

            % conditions to consider the optimisation result
            obj.setup.Jdot_thresh = 9e-1;

            % built in/gradient optimisation conditions
            obj.setup.J_thresh = [1e-10, 1e3];
            obj.setup.max_iter = 100;

            % optimisation
            obj.setup.fmin = @fminsearch;

            % integration
            obj.setup.forward = 1;
            obj.setup.ode = @oderk4;
            
            % model plant
            obj.setup.model = @cubli_model;
            obj.setup.measure = @measure_cubli;
            
            % cost function setup
            obj.setup.J_temp_scale = [1,0,0];
            obj.setup.J_nterm = size(nonzeros(obj.setup.J_temp_scale),1);
        end
        
        % init function 
        function obj = obs_init(obj,dim_out,dim_state)
            % output dimension
            obj.init.dim_out = dim_out;
            
            % state dimension
            obj.init.dim_state = dim_state;
            
            % create scale factor
            obj = obj.scale_factor();
            
            % Window Samples
            obj.init.WindowSamples = max(2,obj.setup.Nts*(obj.setup.w-1)+1);
            
            % BackIterIndex
            obj.init.BackIterIndex = 1;

            % measure derivative buffer
            obj.init.Y =  zeros(obj.setup.J_nterm,obj.init.dim_out,obj.setup.w);
            obj.init.Ytrue_full_story = [];
            obj.init.Y_full_story = [];
            obj.init.Yhat_full_story = [];

            % buffer adaptive sampling
            obj.init.Y_space = zeros(1,obj.setup.w);
            obj.init.Y_space_full_story = 0;

            % dJ condition buffer (adaptive sampling)
            obj.init.dJ_cond_story = [];
            
            % observer cost function init
            obj.init.J = 1e3;
            obj.init.Jstory = obj.init.J;
            obj.init.Jdot_story = 0;
            obj.init.J_components = ones(obj.init.dim_out,obj.setup.J_nterm);
            obj.init.temp_time = [];
            obj.init.opt_chosen_time = [];
            obj.init.grad_story = zeros(obj.init.dim_state,1);
            
            % opt counters
            obj.init.opt_counter = 0;
            obj.init.select_counter = 0;
            
            % optimset
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

        end
        
        % scale factor (cost function weights)
        function obj = scale_factor(obj)
            obj.init.scale_factor = obj.setup.J_temp_scale.*ones(obj.init.dim_out,obj.setup.J_nterm);
        end
        
        % numeric derivative
        function [buffer_out, dy] = derivative(T,y,c,d,mediana,buffer)

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
        
        % outfun (exit from fmin)
        function stop = outfun(obj,x, optimValues, state)
            if (abs(optimValues.fval) < 1e-8)
                stop = 1;
            else
                stop = 0;
            end
        end
        
        % cost function (obj,x,varargin)
        function [Jtot,obj] = cost_function(obj,varargin)

            % get state
            x = varargin{1};
            
            % get params if exists
            if any(strcmp(varargin,'params'))
                pos = find(strcmp(varargin,'params'));
                params = varargin{pos+1};
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
                    Yhat = obj.measure_function(X,j,'params',params);

                    J = zeros(obj.setup.J_nterm,size(Yhat,1));

                    for term=1:obj.setup.J_nterm
                        % get the J
                        for i=1:obj.init.dim_out
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

            %%% final stuff %%%
            if n_iter > 0
                obj.init.Yhat_temp = Yhat;
            else
                Jtot = 1; 
            end
        end
        
        % get measure (x,W_buf_pos,...)
        function y_read = measure_function(obj,varargin)
            
            % get the state
            x_propagate = varargin{1};
            W_buf_pos = varargin{2};
            
            % get params if exists
            if any(strcmp(varargin,'params'))
                pos = find(strcmp(varargin,'params'));
                params = varargin{pos+1};
            end
            
            
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
                    X = obj.setup.ode(@(t,x)obj.setup.model(t, x, params), obj.setup.tspan, x_propagate);
                    x_propagate = X.y(:,end);

                end
            else
                %%%%% TO BE DONE %%%%
            end
            
            % get the measure
            y_read = obj.setup.measure(x_propagate,params);
            
        end
    end
end