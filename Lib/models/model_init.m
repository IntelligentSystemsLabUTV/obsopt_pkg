%% MODEL_INIT
% file: model_init.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function defines the model parameters. It's a general 
% method, depending only on the params_init.m script, defining the 
% model variables and costants. A default case is set for each entry, but
% please check before running the code. 
% INPUT: no input
% OUTPUT: 
% params: structure with all the necessary parameter to the model
function params = model_init(varargin)

    % get params_init.m script
    if any(strcmp(varargin,'params_init'))
        pos = find(strcmp(varargin,'params_init'));
        params_init = varargin{pos+1};
        params = params_init();
    else
        params.X = 5;
    end
    
    % get the model parameters to be estimated. Default is standard MHE
    % with no model identification capabilities.
    fields = fieldnames(params);
    if ~any(strcmp(fields,'estimated_params'))
        params.estimated_params = [];
    end
    
    % sampling time
    if any(strcmp(varargin,'Ts'))
        pos = find(strcmp(varargin,'Ts'));
        Ts = varargin{pos+1};
    else
        Ts = 1e-1;
    end
    
    % start and finish time
    if any(strcmp(varargin,'T0'))
        pos = find(strcmp(varargin,'T0'));
        time_tmp = varargin{pos+1};
        t0 = time_tmp(1);
        tend = time_tmp(2);
    else
        t0 = 0;
        tend = 1;
    end

    % setup the structure as far as time and iterations are concerned
    params.time = t0:Ts:tend;
    params.Ts = Ts;
    params.Niter = length(params.time);
    params.tspan = [0, Ts];
    
    % get noise if exists. Default is 0 (no noise)
    if any(strcmp(varargin,'noise'))
        pos = find(strcmp(varargin,'noise'));
        params.noise = varargin{pos+1};
    else
        params.noise = 0;
    end
    
    % get noise setup if exists. Default is mu=0, std=5e-2 (Gaussian)
    if any(strcmp(varargin,'noise_spec'))
        pos = find(strcmp(varargin,'noise_spec'));
        noise_spec = varargin{pos+1};
        params.noise_mu = noise_spec(:,1);
        params.noise_std = noise_spec(:,2);
    else
        params.noise_mu = 0;
        params.noise_std = 5e-2;
    end
    
    % get state dimension. Default is 1
    if any(strcmp(varargin,'StateDim'))
        pos = find(strcmp(varargin,'StateDim'));
        params.StateDim = varargin{pos+1};
    else
        params.StateDim = params.dim_state;
    end
    
    % get set of observed states. Default is 1
    if any(strcmp(varargin,'ObservedState'))
        pos = find(strcmp(varargin,'ObservedState'));
        params.observed_state = varargin{pos+1};
    else
        params.observed_state = 1;
    end
    
    % set the output dimensions from the observed state
    params.OutDim = length(params.observed_state);
    
    % get model if exists. Default is a 1 dimension asymptotically stable
    % system.
    if any(strcmp(varargin,'model'))
        pos = find(strcmp(varargin,'model'));
        params.model = varargin{pos+1};
    else
        params.model = @(t,x,params) -2*x;
    end

    % get measure if exists. Default measures the whole state
    if any(strcmp(varargin,'measure'))
        pos = find(strcmp(varargin,'measure'));
        params.measure = varargin{pos+1};
    else
        params.measure = @(x,params,t) x;
    end
    
    % get measure if exists. Default measures the whole state
    if any(strcmp(varargin,'params_update'))
        pos = find(strcmp(varargin,'params_update'));
        params.params_update = varargin{pos+1};
    else
        params.params_update = @(x,params) 1;
    end
    
    % get the integration algorithm. Default is ode45
    if any(strcmp(varargin,'ode'))
        pos = find(strcmp(varargin,'ode'));
        params.ode = varargin{pos+1};
    else
        params.ode = @ode45;
    end
    
    % get the integration settings
    if any(strcmp(varargin,'odeset'))
        pos = find(strcmp(varargin,'odeset'));
        params.odeset = odeset('RelTol',varargin{pos+1}(1),'AbsTol',varargin{pos+1}(2));
    else
        params.odeset = odeset('RelTol',1e-3,'AbsTol',1e-6);
    end
    
    % check and set the control presence (default is free evolution)
    if any(strcmp(varargin,'input_enable'))
        pos = find(strcmp(varargin,'input_enable'));
        params.input_enable = varargin{pos+1};
    else
        params.input_enable = 0;
    end        

    % input law definition. Default is free evolution 
    if any(strcmp(varargin,'input_law'))
        pos = find(strcmp(varargin,'input_law'));
        if ~isempty(varargin{pos+1})
            params.input = varargin{pos+1};
        else
            params.input = @(x,params) 0;
        end 
    end
               
    % remark: the params.Ntraj variable describes on how many different
    % trajectories the MHE is run. This makes sense in the control design
    % framework, which is under development. If standard MHE is to be used,
    % please ignore obs.setup.Ntraj as it is set to 1.
    for traj=1:params.Ntraj
        
        %%%% initial condition %%%%
        % it is taken from params
        init = params.X(traj).val(:,1);
        
        % different kinds of perturbation can be imposed. See below
        if 1
            %%%% PERTURBATION ON X0 WITH PERCENTAGE %%%%
            % define perturbation. Both the optimised and not variables are 
            % condiered. If a perturbation was to be considered on 
            % non-optimised variables, there would be a "non-estimated" 
            % dynamics which could hijack the estimation process. There 
            % could be situations in which this makes sense so try it, but  
            % at yourown risk
            params.perc = zeros(params.StateDim,params.Ntraj);
            
            % randomly define the percentage (bool flag, see below)
            randflag_opt = 0;
            randflag_nonopt = 0;
            noise_opt = 0;
            noise_nonopt = 0;
            
            % if case: random perturbation percentage - non optimised vars
            if noise_nonopt
                if randflag_nonopt
                    params.perc(params.nonopt_vars,traj) = randn(1,length(params.nonopt_vars))*1e-1;
                else
                    params.perc(params.nonopt_vars,traj) = ones(1,length(params.nonopt_vars))*1e-1;
                end
            end

            % if case: random perturbation percentage - optimised vars
            if noise_opt
                if randflag_opt
                    params.perc(params.opt_vars,traj) = 1*randn(1,length(params.opt_vars))*1e-1;
                else
                    params.perc(params.opt_vars,traj) = 1*ones(1,length(params.opt_vars))*1e-1;
                end
            end
            

            % final setup on perc
            params.perc = 1*params.perc;
            
            params.X_est(traj).val(:,1) = init;
            noise_std = 0*5e-2;
            
            if params.noise            
                
                if noise_opt
                    % around init
                    params.X_est(traj).val(params.opt_vars,1) = init(params.opt_vars).*(1 + params.noise*params.perc(params.opt_vars,traj).*ones(length(params.opt_vars),1)) + params.noise*noise_std.*randn(length(params.opt_vars),1);
                    
                    % around 0
%                     params.X_est(traj).val(params.opt_vars,1) =  params.noise*params.perc(params.opt_vars,traj).*init(params.opt_vars) + params.noise*noise_std.*randn(length(params.opt_vars),1);
                end
                
                if noise_nonopt
                    % around init
                    params.X_est(traj).val(params.multi_traj_var,1) = init(params.multi_traj_var).*(1 + params.noise*params.perc(params.multi_traj_var,traj).*ones(length(params.multi_traj_var),1)) + params.noise*noise_std.*randn(length(params.multi_traj_var),1);
                    
                    % around 0
%                     params.X_est(traj).val(params.multi_traj_var,1) =  params.noise*params.perc(params.multi_traj_var,traj).*init(params.multi_traj_var) + params.noise*noise_std.*randn(length(params.multi_traj_var),1);
                end               
                
            end
        else
        
            %%%% PERTURBATION ON X0 WITH RANDOM NOISE %%%%
            % define noise boundaries (uniform distribution)
            params.bound_delta_x = params.noise*1e-1*[-1,1]*1;
            params.bound_delta_x_dot = params.noise*1e-1*[-1,1]*1;

            % define the perturbation on the initial condition
            noise = [unifrnd(params.bound_delta_x(1),params.bound_delta_x(2),2,1); unifrnd(params.bound_delta_x_dot(1),params.bound_delta_x_dot(2),2,1)];

            % final setup on initial condition
            params.X_est(traj).val(:,1) = init;
            params.X_est(traj).val(params.nonopt_vars,1) = params.X(traj).val(params.nonopt_vars,1) + params.noise*noise;
        end

        
    end
    
end