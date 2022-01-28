%% PLANT model and data
% model simulation
% plant data
% this function defines the model parameters. It's a general method,
% depending only on the parameters_init.m script, defining the model
% variables and costants. 
function params = model_init(varargin)

    % get params_init.m script
    if any(strcmp(varargin,'params_init'))
        pos = find(strcmp(varargin,'params_init'));
        params_init = varargin{pos+1};
        params = params_init();
    else
        params.X = 5;
    end
    
    % handle parameters setup (identification task)
    fields = fieldnames(params);
    if ~any(strcmp(fields,'estimated_params'))
        params.estimated_params = [];
    end
    
    % sample time
    if any(strcmp(varargin,'Ts'))
        pos = find(strcmp(varargin,'Ts'));
        Ts = varargin{pos+1};
    else
        Ts = 1e-1;
    end
    
    % starting and finish time
    if any(strcmp(varargin,'T0'))
        pos = find(strcmp(varargin,'T0'));
        time_tmp = varargin{pos+1};
        t0 = time_tmp(1);
        tend = time_tmp(2);
    else
        t0 = 0;
        tend = 1;
    end

    % setup the structure
    params.time = t0:Ts:tend;
    params.Ts = Ts;
    params.Niter = length(params.time);
    params.tspan = [0, Ts];
    
    % get noise if exists. Default is 0
    if any(strcmp(varargin,'noise'))
        pos = find(strcmp(varargin,'noise'));
        params.noise = varargin{pos+1};
    else
        params.noise = 0;
    end
    
    % get noise setup if exists. Default is mu=0, std=5e-2;
    if any(strcmp(varargin,'noise_spec'))
        pos = find(strcmp(varargin,'noise_spec'));
        noise_spec = varargin{pos+1};
        params.noise_mu = noise_spec(1);
        params.noise_std = noise_spec(2);
    else
        params.noise_mu = 0;
        params.noise_std = 5e-2;
    end
    
    % get state dimension. Default is 1
    if any(strcmp(varargin,'StateDim'))
        pos = find(strcmp(varargin,'StateDim'));
        params.StateDim = varargin{pos+1};
    else
        params.StateDim = 1;
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
        params.measure = @(x,params) x;
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
    
    % input enable
    if any(strcmp(varargin,'input_enable'))
        pos = find(strcmp(varargin,'input_enable'));
        params.input_enable = varargin{pos+1};
    else
        params.input_enable = 0;
    end
    
    % input 
    if any(strcmp(varargin,'dim_input'))
        pos = find(strcmp(varargin,'dim_input'));
        params.dim_input = varargin{pos+1};
    else
        params.dim_input = 1;
    end

    % input law 
    if any(strcmp(varargin,'input_law'))
        pos = find(strcmp(varargin,'input_law'));
        if ~isempty(varargin{pos+1})
            params.input = varargin{pos+1};
        else
            params.input = @(x,params) 0;
        end 
    end
           
    params.perc = zeros(params.StateDim,params.Ntraj);
    randflag = 0;
    for traj=1:params.Ntraj
        
        % define perturbation
        % non opt vars (state init)
        if randflag
            params.perc(params.nonopt_vars,traj) = 1*randn(1,length(params.nonopt_vars))*5e-1;
        else
            params.perc(params.nonopt_vars,traj) = 1*ones(1,length(params.nonopt_vars))*6e-1;
        end
        
        % opt vars (control/params)
        if randflag
            params.perc(params.opt_vars,traj) = 1*randn(1,length(params.opt_vars));
        else
            params.perc(params.opt_vars,traj) = 1*ones(1,length(params.opt_vars))*6e-1;
        end
        
        % final setup on perc
        params.perc = 0*params.perc;
        
        % init state
        init = params.X(traj).val(:,1);
        
        % pendulum params
%         init(1:6) = [0;0;0;0;0;0];
        % pendulum
        init(1:4) = [0;0;0;0];
        % oscillator
%         init(1:2) = [5e-2;1e-2];
        % runaway
%         init(1:3) = [0.5, 0.1, 1];
        % SQR - params
%         init(1:5) = [0.5, 0.1, 1, 0.2, 0.4];
        
        % init state
        params.X_est(traj).val(:,1) = init.*(1 + params.noise*params.perc(:,traj).*ones(params.StateDim,1)) + params.noise*params.noise_std*randn(params.StateDim,1);
        
    end
    
end