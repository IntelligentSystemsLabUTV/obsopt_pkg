%% MODEL_INIT
% file: model_init.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function defines the model parameters. It's a general 
% method, depending only on the Params_init.m script, defining the 
% model variables and costants. A default case is set for each entry, but
% please check before running the code. 
% INPUT: no input
% OUTPUT: 
% Params: structure with all the necessary parameter to the model
function Params = model_init(varargin)

    % get Params_init.m script
    if any(strcmp(varargin,'ParamsInit'))
        
        pos = find(strcmp(varargin,'ParamsInit'));
        Params_init = varargin{pos+1};

        if any(strcmp(varargin,'Ntraj'))
            pos = find(strcmp(varargin,'Ntraj'));
            Ntraj = varargin{pos+1};
            Params = Params_init(Ntraj);
        else
            Params = Params_init(1);
        end
        
    else
        error('No "Params" initialization function provided.')
    end

    % sampling time
    if any(strcmp(varargin,'Ts'))
        pos = find(strcmp(varargin,'Ts'));
        Ts = varargin{pos+1};
    else
        warning('setting default sampling time to 0.1s');
        Ts = 1e-1;
    end

    % set to Params
    Params.Ts = Ts;

    % start and finish time
    if any(strcmp(varargin,'T0'))
        pos = find(strcmp(varargin,'T0'));
        time_tmp = varargin{pos+1};
        T0 = time_tmp;
        
    else
        warning('setting default initial time to 0s');
        T0 = 0;   
    end

    % det to Params
    Params.T0 = T0;

    % start and finish time
    if any(strcmp(varargin,'Tend'))
        pos = find(strcmp(varargin,'Tend'));
        time_tmp = varargin{pos+1};
        Tend = time_tmp;
        
    else
        warning('setting default initial time to 1s');
        Tend = 1;   
    end

    % det to Params
    Params.Tend = Tend;

    % setup the structure as far as time and iterations are concerned
    Params.T = T0:Ts:Tend;
    Params.Niter = numel(Params.T);
    Params.Tspan = [0, Ts];
    
    % get model if exists. Default is a 1 dimension asymptotically stable
    % system.
    if any(strcmp(varargin,'Model'))
        pos = find(strcmp(varargin,'Model'));
        Params.Model = varargin{pos+1};
    else
        error('No integration model provided.');
    end

    % get measure if exists. Default measures the whole state
    if any(strcmp(varargin,'Measure'))
        pos = find(strcmp(varargin,'Measure'));
        Params.Measure = varargin{pos+1};
    else
        error('No measure model provided.');
    end
    
    % get measure if exists. Default measures the whole state
    if any(strcmp(varargin,'ParamsUpdate'))
        pos = find(strcmp(varargin,'ParamsUpdate'));
        Params.ParamsUpdate = varargin{pos+1};
    else
        warning('No update parameters provided. Setting to @null function')
        Params.ParamsUpdate = @(x,Params) 1;
    end
    
    % get the integration algorithm. Default is ode45
    if any(strcmp(varargin,'Ode'))
        pos = find(strcmp(varargin,'Ode'));
        Params.Ode = varargin{pos+1};
    else
        warning('No integration method provided. Setting to @ode45')
        Params.Ode = @ode45;
    end
    
    % get the integration settings
    Params.Odeset = odeset('RelTol',1e-3,'AbsTol',1e-6);
    
    % input law definition. Default is free evolution 
    if any(strcmp(varargin,'InputLaw'))
        pos = find(strcmp(varargin,'InputLaw'));
        if ~isempty(varargin{pos+1})
            Params.Input = varargin{pos+1};
        else
            warning('No control law provided. Setting to @null function')
            Params.Input = @(x,Params) zeros(Params.DimInput);
        end 
    end
    
end
