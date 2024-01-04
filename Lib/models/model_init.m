%% model_init
% file: model_init.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function defines the model parameters. It's a general 
%              method, depending only on the Params_init.m script, defining 
%              the model variables and costants. A default case is set for 
%              each entry, but please check before running the code. 
% INPUT: 
%           ParamsInit: params function, changes depending on the model
%           Ntraj (optional): number of trajectories (see ref) 
%           Ts (optional): model sampling time
%           T0 (optional): simulation start time
%           Tend (optional): simulation finish time
%           Model: model function used for the plant integration
%           Measure: measure function used for the plant output
%           ParamsUpdate (optional): function updating the model parameters
%           Ode (optional): numerical integration algorithm
%           
% OUTPUT: 
%           Params: structure with all the necessary parameter to the model
function Params = model_init(varargin)

    % get Params_init.m script. If not ofund, throw error
    if any(strcmp(varargin,'ParamsInit'))
        
        % find position in varargin of ParamsInit
        pos = find(strcmp(varargin,'ParamsInit'));

        % get the function handle. it is in the next position
        Params_init = varargin{pos+1};

        % ParamsInit has an optional parameter which is the number of
        % trajectories (see ref). here Ntraj is found if present and used
        % as input to ParamsInit. If not found, no problem, it's
        % automatically set to one
        if any(strcmp(varargin,'Ntraj'))

            % find position in varargin of Ntraj
            pos = find(strcmp(varargin,'Ntraj'));

            % the handle function is next in position
            Ntraj = varargin{pos+1};

            % Init Params structure with the Ntraj parameter
            Params = Params_init(Ntraj);

        else

            % Init Params structure with the Ntraj=1 (default)
            Params = Params_init(1);

        end
        
    else

        % you need ParamsInit
        error('No "Params" initialization function provided.')
    end

    % get the model sampling time. It's not mandatory, the default value is
    % 0.1s
    if any(strcmp(varargin,'Ts'))

        % get Ts position in varagin
        pos = find(strcmp(varargin,'Ts'));

        % Ts value is next in position
        Ts = varargin{pos+1};

    else

        % Init Ts with default value
        warning('setting default sampling time to 0.1s');
        Ts = 1e-1;

    end

    % set the value in Params
    Params.Ts = Ts;

    % get the simulation start time. It's not mandatory, the default value 
    % is 0s
    if any(strcmp(varargin,'T0'))

        % get T0 position in varargin
        pos = find(strcmp(varargin,'T0'));

        % T0 value is next in position
        T0 = varargin{pos+1};
        
    else

        % Init T0 with the default value
        warning('setting default initial time to 0s');
        T0 = 0;   

    end

    % set the value in Params
    Params.T0 = T0;

    % get the simulation finish time. It's not mandatory, the default
    % value is 1s
    if any(strcmp(varargin,'Tend'))

        % get Tend position in varargin 
        pos = find(strcmp(varargin,'Tend'));

        % Tend value is in next position
        Tend = varargin{pos+1};
        
    else

        % Init Tend with the default value
        warning('setting default initial time to 1s');
        Tend = 1;   
    end

    % set the value in Params
    Params.Tend = Tend;

    % setup the params structure with the time vector and iterations number
    Params.T = T0:Ts:Tend;              % time vector
    Params.Niter = numel(Params.T);     % number of iterations

    % set the single time-span. Used in the control law mainly
    Params.Tspan = [0, Ts];
    
    % get Model.m script. If not found, throw error
    if any(strcmp(varargin,'Model'))

        % find position in varargin of Model
        pos = find(strcmp(varargin,'Model'));

        % the handle function is next in position
        Params.Model = varargin{pos+1};

    else

        % Model is mandatory. If not found, throw an error
        error('No integration model provided.');
    end

    % get Measure.m script. If not found, throw error
    if any(strcmp(varargin,'Measure'))

        % find position in varargin of Measure
        pos = find(strcmp(varargin,'Measure'));

        % the handle function is next in position
        Params.Measure = varargin{pos+1};
    else

        % Measure is mandatory. If not found, throw an error
        error('No measure model provided.');

    end
    
    % get ParamsUpdate.m script. If not found, a default handle is used.
    % Specifically, a handle doing nothing. Please see ref for more info on
    % the update function. 
    if any(strcmp(varargin,'ParamsUpdate'))

        % find position in varargin of ParamsUpdate
        pos = find(strcmp(varargin,'ParamsUpdate'));

        % set the handle in Params
        Params.ParamsUpdate = varargin{pos+1};
    else

        % Init ParamsUpdate with the default value. In this case, it is a
        % handle function doing nothing. 
        warning('No update parameters provided. Setting to @null function')
        Params.ParamsUpdate = @(x,Params) 1;

    end
    
    % get the integration algorithm. If not found, set as default ode45
    if any(strcmp(varargin,'Ode'))

        % find position in varargin of Ode
        pos = find(strcmp(varargin,'Ode'));

        % set the handle in Params
        Params.Ode = varargin{pos+1};

    else

        % Init Ode with the default handle, which is ode45
        warning('No integration method provided. Setting to @ode45')
        Params.Ode = @ode45;
    end
    
    % set the integration settings
    Params.Odeset = odeset('RelTol',1e-3,'AbsTol',1e-6);
    
    % get the control law algorithm. If not found, set as default zero
    % control
    if any(strcmp(varargin,'InputLaw'))

        % find position in varargin of InputLaw
        pos = find(strcmp(varargin,'InputLaw'));
        
        % there could be an empty list (instead of removing the "InputLaw"
        % entry it is easier to just put an empty list)
        if ~isempty(varargin{pos+1})

            % set the handle in Params
            Params.Input = varargin{pos+1};
        else

            % Init InputLaw with the default value. In this case, it is a
            % handle function returning a vector of zeros. 
            warning('No control law provided. Setting to @null function')
            Params.Input = @(x,Params) zeros(Params.DimInput);
        end 
    end    
end
