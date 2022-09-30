%% setup data for simulink model
function [obs, params, SimParams] = setup_model   

    % set initial and final time instant
    t0 = 0;
    tend = 10000;
    first_guess_flag = 0;
    
    % set sampling time
    Ts = 1e0;
    
    %%%% params init function %%%%
    params_init = @params_battery_tushar;    
    %%%% params update function %%%%
    params_update = @params_update_battery_tushar;    
    %%%% model function %%%%
    model = @model_battery_tushar;            
    %%%% measure function %%%%
    measure = @measure_battery_tushar;
    % measure = @measure_general;    
    
    % MHE
    Nw = 20;
    Nts = 100;
    
    %%%% filters %%%%
    [filter, filterScale, ~] = filter_define(Ts,Nts);
    
    %%%% integration method %%%%
    ode = @odeDD;
    
    %%%% input law %%%
    input_law = @control;        
    
    % setup params
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0, 'params_update', params_update, ...
            'model',model,'measure',measure,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
            'input_enable',1,'input_law',input_law,'params_init',params_init);
        
    %%%% observer init %%%%    
    obs = obsopt('DataType', 'measured', 'optimise', 1 , 'GlobalSearch', 0, 'MultiStart', 0, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 1, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'Jdot_thresh',0.9,'MaxIter',20, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 6, 'AdaptiveHist', [5e-4, 1e-3, 1e-3], ...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminsearch, 'terminal', 1, 'terminal_states', [11:18], 'terminal_weights', [1 1 1 1 1 1 1 1], 'terminal_normalise', 1, ...
          'ConPos', [1], 'LBcon', [0], 'UBcon', [1], 'Bounds', 0);

      
    %%%%%%%%%% SIMULINK STUFF %%%%%%%%%
    % load OCV and R data (for ECM in simulink)
    input_data = load('data/ECM_parameters.mat');
    params.input_time = input_data.Time;
    params.input_current = input_data.Current;
    params.input_OCV = input_data.OCV(2:end);
    params.input_soc = input_data.SOC(2:end)';
    params.input_R0 = input_data.R0(2:end);
    params.input_R1 = input_data.R1(2:end);
    params.input_C1 = input_data.C1(2:end);

    % set model temperature
    params.Temperature = 313.15;
    params.Tend = tend;

    % propagation model in simulink
    params.model_SIM = @model_battery_tushar_reference_SIM;
    
    %%%% first guess %%%%
    if first_guess_flag
        first_guess;                
    end
    
    % create the bus to be provided to simulink
    stack = [params.C_n; params.eta; params.input_soc; params.input_OCV; params.input_R0; params.input_R1; params.input_C1];
    SimParams = timeseries(stack.*ones(size(params.time)),params.time);
    
end