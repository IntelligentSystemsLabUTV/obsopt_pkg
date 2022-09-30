%% setup data for simulink model
function [obs, params, SimParams] = setup_model_fast   

    % set initial and final time instant
    t0 = 0;
    tend = 2000;
    
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
    Nw = 12;
    Nts = 5;
    
    %%%% filters %%%%
    system("sed -i 's/fil1 = .*/fil1 = 0;/' Lib/measure/filter_define.m");
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
    terminal_states = [1:2];
    terminal_weights = 1*ones(size(terminal_states));
%     terminal_weights(3:end) = 0.1;

    obs = obsopt('DataType', 'measured', 'optimise', 1, 'GlobalSearch', 0, 'MultiStart', 0, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 1, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'Jdot_thresh',0.95,'MaxIter',20, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 3, 'AdaptiveHist', [1e-3, 5e-3, 5e-1], ...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminsearch, 'terminal', 1, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
          'ConPos', [1], 'LBcon', [0], 'UBcon', [1], 'Bounds', 0);
      
    % update vars for multi MHE
    obs.setup.update_vars = [8:10 12:14 16:18 20:22];

      
    %%%%%%%%%% SIMULINK STUFF %%%%%%%%%
    % load OCV and R data (for ECM in simulink)
    input_data = load('data/ECM_parameters_updated.mat');
    params.input_time = input_data.Time;
    params.input_current = input_data.Current;
    params.input_OCV = reshape(input_data.OCV(2:end),length(input_data.OCV(2:end)),1);
    params.input_soc = reshape(input_data.SOC(2:end),length(input_data.SOC(2:end)),1);
    params.input_R0 = reshape(input_data.R0(2:end),length(input_data.R0(2:end)),1);
    params.input_R1 = reshape(input_data.R1(2:end),length(input_data.R1(2:end)),1);
    params.input_C1 = reshape(input_data.C1(2:end),length(input_data.C1(2:end)),1);
    params.LengthFit = length(input_data.SOC(2:end));

    % set model temperature
    params.Temperature = 313.15;
    params.Tend = tend;

    % propagation model in simulink
    params.model_SIM = @model_battery_tushar_reference_SIM;
    
    % create the bus to be provided to simulink
    stack = [params.C_n; params.eta; params.input_soc; params.input_OCV; params.input_R0; params.input_R1; params.input_C1];
    SimParams = timeseries(stack.*ones(size(params.time)),params.time);
    
end